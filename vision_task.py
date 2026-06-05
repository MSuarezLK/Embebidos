# =============================================================================
# vision_task.py — ESP32-CAM + ArUco FASTCAM
# Enfoque:
#   - Un solo lector del stream ESP32-CAM.
#   - Dashboard lee frame.jpg desde el broker, no directo desde la cámara.
#   - La imagen para la web se actualiza rápido.
#   - ArUco se procesa a frecuencia limitada para no matar FPS.
#   - Publicación PubSub reducida para no inundar broker/dashboard.
# =============================================================================
import asyncio
import threading
import time
import cv2
import cv2.aruco as aruco
import urllib.request
import numpy as np
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

ESP32_CAM_URL = "http://192.168.1.1:81/stream"
HTTP_FRAME_PORT = 8089

# Calibración visual dinámica para cámara lateral izquierda.
TARGET_OFFSET_FAR_PX = -35     # cerca de 20.5 cm, antes de bajar horquilla
TARGET_OFFSET_NEAR_PX = -65    # cerca de 14.6 cm, listo para alzar
DIST_PREPARAR_CM = 20.5
DIST_INSERTAR_CM = 14.6
ALIGN_TOL_PX = 30

# Rendimiento
DETECT_INTERVAL_S = 0.12       # ArUco máx ~8 Hz. La imagen web puede ir más rápido.
PUB_DETECT_INTERVAL_S = 0.18   # PubSub visión máx ~5.5 Hz si detecta.
PUB_LOST_INTERVAL_S = 0.75     # Sin tag, no inundar.
JPEG_QUALITY = 68              # Baja carga de red/HTTP local.
MAX_WEB_WIDTH = 360            # Imagen servida al dashboard.
STREAM_TIMEOUT_S = 3


def target_offset_para_dist(dist_cm):
    try:
        d = float(dist_cm)
    except Exception:
        return TARGET_OFFSET_FAR_PX
    if d >= DIST_PREPARAR_CM:
        return TARGET_OFFSET_FAR_PX
    if d <= DIST_INSERTAR_CM:
        return TARGET_OFFSET_NEAR_PX
    k = (DIST_PREPARAR_CM - d) / max(0.1, (DIST_PREPARAR_CM - DIST_INSERTAR_CM))
    return TARGET_OFFSET_FAR_PX + k * (TARGET_OFFSET_NEAR_PX - TARGET_OFFSET_FAR_PX)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def make_handler(vision_instance):
    class SnapshotHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith('/frame.jpg'):
                with vision_instance.lock:
                    frame_bytes = vision_instance.output_jpg
                if frame_bytes:
                    self.send_response(200)
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame_bytes))
                    self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                    self.send_header('Pragma', 'no-cache')
                    self.end_headers()
                    try:
                        self.wfile.write(frame_bytes)
                    except Exception:
                        pass
                else:
                    self.send_response(404)
                    self.end_headers()
            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, format, *args):
            pass
    return SnapshotHandler


class VisionTask:
    def __init__(self, pubsub, prefix, camera_url=None, **kwargs):
        self.pubsub = pubsub
        self.prefix = prefix
        self.url = camera_url or ESP32_CAM_URL
        self.loop = asyncio.get_running_loop()

        self.prefix_limpio = self.prefix if self.prefix.endswith('/') else self.prefix + '/'
        self.car_topic = self.prefix_limpio + "car/cmd"
        self.vision_topic = self.prefix_limpio + "vision/estado"

        self.latest_jpg = None
        self.output_jpg = None
        self.lock = threading.Lock()

        self.busqueda_activa = False  # Arranca OFF siempre.
        self.marcador_antes_visible = False
        self.fps = 0.0
        self.frame_counter = 0

        self.MARKER_SIZE_CM = 5.0
        self.FOCAL_LENGTH_PX = 285.0
        self.D_DISTANCIA_CAMARA_FRENTE = 0.0

        self.telemetria = self._estado_base()
        self.last_detection = self.telemetria.copy()
        self.last_detect_ts = 0.0
        self.last_pub_ts = 0.0
        self.last_lost_pub_ts = 0.0
        self.last_stream_error_print = 0.0

        self._crear_detector_rapido()

        threading.Thread(target=self._stream_reader, daemon=True).start()
        threading.Thread(target=self._image_processor, daemon=True).start()
        # Auto-control viejo deshabilitado: la misión PC decide; VisionTask solo ve.
        # threading.Thread(target=self._pulse_trajectory_brain, daemon=True).start()

        self.http_server = ThreadedHTTPServer(('0.0.0.0', HTTP_FRAME_PORT), make_handler(self))
        threading.Thread(target=self.http_server.serve_forever, daemon=True).start()

        print("[VIS] VisionTask FASTCAM activo. Auto visión inicia OFF.")
        print("[VIS] Cámara:", self.url)
        print("[VIS] Frame HTTP: http://0.0.0.0:{}/frame.jpg".format(HTTP_FRAME_PORT))

    def _estado_base(self):
        return {
            "detectado": False,
            "alineado": False,
            "offset_x": 0,
            "error_x": 0,
            "target_offset_x": TARGET_OFFSET_FAR_PX,
            "distancia_cm": 0.0,
            "angulo_yaw": 0.0,
            "ultimo_avistamiento": 0.0,
            "activo": self.busqueda_activa if hasattr(self, 'busqueda_activa') else False,
            "id": None,
            "marker_id": None,
            "diccionario": "DICT_4X4_50",
            "variante": "fast_gray",
            "fps": 0.0,
            "cx": 0,
            "cy": 0,
        }

    def _crear_detector_rapido(self):
        self.dict_name = "DICT_4X4_50"
        self.diccionario = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        params = aruco.DetectorParameters()
        try:
            # Menos pesado que la versión robusta multipaso.
            params.adaptiveThreshWinSizeMin = 5
            params.adaptiveThreshWinSizeMax = 35
            params.adaptiveThreshWinSizeStep = 10
            params.minMarkerPerimeterRate = 0.02
            params.maxMarkerPerimeterRate = 4.0
            params.polygonalApproxAccuracyRate = 0.055
            params.minCornerDistanceRate = 0.03
            params.minDistanceToBorder = 2
            # SUBPIX mejora precisión pero puede costar. Déjalo en NONE para ganar FPS.
            params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE
        except Exception:
            pass
        self.parametros = params
        try:
            self.detector = aruco.ArucoDetector(self.diccionario, self.parametros)
        except AttributeError:
            self.detector = None

    def activar_busqueda(self, on: bool):
        self.busqueda_activa = bool(on)
        if not self.busqueda_activa:
            self._enviar_comando_carro({"action": "detener"})
        with self.lock:
            self.telemetria["activo"] = self.busqueda_activa
        self._publish_vision(self.telemetria.copy())
        print("[VIS] Auto visión", "ON" if self.busqueda_activa else "OFF")

    def _stream_reader(self):
        while True:
            try:
                req = urllib.request.urlopen(self.url, timeout=STREAM_TIMEOUT_S)
                bytes_data = b''
                print("[VIS] Stream conectado")
                while True:
                    chunk = req.read(8192)
                    if not chunk:
                        raise RuntimeError("stream vacío")
                    bytes_data += chunk
                    while True:
                        start = bytes_data.find(b'\xff\xd8')
                        end = bytes_data.find(b'\xff\xd9', start + 2)
                        if start != -1 and end != -1:
                            jpg = bytes_data[start:end + 2]
                            with self.lock:
                                # Solo conserva el último frame; no hace cola.
                                self.latest_jpg = jpg
                            bytes_data = bytes_data[end + 2:]
                        else:
                            break
                    if len(bytes_data) > 250000:
                        bytes_data = b''
            except Exception as e:
                now = time.time()
                if now - self.last_stream_error_print > 2.5:
                    self.last_stream_error_print = now
                    print("[VIS] Stream error:", e)
                time.sleep(0.35)

    def _detectar_aruco_fast(self, frame):
        # Detectar sobre gris es más liviano y suficiente para Tag claro.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        try:
            if self.detector is not None:
                esquinas, ids, _ = self.detector.detectMarkers(gray)
            else:
                esquinas, ids, _ = aruco.detectMarkers(gray, self.diccionario, parameters=self.parametros)
        except Exception:
            return None, None
        return esquinas, ids

    def _procesar_deteccion(self, frame, now):
        h_f, w_f = frame.shape[:2]
        esquinas, ids = self._detectar_aruco_fast(frame)
        estado = self._estado_base()
        estado["activo"] = self.busqueda_activa
        estado["fps"] = round(self.fps, 1)

        if ids is None or len(ids) == 0:
            with self.lock:
                self.telemetria["detectado"] = False
                self.telemetria["alineado"] = False
                self.telemetria["activo"] = self.busqueda_activa
                self.telemetria["fps"] = round(self.fps, 1)

            if self.marcador_antes_visible or (now - self.last_lost_pub_ts > PUB_LOST_INTERVAL_S):
                self.last_lost_pub_ts = now
                self.marcador_antes_visible = False
                self._publish_vision(estado)
            return estado, esquinas, ids

        pts = esquinas[0][0]
        marker_id = int(ids[0][0])
        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))
        offset_x = cx - (w_f // 2)

        ancho_px = (np.linalg.norm(pts[0] - pts[1]) + np.linalg.norm(pts[3] - pts[2])) / 2.0
        distancia_cm = 0.0
        if ancho_px > 1:
            distancia_cm = (self.MARKER_SIZE_CM * self.FOCAL_LENGTH_PX) / ancho_px

        target_offset = target_offset_para_dist(distancia_cm)
        error_x = offset_x - target_offset
        alineado = abs(error_x) <= ALIGN_TOL_PX

        alto_izq = np.linalg.norm(pts[0] - pts[3])
        alto_der = np.linalg.norm(pts[1] - pts[2])
        angulo_yaw = (alto_izq - alto_der) * 2.5

        estado.update({
            "detectado": True,
            "alineado": alineado,
            "offset_x": int(offset_x),
            "error_x": int(error_x),
            "target_offset_x": round(float(target_offset), 1),
            "distancia_cm": round(float(distancia_cm), 1),
            "angulo_yaw": round(float(angulo_yaw), 1),
            "ultimo_avistamiento": now,
            "id": marker_id,
            "marker_id": marker_id,
            "cx": cx,
            "cy": cy,
        })

        self.marcador_antes_visible = True
        self.last_detect_ts = now
        self.last_detection = estado.copy()
        with self.lock:
            self.telemetria = estado.copy()

        if now - self.last_pub_ts >= PUB_DETECT_INTERVAL_S:
            self.last_pub_ts = now
            self._publish_vision(estado)

        return estado, esquinas, ids

    def _draw_overlay(self, frame, estado):
        h_f, w_f = frame.shape[:2]
        # Cruz central y línea objetivo compensada.
        cv2.line(frame, (w_f // 2, 0), (w_f // 2, h_f), (160, 160, 160), 1)
        target_x = int(w_f // 2 + float(estado.get("target_offset_x", TARGET_OFFSET_FAR_PX)))
        cv2.line(frame, (target_x, 0), (target_x, h_f), (255, 180, 0), 1)

        if estado.get("detectado"):
            cx = int(estado.get("cx", 0))
            cy = int(estado.get("cy", 0))
            color = (0, 220, 0) if estado.get("alineado") else (0, 0, 255)
            cv2.circle(frame, (cx, cy), 5, color, -1)
            cv2.line(frame, (target_x, cy), (cx, cy), (255, 0, 0), 2)
            txt = "ID:{} off:{} err:{} d:{}cm fps:{}".format(
                estado.get("id", "--"),
                estado.get("offset_x", "--"),
                estado.get("error_x", "--"),
                estado.get("distancia_cm", "--"),
                estado.get("fps", "--")
            )
            cv2.putText(frame, txt, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2)
        else:
            cv2.putText(frame, "SIN TAG  fps:{}".format(estado.get("fps", "--")),
                        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 220), 2)

    def _image_processor(self):
        last_ts = time.time()
        last_detect_run = 0.0
        estado_para_dibujo = self._estado_base()

        while True:
            time.sleep(0.004)
            with self.lock:
                jpg = self.latest_jpg
                self.latest_jpg = None

            if jpg is None:
                continue

            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                continue

            now = time.time()
            dt = now - last_ts
            last_ts = now
            if dt > 0:
                self.fps = 0.90 * self.fps + 0.10 * (1.0 / dt)

            # Corre ArUco solo cada DETECT_INTERVAL_S. La web se actualiza con último estado.
            if now - last_detect_run >= DETECT_INTERVAL_S:
                last_detect_run = now
                estado_para_dibujo, _, _ = self._procesar_deteccion(frame, now)
            else:
                # Mantener fps fresco en overlay sin publicar.
                estado_para_dibujo = self.last_detection.copy() if self.last_detection else self._estado_base()
                estado_para_dibujo["fps"] = round(self.fps, 1)
                if now - self.last_detect_ts > 0.55:
                    estado_para_dibujo["detectado"] = False
                    estado_para_dibujo["alineado"] = False

            self._draw_overlay(frame, estado_para_dibujo)

            # Reducir tamaño servido al navegador si hace falta.
            h, w = frame.shape[:2]
            if w > MAX_WEB_WIDTH:
                scale = MAX_WEB_WIDTH / float(w)
                frame = cv2.resize(frame, (MAX_WEB_WIDTH, int(h * scale)), interpolation=cv2.INTER_AREA)

            ok, encoded_img = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if ok:
                with self.lock:
                    self.output_jpg = encoded_img.tobytes()

    def _publish_vision(self, data):
        try:
            asyncio.run_coroutine_threadsafe(self.pubsub.publish(self.vision_topic, data), self.loop)
        except Exception as e:
            print("[VIS] publish error:", e)

    def _enviar_comando_carro(self, payload):
        # Se mantiene solo por compatibilidad con Auto visión OFF/ON.
        try:
            asyncio.run_coroutine_threadsafe(self.pubsub.publish(self.car_topic, payload), self.loop)
        except Exception as e:
            print("[VIS] car cmd error:", e)
