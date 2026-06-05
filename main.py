import network
import time
import gc
import usocket as socket
import rp2

try:
    import ujson as json
except:
    import json

from machine import Pin, SoftI2C

from bateria import Bateria
from distancia import SensorDistancia
from pantalla import PantallaOLED
from carro import Carro
from brazo import Brazo


# =========================================================
# CONFIG ROBOT10
# =========================================================
ROBOT_NAME = "robot10"
PREFIX = "UDFJC/emb1/{}/".format(ROBOT_NAME)

SSID = "Core"
PASSWORD = "core2026"

BROKER_IP = "192.168.1.10"
BROKER_PORT = 5051

PIN_IR = 22
PIN_PIO_SET = 15


# =========================================================
# SCHEDULER
# =========================================================
class Task:
    def __init__(self, scheduler, period_ms=1000, priority=5):
        self.period = period_ms
        self.priority = priority
        self.next_run = time.ticks_ms()
        scheduler.add(self)

    def update(self):
        pass


class Scheduler:
    def __init__(self):
        self.tasks = []

    def add(self, task):
        self.tasks.append(task)
        self.tasks.sort(key=lambda t: t.priority)

    def run(self):
        print("Scheduler corriendo...")
        while True:
            now = time.ticks_ms()

            for task in self.tasks:
                # Verificar si ya es hora de ejecutar la tarea
                if time.ticks_diff(now, task.next_run) >= 0:
                    task.update()
                    
                    # 🛡️ CORRECCIÓN: Programar la siguiente ejecución sumando el período
                    task.next_run = time.ticks_add(now, task.period)
            
            # Pequeño micro-descanso para evitar que la Pico W consuma el 100% de CPU en un bucle vacío
            time.sleep_ms(2)

# =========================================================
# WIFI
# =========================================================
class WiFiManager:
    def __init__(self, ssid, password):
        self.ssid = ssid
        self.password = password
        self.wlan = network.WLAN(network.STA_IF)
        self.wlan.active(True)

    def connect(self):
        print("Conectando WiFi...")
        
        self.wlan.ifconfig((
            "192.168.1.2",
            "255.255.255.0",
            "192.168.1.253",
            "8.8.8.8"
        ))

        if not self.wlan.isconnected():
            self.wlan.connect(self.ssid, self.password)

            t0 = time.ticks_ms()
            while not self.wlan.isconnected():
                print("wlan.status:", self.wlan.status())

                if time.ticks_diff(time.ticks_ms(), t0) > 25000:
                    raise RuntimeError("No conectó al WiFi")

                time.sleep(1)

        print("WiFi OK:", self.wlan.ifconfig())


# =========================================================
# SOCKET TCP CLIENT
# =========================================================
class SocketClient(Task):
    def __init__(self, scheduler, host, port, period_ms=35):
        super().__init__(scheduler, period_ms, priority=1)
        self.host = host
        self.port = port
        self.sock = None
        self.rx = b""
        self.actions = {}
        self.last_retry = 0
        self.on_connect_callbacks = []

    def add_action(self, action, callback):
        self.actions[action] = callback

    def add_on_connect(self, callback):
        self.on_connect_callbacks.append(callback)

    def connected(self):
        return self.sock is not None

    def connect(self):
        if self.sock is not None:
            return True

        now = time.ticks_ms()
        if time.ticks_diff(now, self.last_retry) < 2000:
            return False

        self.last_retry = now

        try:
            print("Conectando broker", self.host, self.port)
            addr = socket.getaddrinfo(self.host, self.port)[0][-1]
            s = socket.socket()
            s.connect(addr)
            s.setblocking(False)
            self.sock = s
            print("Broker OK")

            for cb in self.on_connect_callbacks:
                try:
                    cb()
                except Exception as e:
                    print("on_connect error:", e)

            return True

        except Exception as e:
            print("No se pudo conectar broker:", e)
            try:
                s.close()
            except:
                pass
            self.sock = None
            return False

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except:
            pass
        self.sock = None

    def send(self, data):
        if self.sock is None:
            return False

        try:
            self.sock.send(data)
            return True
        except Exception as e:
            print("Socket send error:", e)
            self.close()
            return False

    def send_json(self, obj):
        return self.send((json.dumps(obj) + "\n").encode())

    def recv_json(self):
        msgs = []

        if self.sock is None:
            return msgs

        try:
            data = self.sock.recv(2048)

            if data:
                self.rx += data
            elif data == b"":
                self.close()
                return msgs

        except OSError:
            return msgs
        except Exception:
            self.close()
            return msgs

        while b"\n" in self.rx:
            line, self.rx = self.rx.split(b"\n", 1)

            if not line:
                continue

            try:
                msgs.append(json.loads(line.decode()))
            except Exception as e:
                print("JSON malo:", e, line)

        return msgs

    def update(self):
        if self.sock is None:
            self.connect()
            return

        msgs = self.recv_json()

        for msg in msgs:
            action = msg.get("action")
            cb = self.actions.get(action)

            if cb:
                cb(msg)


# =========================================================
# NODE PUB/SUB LOCAL + BROKER
# =========================================================
class Node:
    def __init__(self, socket_client, prefix):
        self.sock = socket_client
        self.prefix = prefix
        self.local_subs = {}
        self.remote_subs = set()

        self.sock.add_action("PUB", self._handle_pub)
        self.sock.add_on_connect(self._resubscribe_all)

    def topic_full(self, topic):
        if topic.startswith(self.prefix):
            return topic
        return self.prefix + topic

    def publish(self, topic, data):
        full = self.topic_full(topic)

        pkt = {
            "action": "PUB",
            "topic": full,
            "data": data
        }

        self.sock.connect()
        self.sock.send_json(pkt)
        self._local_publish(topic, data)

    def subscribe(self, topic, callback):
        self.local_subs.setdefault(topic, []).append(callback)
        self.remote_subs.add(topic)

        print("SUB", topic)

        if self.sock.connected():
            self._send_sub(topic)

    def _send_sub(self, topic):
        pkt = {
            "action": "SUB",
            "topic": self.topic_full(topic)
        }
        self.sock.send_json(pkt)

    def _resubscribe_all(self):
        for topic in self.remote_subs:
            self._send_sub(topic)

    def _local_publish(self, topic, data):
        callbacks = self.local_subs.get(topic, [])

        for cb in callbacks:
            try:
                cb(data)
            except Exception as e:
                print("Callback error:", topic, e)

    def _handle_pub(self, msg):
        full = msg.get("topic", "")

        if not full.startswith(self.prefix):
            return

        topic = full[len(self.prefix):]
        data = msg.get("data", {})
        self._local_publish(topic, data)


# =========================================================
# IR PIO
# =========================================================
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def ir_rx():
    label('inicio')
    mov(x, invert(null))
    jmp(pin, 'uno')

    label('cero')
    set(pins, 0)
    jmp(x_dec, 'cero_bis')
    jmp('fin')
    label('cero_bis')
    jmp(pin, 'fin')
    jmp('cero')

    label('uno')
    set(pins, 1)
    jmp(x_dec, 'uno_bis')
    jmp('fin')
    label('uno_bis')
    nop()
    jmp(pin, 'uno')

    label('fin')
    mov(isr, x)
    push(noblock)
    irq(0)
    jmp('inicio')


class IRRemote:
    def __init__(self, pin_ir=PIN_IR):
        self.sm = rp2.StateMachine(
            0,
            ir_rx,
            freq=38000 * 100,
            set_base=Pin(PIN_PIO_SET),
            jmp_pin=Pin(pin_ir, Pin.IN, Pin.PULL_UP)
        )

        self.keys = {
            0x00FF6897: '0',
            0x00FF30CF: '1',
            0x00FF18E7: '2',
            0x00FF7A85: '3',
            0x00FF10EF: '4',
            0x00FF38C7: '5',
            0x00FF5AA5: '6',
            0x00FF42BD: '7',
            0x00FF4AB5: '8',
            0x00FF52AD: '9',
            0x00FF9867: '*',
            0x00FFB04F: '#'
        }

        self.frame_gap = 50000
        self.bit_thr = 1000
        self.frame = []
        self.started = False
        self.ready_key = None
        self.ready_code = None

        self.sm.active(1)
        self.sm.irq(self._on_irq)

    def _decode_frame(self, frame):
        if len(frame) < 66:
            return None, None

        odds = frame[2:66][1::2]

        if len(odds) < 32:
            return None, None

        bits = ''.join('1' if t > self.bit_thr else '0' for t in odds[:32])
        code = int(bits, 2)
        key = self.keys.get(code, '?')

        return key, code

    def _on_irq(self, sm):
        while sm.rx_fifo():
            dat = sm.get()
            interval = (2**32 - dat)

            if (not self.started) and interval > self.frame_gap:
                continue

            self.started = True

            if interval > self.frame_gap:
                if self.frame:
                    key, code = self._decode_frame(self.frame)
                    if key is not None:
                        self.ready_key = key
                        self.ready_code = code
                self.frame = []
            else:
                self.frame.append(interval)

    def get_key(self):
        if self.ready_key is None:
            return None, None

        k = self.ready_key
        c = self.ready_code

        self.ready_key = None
        self.ready_code = None

        return k, c


# =========================================================
# TASKS SENSORES / OLED (VERSIÓN OPTIMIZADA ANTI-LAG)
# =========================================================
class BatteryTask(Task):
    def __init__(self, scheduler, node, bateria):
        # ⏱️ OPTIMIZACIÓN: Cambiado de 3000ms a 15000ms (15 segundos). La batería baja muy lento.
        super().__init__(scheduler, 15000, priority=8)
        self.node = node
        self.bateria = bateria
        self.ultimo_p = -1

    def update(self):
        p = self.bateria.obtener() # Método real de tu compañero
        
        self.node.publish("sensor/bateria", {
            "robot": ROBOT_NAME,
            "porcentaje": p
        })
        self.ultimo_p = p


class DistanceTask(Task):
    def __init__(self, scheduler, node, sensor):
        # ⏱️ OPTIMIZACIÓN: Regulado a 400ms para dar respiro al WiFi del Broker
        super().__init__(scheduler, 1200, priority=4)
        self.node = node
        self.sensor = sensor
        self.ultima_distancia = -999

    def update(self):
        d = self.sensor.obtener_rapido() # ¡CORREGIDO!: Método real de tu compañero
        
        self.node.publish("sensor/distancia", {
            "robot": ROBOT_NAME,
            "mm": d
        })
        self.ultima_distancia = d


class DisplayTask(Task):
    def __init__(self, scheduler, node, pantalla):
        super().__init__(scheduler, 700, priority=7)
        self.node = node
        self.pantalla = pantalla
        self.percent = 0
        self.distancia = None
        self.view = "completo"
        self.node.subscribe("sensor/bateria", self.on_battery)
        self.node.subscribe("sensor/distancia", self.on_distance)
        self.node.subscribe("oled/show", self.on_cmd)

    def on_battery(self, data):
        if "porcentaje" in data: self.percent = data["porcentaje"]

    def on_distance(self, data):
        self.distancia = data.get("mm", self.distancia)

    def on_cmd(self, data):
        self.view = data.get("mode", "completo")

    def update(self):
        # Si la pantalla está desactivada por hardware (comentada en tu Init), esto no romperá el código
        try:
            if self.view == "bateria":
                self.pantalla.mostrar_bateria(self.percent)
            elif self.view == "distancia":
                self.pantalla.mostrar_distancia(self.distancia)
            else:
                self.pantalla.mostrar_completo(self.percent, self.distancia)
        except:
            pass
            
# =========================================================
# CONTROL CARRO
# =========================================================
class CarController:
    def __init__(self, node, carro):
        self.node = node
        self.carro = carro
        self.modo = "detenido"
        self.last_cmd_ms = time.ticks_ms()

        self.node.subscribe("car/cmd", self.on_cmd)

    def _set_modo(self, modo):
        self.modo = modo
        self.last_cmd_ms = time.ticks_ms()
        self.publish_state("cmd")

    def on_cmd(self, data):
        action = data.get("action", "")

        try:
            if action == "detener":
                self.carro.detener()
                self._set_modo("detenido")

            elif action == "adelante":
                self._set_modo("adelante")
                self.carro.web_adelante()

            elif action == "atras":
                self._set_modo("atras")
                self.carro.web_atras()

            elif action == "izquierda":
                self._set_modo("izquierda")
                self.carro.web_izquierda()

            elif action == "derecha":
                self._set_modo("derecha")
                self.carro.web_derecha()

            elif action == "recta":
                self._set_modo("recta")
                vel = float(data.get("velocidad", 0.25))
                tiempo = float(data.get("tiempo", 2.0))
                self.carro._configurar_direccion(True)
                self.carro.recta(vel, tiempo)
                self.carro.detener()
                self._set_modo("detenido")

            elif action == "girar":
                self._set_modo("girar")
                direccion = data.get("direccion", "derecha")
                vel = float(data.get("velocidad", 0.10))
                tiempo = float(data.get("tiempo", 1.0))
                self.carro.girar(direccion, vel, tiempo)
                self.carro.detener()
                self._set_modo("detenido")

            elif action == "circulo":
                self._set_modo("circulo")
                radio = float(data.get("radio", 0.20))
                direccion = data.get("direccion", "derecha")
                tiempo = float(data.get("tiempo", 3.0))
                vel = float(data.get("velocidad", 0.18))
                self.carro.circulo(radio, vel, direccion, tiempo)
                self.carro.detener()
                self._set_modo("detenido")

            else:
                self.publish_state("accion_no_reconocida")

        except Exception as e:
            self.node.publish("car/state", {
                "robot": ROBOT_NAME,
                "modo": "error",
                "error": str(e)
            })

    def publish_state(self, evento="estado"):
        try:
            enc = self.carro.estado_encoders()
        except:
            enc = (0, 0)

        try:
            params = self.carro.parametros_odometria()
        except:
            params = {}

        pwm = 0
        try:
            pwm = self.carro.pwm_web(self.modo)
        except:
            pwm = 0

        self.node.publish("car/state", {
            "robot": ROBOT_NAME,
            "modo": self.modo,
            "evento": evento,
            "pwm": pwm,
            "encoders": enc,
            "odom": params,
            "t_ms": time.ticks_ms()
        })


class CarStateTask(Task):
    def __init__(self, scheduler, controller):
        # Reporte rápido para que el gemelo use encoders casi en tiempo real.
        super().__init__(scheduler, 250, priority=3)
        self.controller = controller
        self.ultimos_encoders = None
        self.ultimo_modo = None

    def update(self):
        # Watchdog para control manual tipo D-pad: evita que una pulsación corta
        # se quede ejecutándose por lag, pérdida de pointerup o congestión WiFi.
        try:
            if self.controller.modo in ("adelante", "atras", "izquierda", "derecha"):
                if time.ticks_diff(time.ticks_ms(), self.controller.last_cmd_ms) > CAR_HOLD_TIMEOUT_MS:
                    self.controller.carro.detener()
                    self.controller.modo = "detenido"
                    self.controller.publish_state("watchdog_stop")
        except Exception as e:
            print("Car watchdog error:", e)

        try:
            enc = self.controller.carro.estado_encoders()
        except:
            enc = (0, 0)

        # Publica siempre mientras se mueve; en reposo solo cuando cambia algo.
        if self.controller.modo != "detenido":
            self.controller.publish_state("tick")
            self.ultimos_encoders = enc
            self.ultimo_modo = self.controller.modo
            return

        if enc != self.ultimos_encoders or self.controller.modo != self.ultimo_modo:
            self.controller.publish_state("tick")
            self.ultimos_encoders = enc
            self.ultimo_modo = self.controller.modo

# =========================================================
# CONTROL BRAZO
# =========================================================
class ArmController:
    def __init__(self, node, brazo):
        self.node = node
        self.brazo = brazo
        self.busy = False
        self.mission = None

        self.node.subscribe("arm/cmd", self.on_cmd)

    def _publish_moviendo(self):
        self.publish_state(True, "moviendo")

    def mission_activa(self):
        try:
            return self.mission is not None and self.mission.activa()
        except:
            return False

    def on_cmd(self, data):
        if self.busy:
            self.publish_state(False, "ocupado")
            return

        action = data.get("action", "")

        # Durante misión se bloquea XYZ/ángulos manuales para que no compitan.
        if self.mission_activa() and action not in ("emergencia", "stop", "estado"):
            self.publish_state(False, "bloqueado: misión activa")
            return

        self.busy = True
        self.publish_state(True, "ejecutando")
        ok = False
        msg = "ok"

        try:
            if action == "reposo":
                ok = self.brazo.reposo(on_step=self._publish_moviendo, publish_every=30)

            elif action == "recoger":
                ok = self.brazo.recoger(on_step=self._publish_moviendo, publish_every=30)

            elif action == "alzar":
                ok = self.brazo.alzar(on_step=self._publish_moviendo, publish_every=30)

            elif action == "angles":
                ok = self.brazo.mover_angulos(
                    data.get("base", 90),
                    data.get("brazo", 0),
                    data.get("antebrazo", 165),
                    data.get("muneca", 140),
                    duracion=float(data.get("duracion", 4.5)),
                    pasos=int(data.get("pasos", 150)),
                    on_step=self._publish_moviendo,
                    publish_every=30
                )

            elif action == "xyz":
                ok = self.brazo.mover_xyz(
                    data.get("x", 8.5),
                    data.get("y", 0),
                    data.get("z", 8.5),
                    data.get("phi", 0),
                    duracion=float(data.get("duracion", 3.8)),
                    pasos=int(data.get("pasos", 140)),
                    verbose=True,
                    on_step=self._publish_moviendo,
                    publish_every=30
                )

            elif action == "delta_xyz":
                ok = self.brazo.mover_delta_xyz(
                    dx=float(data.get("dx", 0)),
                    dy=float(data.get("dy", 0)),
                    dz=float(data.get("dz", 0)),
                    dphi=float(data.get("dphi", 0)),
                    duracion=float(data.get("duracion", 2.8)),
                    pasos=int(data.get("pasos", 100)),
                    on_step=self._publish_moviendo,
                    publish_every=30
                )

            elif action in ("emergencia", "stop"):
                self.brazo.emergencia()
                ok = True
                msg = "emergencia"

            elif action == "estado":
                ok = True
                msg = "estado"

            else:
                msg = "accion no reconocida"

        except Exception as e:
            msg = str(e)

        self.busy = False
        self.publish_state(ok, msg)

    def publish_state(self, ok=True, msg="ok"):
        try:
            s1, s2, s3, s4 = self.brazo.leer_servos()
            x, y, z = self.brazo.directa(s1, s2, s3, s4)
        except:
            s1, s2, s3, s4 = 0, 0, 0, 0
            x, y, z = 0, 0, 0

        self.node.publish("arm/joint_state", {
            "robot": ROBOT_NAME,
            "busy": self.busy,
            "ok": ok,
            "msg": msg,
            "base": round(s1, 2),
            "brazo": round(s2, 2),
            "antebrazo": round(s3, 2),
            "muneca": round(s4, 2),
            "x": round(x, 2),
            "y": round(y, 2),
            "z": round(z, 2)
        })


class ArmStateTask(Task):
    def __init__(self, scheduler, controller):
        super().__init__(scheduler, 4000, priority=7)
        self.controller = controller

    def update(self):
        self.controller.publish_state(True, "estado")


# =========================================================
# IR COMO CONTROL AUXILIAR
# =========================================================
class IRTask(Task):
    def __init__(self, scheduler, node, ir):
        super().__init__(scheduler, 80, priority=2)
        self.node = node
        self.ir = ir
        self.mode = "sin modo"

    def update(self):
        key, code = self.ir.get_key()

        if key is None:
            return

        hex_code = hex(code) if code is not None else "---"

        self.node.publish("ir/state", {
            "key": key,
            "hex": hex_code
        })

        if key == "*":
            self.mode = "car"
            self.node.publish("mode/state", {"mode": "car"})
            return

        if key == "#":
            self.mode = "arm"
            self.node.publish("mode/state", {"mode": "arm"})
            return

        if self.mode == "car":
            self._car_from_key(key)

        elif self.mode == "arm":
            self._arm_from_key(key)

    def _car_from_key(self, key):
        if key in ("4", "5", "6"):
            tiempos = {"4": 1.5, "5": 2.5, "6": 3.5}
            self.node.publish("car/cmd", {
                "action": "recta",
                "velocidad": 0.25,
                "tiempo": tiempos[key]
            })

        elif key in ("1", "2", "3"):
            radios = {"1": 0.15, "2": 0.25, "3": 0.35}
            self.node.publish("car/cmd", {
                "action": "circulo",
                "radio": radios[key],
                "velocidad": 0.18,
                "direccion": "izquierda",
                "tiempo": 2.0
            })

        elif key in ("7", "8", "9"):
            radios = {"7": 0.15, "8": 0.25, "9": 0.35}
            self.node.publish("car/cmd", {
                "action": "circulo",
                "radio": radios[key],
                "velocidad": 0.18,
                "direccion": "derecha",
                "tiempo": 2.0
            })

        elif key == "0":
            self.node.publish("car/cmd", {"action": "detener"})

    def _arm_from_key(self, key):
        if key == "0":
            self.node.publish("arm/cmd", {"action": "reposo"})

        elif key == "1":
            self.node.publish("arm/cmd", {
                "action": "xyz",
                "x": 8.5,
                "y": 0,
                "z": 8.5,
                "phi": 0
            })

        elif key == "2":
            self.node.publish("arm/cmd", {
                "action": "xyz",
                "x": 8.5,
                "y": 2,
                "z": 8.5,
                "phi": 0
            })

        elif key == "3":
            self.node.publish("arm/cmd", {
                "action": "xyz",
                "x": 8.5,
                "y": -2,
                "z": 8.5,
                "phi": 0
            })






# =========================================================
# MISION TASK — montacargas no bloqueante con ArUco + ToF
# =========================================================

# Poses testeadas por ángulos.
POS_RECOGER = (90, 160, 45, 140)
POS_ALZAR   = (90, 90, 120, 140)

# Distancias de estrategia.
VISION_RECOGER_CM = 20.0
DIST_OBJ_MIN_MM = 172
DIST_OBJ_MAX_MM = 178
DIST_RETROCESO_MM = 165
UMBRAL_PX = 25

# Pulsos suaves. La misión NO usa time.sleep(); enciende motor, guarda stop_at y retorna.
VEL_BUSQ = 0.065
VEL_AVANCE = 0.075
VEL_RETRO = 0.065
KP_GIRO = 0.00035

CAR_HOLD_TIMEOUT_MS = 380  # Si el dashboard deja de mandar keepalive, detener D-pad.


class MisionTask(Task):
    """
    Misión no bloqueante:
      IDLE → BUSCAR → ALINEAR → PREPARAR_RECOGER
           → APROXIMAR_FINAL → ALZAR → COMPLETADO

    Regla clave:
    - El control del carro en misión se hace con pulsos temporizados sin time.sleep().
    - La misión usa poses por ángulos probadas para recoger/alzar.
    - XYZ queda bloqueado mientras la misión está activa.
    """

    _NOMBRES = {
        "IDLE": "En reposo",
        "BUSCAR": "Buscando tag",
        "ALINEAR": "Alineando tag",
        "PREPARAR_RECOGER": "Preparando recoger",
        "APROXIMAR_FINAL": "Aproximación final",
        "ALZAR": "Alzando cubeta",
        "COMPLETADO": "Completado",
        "PAUSADO": "Pausado",
        "ERROR": "Error",
    }

    def __init__(self, scheduler, node, arm_ctrl, car_ctrl, sensor):
        # Más rápido que antes, pero sin bloquear. Esto mejora respuesta sin saturar.
        super().__init__(scheduler, 120, priority=5)
        self.node = node
        self.arm = arm_ctrl
        self.car = car_ctrl
        self.sensor = sensor
        self._estado = "IDLE"
        self._prev = "IDLE"
        self._modo = "auto"
        self._vision = {}
        self._ultimo_info = "Sistema listo"

        # Pulso de carro no bloqueante.
        self._pulse_action = None
        self._pulse_stop_ms = 0
        self._pulse_info = ""

        # Antispam de estado.
        self._last_pub_ms = 0
        self._last_pub_estado = None
        self._last_pub_info = None

        node.subscribe("vision/estado", self._on_vision)
        node.subscribe("mission/cmd", self._on_cmd)
        node.subscribe("mission/manual", self._on_manual)

        self._pub("Sistema listo", force=True)

    def activa(self):
        return self._estado not in ("IDLE", "COMPLETADO", "PAUSADO", "ERROR")

    def _on_vision(self, data):
        self._vision = data or {}

    def _on_cmd(self, data):
        action = data.get("action", "")

        if action == "iniciar":
            if self._estado in ("IDLE", "COMPLETADO", "ERROR"):
                self._cancelar_pulso()
                self._ir_a("BUSCAR", "Misión iniciada")
            else:
                self._pub("Misión ya activa")

        elif action == "pausar":
            if self._estado not in ("IDLE", "PAUSADO", "COMPLETADO", "ERROR"):
                self._pausar()

        elif action == "reanudar":
            if self._estado == "PAUSADO":
                self._reanudar()

        elif action == "abortar":
            self._abortar()

        elif action == "recoger":
            self._cancelar_pulso()
            self._ir_a("PREPARAR_RECOGER", "Forzado a recoger")

        elif action == "alzar":
            self._cancelar_pulso()
            self._ir_a("ALZAR", "Forzado a alzar")

    def _on_manual(self, data):
        # Manual dentro de misión: pausa primero y detiene cualquier pulso.
        if self._estado not in ("PAUSADO", "IDLE", "COMPLETADO", "ERROR"):
            self._pausar()

        ca = data.get("car")
        if ca and self.car:
            acciones = {
                "adelante": self.car.carro.web_adelante,
                "atras": self.car.carro.web_atras,
                "izquierda": self.car.carro.web_izquierda,
                "derecha": self.car.carro.web_derecha,
                "detener": self.car.carro.detener,
            }
            if ca in acciones:
                acciones[ca]()

        # XYZ manual queda para consola/main si se necesita; no se expone en dashboard.

    def update(self):
        # Primero atiende temporizador de pulso, sin dormir el scheduler.
        if self._pulse_action is not None:
            if time.ticks_diff(time.ticks_ms(), self._pulse_stop_ms) >= 0:
                self._stop_pulso()
            else:
                return

        if self._estado in ("IDLE", "PAUSADO", "COMPLETADO", "ERROR"):
            return

        try:
            if self._estado == "BUSCAR":
                self._buscar()
            elif self._estado == "ALINEAR":
                self._alinear()
            elif self._estado == "PREPARAR_RECOGER":
                self._preparar_recoger()
            elif self._estado == "APROXIMAR_FINAL":
                self._aproximar_final()
            elif self._estado == "ALZAR":
                self._alzar()
        except Exception as e:
            print("MisionTask exc:", e)
            self._parar_carro()
            self._ir_a("ERROR", str(e))

    # ---------------- Estados ----------------
    def _buscar(self):
        v = self._vision
        if not v or not v.get("detectado"):
            self._pulso_carro("derecha", 80, "Buscando tag")
            return

        self._parar_carro()
        if v.get("alineado") or abs(int(v.get("offset_x", 0))) <= UMBRAL_PX:
            if self._dist_vision_lista(v):
                self._ir_a("PREPARAR_RECOGER", "Tag alineado y cerca de 20 cm")
            else:
                self._ir_a("ALINEAR", "Tag detectado; acercando")
        else:
            self._ir_a("ALINEAR", "Tag detectado, falta alinear")

    def _alinear(self):
        v = self._vision
        if not v or not v.get("detectado"):
            self._parar_carro()
            self._ir_a("BUSCAR", "Tag perdido")
            return

        offset = int(v.get("offset_x", 0))
        if abs(offset) <= UMBRAL_PX:
            self._parar_carro()
            if self._dist_vision_lista(v):
                self._ir_a("PREPARAR_RECOGER", "Alineado y cerca de 20 cm")
            else:
                self._pulso_carro("adelante", 90, "Alineado, acercando hasta 20 cm")
            return

        dire = "derecha" if offset > 0 else "izquierda"
        t_ms = int(min(max(abs(offset) * KP_GIRO * 1000, 30), 80))
        self._pulso_carro(dire, t_ms, "Corrigiendo offset {} px".format(offset))

    def _preparar_recoger(self):
        self._parar_carro()
        if self.arm and self.arm.brazo:
            # No publica durante todos los pasos: reduce carga WiFi y gemelos.
            ok = self.arm.brazo.recoger(on_step=None, publish_every=999)
            self.arm.publish_state(ok, "recoger" if ok else "fallo recoger")
            if not ok:
                self._ir_a("ERROR", "No alcanzó posición recoger")
                return
        self._ir_a("APROXIMAR_FINAL", "Recoger listo; avance fino a 17.2–17.8 cm")

    def _aproximar_final(self):
        d = self._dist_mm()

        if d is None or d <= 0:
            self._parar_carro()
            self._pub("Sin distancia ToF válida")
            return

        if DIST_OBJ_MIN_MM <= d <= DIST_OBJ_MAX_MM:
            self._parar_carro()
            self._ir_a("ALZAR", "Distancia final OK: {} mm".format(d))
            return

        if d > DIST_OBJ_MAX_MM:
            self._pulso_carro("adelante", 65 if d > 210 else 35, "Avanzando fino: {} mm".format(d))
            return

        if d < DIST_RETROCESO_MM:
            self._pulso_carro("atras", 35, "Muy cerca, retrocediendo: {} mm".format(d))
            return

        self._parar_carro()
        self._pub("Cerca del rango, esperando ajuste: {} mm".format(d))

    def _alzar(self):
        self._parar_carro()
        if self.arm and self.arm.brazo:
            ok = self.arm.brazo.alzar(on_step=None, publish_every=999)
            self.arm.publish_state(ok, "alzar" if ok else "fallo alzar")
            if ok:
                self.node.publish("mission/done", {"exito": True})
                self._ir_a("COMPLETADO", "Cubeta alzada")
            else:
                self._ir_a("ERROR", "No alzó")
        else:
            self._ir_a("ERROR", "Sin brazo")

    # ---------------- Carro no bloqueante ----------------
    def _pulso_carro(self, action, dur_ms, info=""):
        if self._pulse_action is not None:
            return
        dur_ms = max(20, min(120, int(dur_ms)))
        self._pulse_action = action
        self._pulse_stop_ms = time.ticks_add(time.ticks_ms(), dur_ms)
        self._pulse_info = info
        try:
            # Usa el controlador para mantener modo/estado consistente con gemelo y watchdog.
            self.car.on_cmd({"action": action})
        except Exception as e:
            print("Pulso carro error:", e)
        self._pub(info)

    def _stop_pulso(self):
        self._pulse_action = None
        self._pulse_stop_ms = 0
        self._pulse_info = ""
        self._parar_carro()

    def _cancelar_pulso(self):
        if self._pulse_action is not None:
            self._pulse_action = None
            self._pulse_stop_ms = 0
        self._parar_carro()

    # ---------------- Helpers ----------------
    def _dist_vision_lista(self, v):
        try:
            dist = float(v.get("distancia_cm", 999))
            return dist <= VISION_RECOGER_CM
        except:
            return False

    def _dist_mm(self):
        try:
            return self.sensor.obtener_rapido()
        except:
            return None

    def _parar_carro(self):
        if self.car:
            try:
                self.car.on_cmd({"action": "detener"})
            except:
                try:
                    self.car.carro.detener()
                    self.car.modo = "detenido"
                except:
                    pass

    def _ir_a(self, nuevo, info=""):
        print("MISION {} -> {} ({})".format(self._estado, nuevo, info))
        self._estado = nuevo
        self._pub(info, force=True)

    def _pausar(self):
        self._cancelar_pulso()
        self._prev = self._estado
        self._estado = "PAUSADO"
        self._modo = "manual"
        self._pub("Pausado", force=True)

    def _reanudar(self):
        self._estado = self._prev or "BUSCAR"
        self._modo = "auto"
        self._pub("Reanudando desde " + self._estado, force=True)

    def _abortar(self):
        self._cancelar_pulso()
        self._estado = "IDLE"
        self._modo = "auto"
        self._pub("Abortado", force=True)

    def _pub(self, info="", force=False):
        now = time.ticks_ms()
        cambio = (self._estado != self._last_pub_estado) or (info != self._last_pub_info)
        vencido = time.ticks_diff(now, self._last_pub_ms) > 1500

        if (not force) and (not cambio) and (not vencido):
            return

        self._ultimo_info = info
        self._last_pub_ms = now
        self._last_pub_estado = self._estado
        self._last_pub_info = info

        self.node.publish("mission/estado", {
            "estado": self._estado,
            "nombre": self._NOMBRES.get(self._estado, self._estado),
            "modo": self._modo,
            "info": info,
        })


# =========================================================
# MAIN APP
# =========================================================
class MainApp:
    def __init__(self):
        print("INICIANDO ROBOT10 CLIENT")

        self.scheduler = Scheduler()

        self.wifi = WiFiManager(SSID, PASSWORD)
        self.socket = SocketClient(self.scheduler, BROKER_IP, BROKER_PORT)
        self.node = Node(self.socket, PREFIX)

        self.i2c = SoftI2C(
            sda=Pin(0),
            scl=Pin(1),
            freq=100000,
            timeout=100000
        )

        print("I2C scan:", self.i2c.scan())

        try:
            self.pantalla = PantallaOLED(self.i2c)
            time.sleep_ms(300)
        except Exception as e:
            print("OLED no inicializada:", e)
            self.pantalla = None

        self.distancia = SensorDistancia(self.i2c)
        self.bateria = Bateria()
        self.carro = Carro()
        self.brazo = Brazo()
        self.ir = IRRemote(PIN_IR)

    def run(self):
        print("RUN APP")

        self.wifi.connect()
        self.socket.connect()

        if self.pantalla is not None:
            self.display_task = DisplayTask(self.scheduler, self.node, self.pantalla)

        self.car_controller = CarController(self.node, self.carro)
        self.arm_controller = ArmController(self.node, self.brazo)

        BatteryTask(self.scheduler, self.node, self.bateria)
        DistanceTask(self.scheduler, self.node, self.distancia)
        CarStateTask(self.scheduler, self.car_controller)
        ArmStateTask(self.scheduler, self.arm_controller)
        IRTask(self.scheduler, self.node, self.ir)

        self.mision = MisionTask(
            self.scheduler,
            self.node,
            self.arm_controller,
            self.car_controller,
            self.distancia
        )

        self.arm_controller.mission = self.mision

        self.node.publish("debug/boot", {
            "robot": ROBOT_NAME,
            "msg": "robot10 listo"
        })

        # Publicación inicial para que el dashboard no arranque en blanco.
        self.car_controller.publish_state()
        self.arm_controller.publish_state(True, "boot")

        self.scheduler.run()


# =========================================================
# ENTRY POINT
# =========================================================
print("CREANDO APP")
app = MainApp()
app.run()




