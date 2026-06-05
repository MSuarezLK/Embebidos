from machine import Pin, PWM
import time
import math

# =============================================================================
# brazo.py — montacargas seguro + XYZ limitado
#
# Convención conservada del código original que sí movía bien:
#   t1 = s1 - 90
#   t2 = 180 - s2
#   t3 = t2 - s3
#   t4 = t3 + (s4 - 90)
#
# Poses testeadas:
#   REPOSO  = base 90, brazo 0,   antebrazo 165, muñeca 140
#   RECOGER = base 90, brazo 160, antebrazo 45,  muñeca 140
#   ALZAR   = base 90, brazo 90,  antebrazo 120, muñeca 140
#
# Regla práctica:
#   - La misión usa poses por ángulos testeadas.
#   - XYZ queda activo solo para ajustes finos/manuales, con límites fuertes.
# =============================================================================

# =====================================================
# PINES
# =====================================================
PIN_BASE = 8
PIN_BRAZO = 9
PIN_ANTEBRAZO = 20
PIN_MUNECA = 21

# =====================================================
# POSES
# =====================================================
REPOSO_BASE = 90
REPOSO_BRAZO = 0
REPOSO_ANTEBRAZO = 180
REPOSO_MUNECA = 140

POS_RECOGER = (90, 180, 60, 140)
POS_ALZAR   = (90, 80, 155, 140)

# =====================================================
# LONGITUDES EN cm
# =====================================================
L1 = 3.5
L2 = 9.0
L3 = 8.0
L4 = 4.0   # eje de muñeca -> punta de tenaza

# Offset físico observado en base. Solo se usa en directa/inversa para coordenadas.
# NO se suma al servo al mover ángulos directos.
OFFSET_BASE_FISICO = 3.0

# =====================================================
# PWM SERVO
# =====================================================
FRECUENCIA_SERVO = 50
DUTY_MIN = 1638
DUTY_RANGO = 6553

# =====================================================
# LÍMITES MECÁNICOS SEGUROS
# =====================================================
BASE_MIN = 0
BASE_MAX = 180

BRAZO_MIN = 0
BRAZO_MAX = 180

ANTEBRAZO_MIN = 35
ANTEBRAZO_MAX = 180

MUNECA_MIN = 40
MUNECA_MAX = 140

# Caja XYZ para ajustes finos. Coordenadas de la cinemática local, sin offset de suelo.
XYZ_X_MIN = 1.0
XYZ_X_MAX = 24.0
XYZ_Y_MIN = -10.0
XYZ_Y_MAX = 10.0
XYZ_Z_MIN = 0.0
XYZ_Z_MAX = 26.0

# =====================================================
# MOVIMIENTO LENTO PARA NO LLEVARSE EL CARRO
# =====================================================
DURACION_DEFAULT = 2.8
PASOS_DEFAULT = 110
MAX_DELTA_SERVO_POR_PASO = 2.2
SLEEP_ENTRE_ETAPAS_MS = 120


def _clamp(v, a, b):
    return max(a, min(b, float(v)))


def _isfinite(*vals):
    for v in vals:
        try:
            if not math.isfinite(float(v)):
                return False
        except Exception:
            return False
    return True


class Servo:
    def __init__(self, pin_num, ang_inicial, amin=0, amax=180, nombre="servo"):
        self.nombre = nombre
        self.amin = float(amin)
        self.amax = float(amax)
        self.pwm = PWM(Pin(pin_num))
        self.pwm.freq(FRECUENCIA_SERVO)
        self.pos_actual = float(ang_inicial)
        self.angulo(ang_inicial)

    def angulo(self, angulo):
        angulo = _clamp(angulo, self.amin, self.amax)
        self.pos_actual = angulo
        duty = int(DUTY_MIN + (angulo / 180.0) * DUTY_RANGO)
        self.pwm.duty_u16(duty)

    def apagar(self):
        self.pwm.duty_u16(0)


class Brazo:
    def __init__(self):
        self.base = Servo(PIN_BASE, REPOSO_BASE, BASE_MIN, BASE_MAX, "base")
        self.brazo = Servo(PIN_BRAZO, REPOSO_BRAZO, BRAZO_MIN, BRAZO_MAX, "brazo")
        self.antebrazo = Servo(PIN_ANTEBRAZO, REPOSO_ANTEBRAZO, ANTEBRAZO_MIN, ANTEBRAZO_MAX, "antebrazo")
        self.muneca = Servo(PIN_MUNECA, REPOSO_MUNECA, MUNECA_MIN, MUNECA_MAX, "muneca")

        self.L1 = L1
        self.L2 = L2
        self.L3 = L3
        self.L4 = L4

    # =================================================
    # LECTURA
    # =================================================
    def leer_servos(self):
        return (
            self.base.pos_actual,
            self.brazo.pos_actual,
            self.antebrazo.pos_actual,
            self.muneca.pos_actual
        )

    def leer_angulos(self):
        return self.leer_servos()

    # =================================================
    # CINEMÁTICA
    # =================================================
    def servo_to_math(self, s1, s2, s3, s4):
        # Offset de base solo para coordenadas.
        t1 = (float(s1) - 90.0) + OFFSET_BASE_FISICO
        t2 = 180.0 - float(s2)
        t3 = t2 - float(s3)
        t4 = t3 + (float(s4) - 90.0)
        return t1, t2, t3, t4

    def directa(self, s1, s2, s3, s4):
        t1, t2, t3, t4 = self.servo_to_math(s1, s2, s3, s4)

        t1r = math.radians(t1)
        t2r = math.radians(t2)
        t3r = math.radians(t3)
        t4r = math.radians(t4)

        r = (
            self.L2 * math.cos(t2r) +
            self.L3 * math.cos(t3r) +
            self.L4 * math.cos(t4r)
        )

        z = (
            self.L1 +
            self.L2 * math.sin(t2r) +
            self.L3 * math.sin(t3r) +
            self.L4 * math.sin(t4r)
        )

        x = r * math.cos(t1r)
        y = r * math.sin(t1r)
        return x, y, z

    def math_to_servo(self, t1, t2, q3_rel, phi_abs):
        t3_abs = t2 + q3_rel

        # Compensa offset de base al volver a servo.
        s1 = (t1 - OFFSET_BASE_FISICO) + 90.0
        s2 = 180.0 - t2
        s3 = -q3_rel
        s4 = phi_abs - t3_abs + 90.0

        return s1, s2, s3, s4

    def inversa(self, x, y, z, phi_abs=0.0):
        try:
            x = float(x)
            y = float(y)
            z = float(z)
            phi_abs = float(phi_abs)

            if not _isfinite(x, y, z, phi_abs):
                return None

            if not (XYZ_X_MIN <= x <= XYZ_X_MAX and XYZ_Y_MIN <= y <= XYZ_Y_MAX and XYZ_Z_MIN <= z <= XYZ_Z_MAX):
                return None

            t1 = math.degrees(math.atan2(y, x))
            R = math.sqrt(x * x + y * y)

            phi = math.radians(phi_abs)
            rw = R - self.L4 * math.cos(phi)
            zw = z - self.L1 - self.L4 * math.sin(phi)

            d2 = rw * rw + zw * zw
            d = math.sqrt(d2)

            if d > (self.L2 + self.L3) or d < abs(self.L2 - self.L3):
                return None

            c = (d2 - self.L2 * self.L2 - self.L3 * self.L3) / (2.0 * self.L2 * self.L3)
            c = max(-1.0, min(1.0, c))

            q3_rel = -math.degrees(math.acos(c))
            q3r = math.radians(q3_rel)

            t2 = math.degrees(
                math.atan2(zw, rw) -
                math.atan2(
                    self.L3 * math.sin(q3r),
                    self.L2 + self.L3 * math.cos(q3r)
                )
            )

            return self.math_to_servo(t1, t2, q3_rel, phi_abs)

        except Exception as e:
            print("Error en inversa:", e)
            return None

    # =================================================
    # SEGURIDAD
    # =================================================
    def config_segura(self, s1, s2, s3, s4):
        if not _isfinite(s1, s2, s3, s4):
            return False

        s1 = float(s1)
        s2 = float(s2)
        s3 = float(s3)
        s4 = float(s4)

        if not (BASE_MIN <= s1 <= BASE_MAX):
            return False
        if not (BRAZO_MIN <= s2 <= BRAZO_MAX):
            return False
        if not (ANTEBRAZO_MIN <= s3 <= ANTEBRAZO_MAX):
            return False
        if not (MUNECA_MIN <= s4 <= MUNECA_MAX):
            return False

        # Evita picada mecánica.
        if s2 < 8 and s3 < 90:
            return False

        # Evita cerrar demasiado con brazo alto y muñeca metida.
        if s2 > 120 and s3 > 150 and s4 < 120:
            return False

        return True

    def _clamp_servos(self, s1, s2, s3, s4):
        return (
            _clamp(s1, BASE_MIN, BASE_MAX),
            _clamp(s2, BRAZO_MIN, BRAZO_MAX),
            _clamp(s3, ANTEBRAZO_MIN, ANTEBRAZO_MAX),
            _clamp(s4, MUNECA_MIN, MUNECA_MAX)
        )

    def imprimir_estado(self, etiqueta="ESTADO"):
        s1, s2, s3, s4 = self.leer_servos()
        x, y, z = self.directa(s1, s2, s3, s4)
        print("\n===== {} =====".format(etiqueta))
        print("Base      = {:.2f}°".format(s1))
        print("Brazo     = {:.2f}°".format(s2))
        print("Antebrazo = {:.2f}°".format(s3))
        print("Muneca    = {:.2f}°".format(s4))
        print("X = {:.2f} cm".format(x))
        print("Y = {:.2f} cm".format(y))
        print("Z = {:.2f} cm".format(z))
        print("==========================")

    # =================================================
    # MOVIMIENTO
    # =================================================
    def mover_angulos(self, s1, s2, s3, s4,
                      duracion=DURACION_DEFAULT,
                      pasos=PASOS_DEFAULT,
                      on_step=None,
                      publish_every=10):

        if not _isfinite(s1, s2, s3, s4):
            print("Comando rechazado: ángulos no finitos.")
            return False

        s1, s2, s3, s4 = self._clamp_servos(s1, s2, s3, s4)

        if not self.config_segura(s1, s2, s3, s4):
            print("Configuración rechazada por seguridad:")
            print("Base={:.2f}, Brazo={:.2f}, Antebrazo={:.2f}, Muneca={:.2f}".format(s1, s2, s3, s4))
            return False

        a1, a2, a3, a4 = self.leer_servos()
        pasos = max(1, int(pasos))
        dt = float(duracion) / pasos

        for i in range(1, pasos + 1):
            t = i / pasos
            t_suave = t * t * (3 - 2 * t)

            n1 = a1 + (s1 - a1) * t_suave
            n2 = a2 + (s2 - a2) * t_suave
            n3 = a3 + (s3 - a3) * t_suave
            n4 = a4 + (s4 - a4) * t_suave

            # Limitador anti-impulso.
            if abs(n1 - self.base.pos_actual) > MAX_DELTA_SERVO_POR_PASO:
                n1 = self.base.pos_actual + MAX_DELTA_SERVO_POR_PASO * (1 if n1 > self.base.pos_actual else -1)
            if abs(n2 - self.brazo.pos_actual) > MAX_DELTA_SERVO_POR_PASO:
                n2 = self.brazo.pos_actual + MAX_DELTA_SERVO_POR_PASO * (1 if n2 > self.brazo.pos_actual else -1)
            if abs(n3 - self.antebrazo.pos_actual) > MAX_DELTA_SERVO_POR_PASO:
                n3 = self.antebrazo.pos_actual + MAX_DELTA_SERVO_POR_PASO * (1 if n3 > self.antebrazo.pos_actual else -1)
            if abs(n4 - self.muneca.pos_actual) > MAX_DELTA_SERVO_POR_PASO:
                n4 = self.muneca.pos_actual + MAX_DELTA_SERVO_POR_PASO * (1 if n4 > self.muneca.pos_actual else -1)

            if not self.config_segura(n1, n2, n3, n4):
                print("Movimiento intermedio abortado por seguridad.")
                print("Intermedio: Base={:.2f}, Brazo={:.2f}, Antebrazo={:.2f}, Muneca={:.2f}".format(n1, n2, n3, n4))
                return False

            # Orden estable con herramienta: muñeca acompaña antes de cargar antebrazo.
            self.base.angulo(n1)
            self.muneca.angulo(n4)
            self.brazo.angulo(n2)
            self.antebrazo.angulo(n3)

            if on_step is not None and (i == pasos or (i % publish_every) == 0):
                try:
                    on_step()
                except Exception as e:
                    print("on_step error:", e)

            time.sleep(dt)

        return True

    def mover_servos_coordinado(self, s1, s2, s3, s4,
                                duracion=DURACION_DEFAULT,
                                pasos=PASOS_DEFAULT,
                                on_step=None,
                                publish_every=10):
        return self.mover_angulos(s1, s2, s3, s4, duracion, pasos, on_step, publish_every)

    def reposo(self, on_step=None, publish_every=10):
        print("\nYendo a reposo montacargas seguro...")
        s1, s2, s3, s4 = self.leer_servos()

        BRAZO_DESPEJE = 75
        ANTEBRAZO_SEGURO_BAJO = 155

        etapas = []

        # 1) Primero asegurar muñeca en posición de reposo.
        etapas.append((s1, s2, s3, REPOSO_MUNECA, 1.1, 55))

        # 2) Si el brazo está bajo, no permitir antebrazo demasiado cerrado.
        if s2 <= 80:
            if s3 > 150:
                etapas.append((s1, s2, ANTEBRAZO_SEGURO_BAJO, REPOSO_MUNECA, 1.4, 70))

            # Subir brazo a zona de despeje antes de cerrar antebrazo.
            etapas.append((s1, BRAZO_DESPEJE, min(s3, 150), REPOSO_MUNECA, 2.0, 95))

        else:
            # Si ya está alto, solo mantener una altura segura.
            etapas.append((s1, max(s2, BRAZO_DESPEJE), min(s3, 150), REPOSO_MUNECA, 1.4, 70))

        # 3) Cerrar antebrazo progresivamente, ya con brazo despejado.
        etapas.append((s1, BRAZO_DESPEJE, ANTEBRAZO_SEGURO_BAJO, REPOSO_MUNECA, 1.6, 80))
        etapas.append((s1, BRAZO_DESPEJE, REPOSO_ANTEBRAZO, REPOSO_MUNECA, 1.8, 90))

        # 4) Centrar base manteniendo despeje.
        etapas.append((REPOSO_BASE, BRAZO_DESPEJE, REPOSO_ANTEBRAZO, REPOSO_MUNECA, 1.4, 70))

        # 5) Ahora sí bajar brazo a reposo final.
        etapas.append((REPOSO_BASE, REPOSO_BRAZO, REPOSO_ANTEBRAZO, REPOSO_MUNECA, 2.7, 125))

        for e in etapas:
            ok = self.mover_angulos(
                e[0], e[1], e[2], e[3],
                duracion=e[4],
                pasos=e[5],
                on_step=on_step,
                publish_every=publish_every
            )
            if not ok:
                print("Reposo abortado en etapa:", e)
                return False

            time.sleep_ms(SLEEP_ENTRE_ETAPAS_MS)

        self.imprimir_estado("REPOSO")
        return True

    def recoger(self, on_step=None, publish_every=10):
        print("\nYendo a posición RECOGER...")
        return self.mover_angulos(*POS_RECOGER, duracion=3.0, pasos=120,
                                  on_step=on_step, publish_every=publish_every)

    def alzar(self, on_step=None, publish_every=10):
        print("\nYendo a posición ALZAR...")
        # Muñeca fija en 140 durante todo el movimiento.
        return self.mover_angulos(*POS_ALZAR, duracion=3.2, pasos=125,
                                  on_step=on_step, publish_every=publish_every)

    # =================================================
    # XYZ LIMITADO
    # =================================================
    def mover_xyz(self, x, y, z,
                  phi_abs=0.0,
                  duracion=2.6,
                  pasos=100,
                  verbose=True,
                  on_step=None,
                  publish_every=10):

        x = float(x)
        y = float(y)
        z = float(z)
        phi_abs = float(phi_abs)

        if verbose:
            print("\nObjetivo XYZ:")
            print("X={:.2f}, Y={:.2f}, Z={:.2f}, phi={:.2f}".format(x, y, z, phi_abs))

        angs = self.inversa(x, y, z, phi_abs)
        if angs is None:
            if verbose:
                print("Punto XYZ fuera de alcance o fuera de caja segura.")
            return False

        s1, s2, s3, s4 = self._clamp_servos(*angs)

        if verbose:
            print("IK calculó: Base={:.2f}, Brazo={:.2f}, Antebrazo={:.2f}, Muneca={:.2f}".format(s1, s2, s3, s4))

        if not self.config_segura(s1, s2, s3, s4):
            if verbose:
                print("XYZ rechazado por seguridad.")
            return False

        return self.mover_angulos(s1, s2, s3, s4, duracion=duracion, pasos=pasos,
                                  on_step=on_step, publish_every=publish_every)

    def mover_delta_xyz(self, dx=0, dy=0, dz=0, dphi=0,
                        duracion=1.8, pasos=70,
                        on_step=None, publish_every=10):
        # Ajuste fino máximo por comando para que XYZ no lance una pose extrema.
        dx = _clamp(dx, -1.0, 1.0)
        dy = _clamp(dy, -1.0, 1.0)
        dz = _clamp(dz, -1.0, 1.0)
        dphi = _clamp(dphi, -10.0, 10.0)

        s1, s2, s3, s4 = self.leer_servos()
        x, y, z = self.directa(s1, s2, s3, s4)
        _, _, _, phi_actual = self.servo_to_math(s1, s2, s3, s4)

        return self.mover_xyz(x + dx, y + dy, z + dz, phi_actual + dphi,
                              duracion=duracion, pasos=pasos,
                              verbose=True, on_step=on_step, publish_every=publish_every)

    # =================================================
    # APAGAR / EMERGENCIA
    # =================================================
    def apagar_servos(self):
        self.base.apagar()
        self.brazo.apagar()
        self.antebrazo.apagar()
        self.muneca.apagar()
        print("PWM de servos apagado.")

    def emergencia(self):
        print("EMERGENCIA: apagando PWM de servos.")
        self.apagar_servos()



