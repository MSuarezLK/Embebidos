from machine import Pin, PWM
import machine
import micropython
import time
import math

micropython.alloc_emergency_exception_buf(100)

# =====================================================
# PINES MOTORES
# =====================================================
PIN_ENA = 2
PIN_IN1 = 3
PIN_IN2 = 4

PIN_ENB = 5
PIN_IN3 = 6
PIN_IN4 = 7

# =====================================================
# PINES ENCODERS
# =====================================================
PIN_ENC_IZQ_A = 16
PIN_ENC_IZQ_B = 17
PIN_ENC_DER_A = 18
PIN_ENC_DER_B = 19

# =====================================================
# CONFIGURACIÓN MECÁNICA
# =====================================================
DIAMETRO_RUEDA_M = 0.080
DISTANCIA_EJES_M = 0.135
RPM_MOTOR = 71
VEL_MAX_TEORICA = math.pi * DIAMETRO_RUEDA_M * RPM_MOTOR / 60

# Calibración de encoders para gemelo digital
TICKS_VUELTA_IZQ = 2840
TICKS_VUELTA_DER = 2765
METROS_TICK_IZQ = math.pi * DIAMETRO_RUEDA_M / TICKS_VUELTA_IZQ
METROS_TICK_DER = math.pi * DIAMETRO_RUEDA_M / TICKS_VUELTA_DER
METROS_POR_TICK = (METROS_TICK_IZQ + METROS_TICK_DER) / 2

# =====================================================
# PWM / VELOCIDADES
# =====================================================
FREQ_PWM = 1000
PWM_MIN = 18000
PWM_WEB_MAX = 50000
PWM_MAX = 65535

VEL_WEB_ADELANTE = 0.24
VEL_WEB_ATRAS = 0.20
VEL_WEB_GIRO = 0.13

# Boost corto para vencer fricción sin meter una rampa larga.
# Especialmente útil cuando una rueda no arranca en reversa.
KICK_MS = 35
KICK_PWM = 56000
PWM_MIN_REV_IZQ = 24000
PWM_MIN_REV_DER = 22000

# =====================================================
# CONTROL PI DE SINCRONÍA
# =====================================================
PERIODO_CONTROL_MS = 50
KP_FWD = 6.0
KI_FWD = 0.15
KP_REV = 9.0
KI_REV = 0.18
INTEGRAL_MAX = 5000
AJUSTE_MAX = 9000

# =====================================================
# RAMPA SOLO PARA COMANDOS PROGRAMADOS
# NO se usa en el D-pad web.
# =====================================================
RAMPA_PASOS = 5
RAMPA_DELAY_MS = 12

# =====================================================
# INVERSIÓN
# Si un motor queda al revés físicamente, cambia estos booleanos,
# pero NO cambies la lógica de adelante/atrás de los comandos.
# =====================================================
ENC_IZQ_INVERTIDO = False
ENC_DER_INVERTIDO = True
MOTOR_IZQ_INVERTIDO = False
MOTOR_DER_INVERTIDO = False


class EncoderIRQ:
    def __init__(self, pin_a, pin_b, invertido=False, nombre="encoder"):
        self.nombre = nombre
        self.a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.invertido = invertido
        self.count = 0
        self.a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=self._callback)

    def _callback(self, pin):
        inc = 1 if self.a.value() == self.b.value() else -1
        if self.invertido:
            inc = -inc
        self.count += inc

    def reset(self):
        estado = machine.disable_irq()
        self.count = 0
        machine.enable_irq(estado)

    def leer(self):
        estado = machine.disable_irq()
        c = self.count
        machine.enable_irq(estado)
        return c

    def leer_pines(self):
        return self.a.value(), self.b.value()


class Carro:
    def __init__(self):
        self.ena = PWM(Pin(PIN_ENA))
        self.enb = PWM(Pin(PIN_ENB))
        self.ena.freq(FREQ_PWM)
        self.enb.freq(FREQ_PWM)

        self.in1 = Pin(PIN_IN1, Pin.OUT)
        self.in2 = Pin(PIN_IN2, Pin.OUT)
        self.in3 = Pin(PIN_IN3, Pin.OUT)
        self.in4 = Pin(PIN_IN4, Pin.OUT)

        self.enc_izq = EncoderIRQ(PIN_ENC_IZQ_A, PIN_ENC_IZQ_B, ENC_IZQ_INVERTIDO, "izquierdo")
        self.enc_der = EncoderIRQ(PIN_ENC_DER_A, PIN_ENC_DER_B, ENC_DER_INVERTIDO, "derecho")

        self.adelante = True
        self.modo_motor = "detenido"
        self.signo_izq = 0
        self.signo_der = 0
        self.pwm_izq_actual = 0
        self.pwm_der_actual = 0
        self.vl_m_s = 0.0
        self.vr_m_s = 0.0
        self._ultima_dir_izq = None
        self._ultima_dir_der = None
        self.detener()

    # =================================================
    # CONFIGURACIÓN / ODOMETRÍA
    # =================================================
    def _configurar_direccion(self, adelante=True):
        self.adelante = bool(adelante)

    def _limitar_pwm(self, pwm, limite=PWM_WEB_MAX):
        pwm = int(pwm)
        if pwm < 0:
            return 0
        if pwm > limite:
            return limite
        return pwm

    def _velocidad_a_pwm(self, velocidad_m_s):
        velocidad_m_s = abs(float(velocidad_m_s))
        if velocidad_m_s > VEL_MAX_TEORICA:
            velocidad_m_s = VEL_MAX_TEORICA
        relacion = velocidad_m_s / VEL_MAX_TEORICA
        pwm = int(PWM_MIN + relacion * (PWM_WEB_MAX - PWM_MIN))
        return self._limitar_pwm(pwm)

    def _pwm_a_velocidad(self, pwm):
        pwm = self._limitar_pwm(pwm)
        if pwm <= 0:
            return 0.0
        if pwm <= PWM_MIN:
            return 0.0
        relacion = (pwm - PWM_MIN) / (PWM_WEB_MAX - PWM_MIN)
        return max(0.0, min(VEL_MAX_TEORICA, relacion * VEL_MAX_TEORICA))

    def pwm_web(self, modo):
        if modo == "adelante":
            return self._velocidad_a_pwm(VEL_WEB_ADELANTE)
        if modo == "atras":
            return self._velocidad_a_pwm(VEL_WEB_ATRAS)
        if modo in ("izquierda", "derecha", "girar"):
            return self._velocidad_a_pwm(VEL_WEB_GIRO)
        return 0

    def parametros_odometria(self):
        return {
            "diametro_rueda_m": DIAMETRO_RUEDA_M,
            "distancia_ejes_m": DISTANCIA_EJES_M,
            "vel_max_teorica_m_s": VEL_MAX_TEORICA,
            "metros_por_tick": METROS_POR_TICK,
            "metros_por_tick_izq": METROS_TICK_IZQ,
            "metros_por_tick_der": METROS_TICK_DER,
            "ticks_vuelta_izq": TICKS_VUELTA_IZQ,
            "ticks_vuelta_der": TICKS_VUELTA_DER,
        }

    def estado_ruedas(self):
        return {
            "signo_izq": self.signo_izq,
            "signo_der": self.signo_der,
            "pwm_izq": self.pwm_izq_actual,
            "pwm_der": self.pwm_der_actual,
            "vl_m_s": round(self.vl_m_s, 4),
            "vr_m_s": round(self.vr_m_s, 4),
        }

    def reset_encoders(self):
        self.enc_izq.reset()
        self.enc_der.reset()

    def estado_encoders(self):
        return self.enc_izq.leer(), self.enc_der.leer()

    def estado_pines_encoder(self):
        return {"izq": self.enc_izq.leer_pines(), "der": self.enc_der.leer_pines()}

    # =================================================
    # MOTORES BASE
    # =================================================
    def _aplicar_dir_izq(self, adelante=True):
        dir_fisica = bool(adelante)
        if MOTOR_IZQ_INVERTIDO:
            dir_fisica = not dir_fisica
        if dir_fisica:
            self.in1.value(1); self.in2.value(0)
        else:
            self.in1.value(0); self.in2.value(1)

    def _aplicar_dir_der(self, adelante=True):
        dir_fisica = bool(adelante)
        if MOTOR_DER_INVERTIDO:
            dir_fisica = not dir_fisica
        if dir_fisica:
            self.in3.value(1); self.in4.value(0)
        else:
            self.in3.value(0); self.in4.value(1)

    def _motor_izq(self, pwm, adelante=True):
        pwm = self._limitar_pwm(pwm)
        if pwm > 0 and not adelante and pwm < PWM_MIN_REV_IZQ:
            pwm = PWM_MIN_REV_IZQ
        self._aplicar_dir_izq(adelante)
        self.ena.duty_u16(pwm)
        self.pwm_izq_actual = pwm
        self.signo_izq = 0 if pwm == 0 else (1 if adelante else -1)

    def _motor_der(self, pwm, adelante=True):
        pwm = self._limitar_pwm(pwm)
        if pwm > 0 and not adelante and pwm < PWM_MIN_REV_DER:
            pwm = PWM_MIN_REV_DER
        self._aplicar_dir_der(adelante)
        self.enb.duty_u16(pwm)
        self.pwm_der_actual = pwm
        self.signo_der = 0 if pwm == 0 else (1 if adelante else -1)

    def detener(self):
        self.in1.value(0); self.in2.value(0)
        self.in3.value(0); self.in4.value(0)
        self.ena.duty_u16(0)
        self.enb.duty_u16(0)
        self.modo_motor = "detenido"
        self.signo_izq = 0
        self.signo_der = 0
        self.pwm_izq_actual = 0
        self.pwm_der_actual = 0
        self.vl_m_s = 0.0
        self.vr_m_s = 0.0
        self._ultima_dir_izq = None
        self._ultima_dir_der = None

    def conducir_continuo(self, pwm_izq, pwm_der, dir_izq=True, dir_der=True, kick=True):
        """Control inmediato para web/autónomo. No usa rampas largas ni sleeps grandes."""
        pwm_izq = self._limitar_pwm(pwm_izq)
        pwm_der = self._limitar_pwm(pwm_der)

        cambio_dir = (self._ultima_dir_izq != bool(dir_izq)) or (self._ultima_dir_der != bool(dir_der))

        # Kick corto solo al cambiar de dirección. Ayuda cuando una rueda no arranca en reversa.
        if kick and cambio_dir and (pwm_izq > 0 or pwm_der > 0):
            if pwm_izq > 0:
                self._motor_izq(min(KICK_PWM, PWM_MAX), dir_izq)
            if pwm_der > 0:
                self._motor_der(min(KICK_PWM, PWM_MAX), dir_der)
            time.sleep_ms(KICK_MS)

        self._motor_izq(pwm_izq, dir_izq)
        self._motor_der(pwm_der, dir_der)
        self._ultima_dir_izq = bool(dir_izq)
        self._ultima_dir_der = bool(dir_der)

        self.vl_m_s = self._pwm_a_velocidad(pwm_izq) * (1 if dir_izq else -1)
        self.vr_m_s = self._pwm_a_velocidad(pwm_der) * (1 if dir_der else -1)

    # =================================================
    # RAMPA CORTA PARA MOVIMIENTOS PROGRAMADOS
    # =================================================
    def _aplicar_motores_suave(self, pwm_izq, pwm_der, dir_izq=True, dir_der=True):
        pwm_izq = self._limitar_pwm(pwm_izq)
        pwm_der = self._limitar_pwm(pwm_der)
        for i in range(1, RAMPA_PASOS + 1):
            k = i / RAMPA_PASOS
            self.conducir_continuo(int(pwm_izq * k), int(pwm_der * k), dir_izq, dir_der, kick=(i == 1))
            time.sleep_ms(RAMPA_DELAY_MS)

    # =================================================
    # COMANDOS WEB
    # =================================================
    def web_adelante(self):
        self.modo_motor = "adelante"
        pwm = self._velocidad_a_pwm(VEL_WEB_ADELANTE)
        self.conducir_continuo(pwm, pwm, True, True)

    def web_atras(self):
        self.modo_motor = "atras"
        pwm = self._velocidad_a_pwm(VEL_WEB_ATRAS)
        self.conducir_continuo(pwm, pwm, False, False)

    def web_izquierda(self):
        self.modo_motor = "izquierda"
        pwm = self._velocidad_a_pwm(VEL_WEB_GIRO)
        # Giro en el puesto: izquierda atrás, derecha adelante.
        self.conducir_continuo(pwm, pwm, False, True)

    def web_derecha(self):
        self.modo_motor = "derecha"
        pwm = self._velocidad_a_pwm(VEL_WEB_GIRO)
        # Giro en el puesto: izquierda adelante, derecha atrás.
        self.conducir_continuo(pwm, pwm, True, False)

    # =================================================
    # MOVIMIENTOS PROGRAMADOS
    # =================================================
    def recta(self, velocidad=0.25, tiempo=2.0, debug=False):
        pwm_base = self._velocidad_a_pwm(velocidad)
        self.reset_encoders()
        last_izq, last_der = self.estado_encoders()
        integral = 0
        t0 = time.ticks_ms()

        while time.ticks_diff(time.ticks_ms(), t0) < int(tiempo * 1000):
            c_izq, c_der = self.estado_encoders()
            d_izq = abs(c_izq - last_izq)
            d_der = abs(c_der - last_der)
            last_izq, last_der = c_izq, c_der
            error = d_der - d_izq
            kp = KP_FWD if self.adelante else KP_REV
            ki = KI_FWD if self.adelante else KI_REV
            integral += error
            integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, integral))
            ajuste = int(kp * error + ki * integral)
            ajuste = max(-AJUSTE_MAX, min(AJUSTE_MAX, ajuste))
            pwm_izq = self._limitar_pwm(pwm_base + ajuste)
            pwm_der = self._limitar_pwm(pwm_base - ajuste)
            self.conducir_continuo(pwm_izq, pwm_der, self.adelante, self.adelante, kick=False)
            if debug:
                print("dI:", d_izq, "dD:", d_der, "err:", error, "aj:", ajuste, "pwmI:", pwm_izq, "pwmD:", pwm_der)
            time.sleep_ms(PERIODO_CONTROL_MS)
        self.detener()

    def girar(self, direccion="derecha", velocidad=0.10, tiempo=1.0):
        pwm = self._velocidad_a_pwm(velocidad)
        direccion = str(direccion).lower()
        if direccion == "derecha":
            self._aplicar_motores_suave(pwm, pwm, True, False)
        else:
            self._aplicar_motores_suave(pwm, pwm, False, True)
        time.sleep(tiempo)
        self.detener()

    def circulo(self, radio_giro=0.20, velocidad_lineal=0.18, direccion="derecha", tiempo=2.0):
        radio_giro = abs(float(radio_giro))
        velocidad_lineal = abs(float(velocidad_lineal))
        if radio_giro < 0.03:
            self.girar(direccion=direccion, velocidad=velocidad_lineal, tiempo=tiempo)
            return
        L = DISTANCIA_EJES_M
        r_in = max(0.01, radio_giro - L / 2)
        r_out = radio_giro + L / 2
        v_in = velocidad_lineal * (r_in / radio_giro)
        v_out = velocidad_lineal * (r_out / radio_giro)
        pwm_in = self._velocidad_a_pwm(v_in)
        pwm_out = self._velocidad_a_pwm(v_out)
        if direccion == "derecha":
            self._aplicar_motores_suave(pwm_out, pwm_in, True, True)
        else:
            self._aplicar_motores_suave(pwm_in, pwm_out, True, True)
        time.sleep(tiempo)
        self.detener()

    # =================================================
    # PRUEBAS DE DIAGNÓSTICO
    # =================================================
    def prueba_motores_basica(self, pwm=32000, tiempo=1.0):
        """Levanta el carro. Prueba cada sentido de cada rueda."""
        pruebas = [
            ("IZQ adelante", pwm, 0, True, True),
            ("IZQ atras",    pwm, 0, False, True),
            ("DER adelante", 0, pwm, True, True),
            ("DER atras",    0, pwm, True, False),
            ("AMBAS adelante", pwm, pwm, True, True),
            ("AMBAS atras",    pwm, pwm, False, False),
            ("GIRO izquierda", pwm, pwm, False, True),
            ("GIRO derecha",   pwm, pwm, True, False),
        ]
        for nombre, pi, pd, di, dd in pruebas:
            print("\nTEST:", nombre)
            self.reset_encoders()
            self.conducir_continuo(pi, pd, di, dd)
            time.sleep(tiempo)
            self.detener()
            time.sleep_ms(200)
            print("Encoders:", self.estado_encoders(), "Pines:", self.estado_pines_encoder())

    def prueba_encoders(self, pwm=25000, tiempo=4):
        print("Prueba de encoders con motores. Levanta el carrito.")
        self.reset_encoders()
        self._aplicar_motores_suave(pwm, pwm, True, True)
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < int(tiempo * 1000):
            print("Encoders:", self.estado_encoders(), "| Pines:", self.estado_pines_encoder())
            time.sleep_ms(300)
        self.detener()
        print("Final:", self.estado_encoders())

    def prueba_manual_encoders(self, tiempo=8):
        print("Mueve las ruedas con la mano.")
        self.reset_encoders()
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < int(tiempo * 1000):
            print("Encoders:", self.estado_encoders(), "| Pines:", self.estado_pines_encoder())
            time.sleep_ms(300)
        print("Final:", self.estado_encoders())

