from machine import Pin, ADC


class Bateria:
    def __init__(self, pin_adc=26, v_min=3.4, v_max=4.2, factor=0.0001023):
        self.adc = ADC(Pin(pin_adc))
        self.v_min = v_min
        self.v_max = v_max
        self.factor = factor
        self.p_suave = None

    def leer_voltaje(self, muestras=150):
        suma = 0

        for _ in range(muestras):
            suma += self.adc.read_u16()

        raw = suma / muestras
        return raw * self.factor

    def leer_porcentaje(self, muestras=150):
        voltaje = self.leer_voltaje(muestras)

        p = int((voltaje - self.v_min) / (self.v_max - self.v_min) * 100)
        p = max(0, min(100, p))

        return p

    def obtener(self):
        p = self.leer_porcentaje()

        if self.p_suave is None:
            self.p_suave = p
        elif abs(p - self.p_suave) > 1:
            self.p_suave = p

        return self.p_suave