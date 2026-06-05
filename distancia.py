import time
import vl53l0x


class SensorDistancia:
    def __init__(self, i2c):
        self.i2c = i2c
        self.sensor = None
        self.disponible = False
        self.dist_suave = None

        self.iniciar()

    def iniciar(self):
        try:
            self.sensor = vl53l0x.VL53L0X(self.i2c)
            self.disponible = True
            print("VL53L0X detectado")
        except Exception as e:
            self.sensor = None
            self.disponible = False
            print("No se pudo iniciar VL53L0X:", e)

    def leer_crudo(self):
        if not self.disponible or self.sensor is None:
            return None

        try:
            d = self.sensor.read()

            if 35 < d < 2000:
                return d

            return None

        except:
            return None

    def leer_mediana(self, n=5, pausa_ms=15):
        lecturas = []

        for _ in range(n):
            d = self.leer_crudo()

            if d is not None:
                lecturas.append(d)

            time.sleep_ms(pausa_ms)

        if not lecturas:
            return None

        lecturas.sort()
        return lecturas[len(lecturas) // 2]

    def obtener(self):
        nueva = self.leer_mediana(n=5, pausa_ms=15)

        if nueva is None:
            return self.dist_suave

        if self.dist_suave is None:
            self.dist_suave = nueva
        else:
            self.dist_suave = int(0.65 * self.dist_suave + 0.35 * nueva)

        return self.dist_suave