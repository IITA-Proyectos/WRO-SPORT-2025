import sensor, time
from pyb import UART

# Inicializar la cámara
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)  # 320x240
sensor.skip_frames(time=1000)
sensor.set_auto_gain(False)       # Apagar auto ganancia para mejor detección
sensor.set_auto_whitebal(False)   # Apagar balance blanco automático

# UART (por ejemplo, para enviar datos a un Arduino)
uart = UART(3, 19200)  # TX=P4, RX=P5
uart.init(19200, bits=8, parity=None, stop=1)

# Umbral para detectar color amarillo (ajustable)
# Usa Tools > Machine Vision > Threshold Editor en OpenMV IDE para afinar
yellow_threshold = (49, 90, -10, 20, 20, 60)  # (L Min, L Max, A Min, A Max, B Min, B Max)

clock = time.clock()
def enviar_paquete(codigo, x, y):
    START_BYTE = 0xAA
    data = [codigo, x, y]
    length = len(data)
    checksum = sum(data) % 256  # Suma simple

    paquete = bytearray([START_BYTE, length] + data + [checksum])
    return paquete
while True:
    clock.tick()
    img = sensor.snapshot()

    # Buscar blobs que coincidan con el umbral amarillo
    blobs = img.find_blobs([yellow_threshold], pixels_threshold=200, area_threshold=200, merge=True)

    if blobs:
        # Se detectó al menos un blob amarillo
        for blob in blobs:
            # Dibujar un rectángulo y una cruz sobre el blob
            img.draw_rectangle(blob.rect(), color=(255, 255, 0))
            img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 0))

            # Enviar por UART un byte que indique "amarillo detectado"
            paquetea=enviar_paquete(1, 0, 0)
            uart.write(paquetea)
            print(paquetea)
    else:
        # No se detecta amarillo, enviar 0
        paquetea=enviar_paquete(0, 0, 0)
        uart.write(paquetea)
        print(paquetea)

    time.sleep_ms(10)
