import sensor, time
from pyb import UART, LED

# Inicializar LEDs
led = LED(2)  # LED verde (cambiar por 1 o 3 si querés rojo o azul)

# Inicializar cámara
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

# UART
uart = UART(3, 19200)
uart.init(19200, bits=8, parity=None, stop=1)

# Umbral MORADO (ajústalo en Threshold Editor)
purple_threshold = (30, 80, 14, 70, -50, 10)

clock = time.clock()

def enviar_paquete(codigo, x, y):
    START_BYTE = 0xAA
    data = [codigo, x, y]
    length = len(data)
    checksum = sum(data) % 256
    paquete = bytearray([START_BYTE, length] + data + [checksum])
    return paquete

while True:
    clock.tick()
    img = sensor.snapshot()

    blobs = img.find_blobs([purple_threshold], pixels_threshold=200, area_threshold=200, merge=True)

    if blobs:
        led.on()   #  ENCENDER LED cuando ve morado

        for blob in blobs:
            img.draw_rectangle(blob.rect(), color=(128, 0, 128))
            img.draw_cross(blob.cx(), blob.cy(), color=(128, 0, 128))

            paquetea = enviar_paquete(2, blob.cx(), blob.cy())
            uart.write(paquetea)
            print("Detectado morado:", paquetea)
    else:
        led.off()  #  APAGAR LED si NO detecta morado

        paquetea = enviar_paquete(0, 0, 0)
        uart.write(paquetea)
        print(paquetea)

    time.sleep_ms(10)
