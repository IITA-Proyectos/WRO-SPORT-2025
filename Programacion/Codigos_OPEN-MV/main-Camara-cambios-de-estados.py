import sensor, time
from pyb import UART

# --- Configuración inicial de la cámara ---
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)

# Bloquear configuraciones automáticas
ganancia = sensor.get_gain_db()
gananciargb = sensor.get_rgb_gain_db()
exposicion = sensor.get_exposure_us()
sensor.set_auto_gain(False, gain_db=ganancia)
sensor.set_auto_whitebal(False, rgb_gain_db=gananciargb)
sensor.set_auto_exposure(False, exposure_us=(exposicion + 10000))

# UART
uart = UART(3, 19200)
uart.init(19200, bits=8, parity=None, stop=1)

# Umbrales LAB
yellow_threshold = (49, 90, -10, 20, 20, 60)   # Naranja
violet_threshold = (26, 55, 9, 21, -13, -1)    # Morado

clock = time.clock()

# Función para enviar paquete UART
def enviar_paquete(codigo, x, y):
    START_BYTE = 0xAA
    data = [codigo, x, y]
    length = len(data)
    checksum = sum(data) % 256
    paquete = bytearray([START_BYTE, length] + data + [checksum])
    return paquete

# Convierte blobs a una lista simple para comparación
def blobs_to_simple(blobs):
    return sorted([(b.cx(), b.cy(), b.w(), b.h()) for b in blobs])

# Inicialización
yellow_blobs_anterior = []
violet_blobs_anterior = []
estado_anterior = -1  # Estado anterior de detección (0, 1, 2)

# --- Bucle principal ---
while True:
    clock.tick()
    img = sensor.snapshot()

    # Blobs actuales
    yellow_blobs_actual = img.find_blobs([yellow_threshold], pixels_threshold=200,
                                         area_threshold=200, merge=True)
    violet_blobs_actual = img.find_blobs([violet_threshold], pixels_threshold=200,
                                         area_threshold=200, merge=True)

    # Dibujar blobs detectados (visualización)
    for blob in yellow_blobs_actual:
        img.draw_rectangle(blob.rect(), color=(255, 255, 0))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 0))

    for blob in violet_blobs_actual:
        img.draw_rectangle(blob.rect(), color=(255, 0, 255))
        img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 255))

    # Verificar presencia
    hay_naranja = bool(yellow_blobs_actual)
    hay_morado = bool(violet_blobs_actual)

    # Determinar nuevo estado
    if hay_naranja and not hay_morado:
        estado_actual = 1
    elif hay_morado:
        estado_actual = 2
    else:
        estado_actual = 0

    # Si el estado cambió, enviar paquete
    if estado_actual != estado_anterior:
        paquete = enviar_paquete(estado_actual, 0, 0)
        uart.write(paquete)
        print("Estado cambiado →", estado_actual, "→ Enviado:", paquete)
        estado_anterior = estado_actual

    # Guardar blobs actuales para el próximo frame
    yellow_blobs_anterior = yellow_blobs_actual
    violet_blobs_anterior = violet_blobs_actual

    time.sleep_ms(10)
