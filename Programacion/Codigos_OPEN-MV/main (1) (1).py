import sensor, time
from pyb import UART
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
ganancia = sensor.get_gain_db()
gananciargb = sensor.get_rgb_gain_db()
exposicion = sensor.get_exposure_us()
sensor.set_auto_gain(False,gain_db=ganancia)
sensor.set_auto_whitebal(False,rgb_gain_db=gananciargb)
sensor.set_auto_exposure(False,exposure_us=(exposicion+10000))
uart = UART(3, 19200)
uart.init(19200, bits=8, parity=None, stop=1)
yellow_threshold = (84, 99, -17, 28, 45, 61)
violet_threshold = (26, 55, 9, 21, -13, -1)
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
	yellow_blobs = img.find_blobs([yellow_threshold], pixels_threshold=200,
								  area_threshold=200, merge=True)
	violet_blobs = img.find_blobs([violet_threshold], pixels_threshold=200,
								  area_threshold=200, merge=True)
	if yellow_blobs:
		for blob in yellow_blobs:
			img.draw_rectangle(blob.rect(), color=(255, 255, 0))
			img.draw_cross(blob.cx(), blob.cy(), color=(255, 255, 0))
			paquetea = enviar_paquete(1, 0, 0)
			uart.write(paquetea)
			print("Amarillo:", paquetea)
	if violet_blobs:
		for blob in violet_blobs:
			img.draw_rectangle(blob.rect(), color=(255, 0, 255))
			img.draw_cross(blob.cx(), blob.cy(), color=(255, 0, 255))
			paquetea = enviar_paquete(2, 0, 0)
			uart.write(paquetea)
			print("Violeta:", paquetea)
	else:
		paquetea = enviar_paquete(0, 0, 0)
		uart.write(paquetea)
		print("Nada:", paquetea)
	time.sleep_ms(10)