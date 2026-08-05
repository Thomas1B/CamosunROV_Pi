#!~/CamosunROV/ROV_Venv/bin/python3
'''
Testing UART communication between RPi and STM32F446.


'''


import serial
import time

ser = serial.Serial(
		port = '/dev/serial0', 
		baudrate = 115200,
		bytesize = serial.EIGHTBITS,
		parity = serial.PARITY_NONE,
		stopbits = serial.STOPBITS_ONE,
		timeout = 1)
		

LED_PC8 = 0x01
LED_PA5 = 0x02

ON = 0x01
OFF = 0x00

def set_led(led_id, state):
	packet = bytes([led_id, state])
	ser.write(packet)
	print(f"Sent: {packet.hex()}\n")
	
	
try:
	while True:
		# PC8 ON, PA5 OFF
		set_led(LED_PC8, ON)
		set_led(LED_PA5, OFF)
		time.sleep(1)
		
		# PC8 OFF, PA5 ON
		set_led(LED_PC8, OFF)
		set_led(LED_PA5, ON)
		time.sleep(1)
		
except KeyboardInterrupt:
	print("\n Script Stopping")
	set_led(LED_PC8, OFF)
	set_led(LED_PA5, OFF)
	
finally:
	ser.close()
		

