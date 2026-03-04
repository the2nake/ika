import signal
import serial
import sys


def interrupt_handler(signal, frame):
    print(" [interrupt] ")
    sys.exit(0)


signal.signal(signal.SIGINT, interrupt_handler)

ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
print("using port", ser.name, "@", ser.baudrate)

while True:
    data = ser.readline().decode().strip()
    if not data:
        continue
    print("[rec]", data)
    
