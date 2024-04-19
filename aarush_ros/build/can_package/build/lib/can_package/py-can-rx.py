import can
from time import sleep

bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

try:
    while True:
        response = bus.recv(timeout=2)
        print(response)
except can.CanError:
    print("CAN error")
finally:
    bus.shutdown()

