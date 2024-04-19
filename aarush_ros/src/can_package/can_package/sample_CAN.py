import can
from time import sleep

bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

msg_l = can.Message(arbitration_id=0x100, data=[1], is_extended_id=False)
msg_h = can.Message(arbitration_id=0x100, data=[16], is_extended_id=False) 

try:
    while True:
        bus.send(msg_l)
        sleep(0.5)
        response = bus.recv(timeout=2)
        print(response)
        bus.send(msg_h)
        sleep(0.5)
        response = bus.recv(timeout=2)
        print(response)
except can.CanError:
    print("CAN error")
finally:
    bus.shutdown()
