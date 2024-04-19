import can
import cantools
db = cantools.database.load_file('/home/ubuntu/Agnirath_LVS_Strategy/aarush_ros/src/can_package/can_package/DBC_Files/mc_can.dbc')
can_bus = can.interface.Bus('can0', bustype='socketcan',bitrate = 500000)
#db.messages
try:
    while True:
	    message = can_bus.recv()
	    print(db.decode_message(message.arbitration_id,message.data))
except KeyboardInterrupt:
       print("CAN error")
finally:
    can_bus.shutdown()

	
