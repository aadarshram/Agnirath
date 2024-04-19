import can
import cantools
from time import sleep

db = cantools.database.load_file('/home/ubuntu/Old_LV/aarush_ros/src/can_package/can_package/DBC_Files/combined_dbc.dbc')

sample_dict = {'Prohelion_ID':0,'Serial_Number':0,'Bus_Voltage':0,'Bus_Current':0,'Motor_Velocity':1.0,'Vehicle_Velocity':1.0}

# Create and manage the CAN interface using the 'with' statement
with can.interface.Bus('can0', bustype='socketcan', bitrate=500000) as can_bus:
    # Create a message notifier for receiving CAN messages
    while True:
        print(sample_dict)
        message = can_bus.recv()
        try:
            decoded_data =  db.decode_message(message.arbitration_id, message.data)
            #print(message.arbitration_id, decoded_data)
            keys_list = list(decoded_data.keys())
            #print(keys_list)
            message_list = list(decoded_data.values())
            #print(message_list)
            for k in keys_list:
                if k in sample_dict.keys():
                    sample_dict[k] = decoded_data[k]
            print("------------------------")
        except KeyError:
            pass
    # notifier = can.Notifier(can_bus, [can.Printer()])

   # try:
       # while True:
            # The 'recv()' function is not used here since the 'Notifier' takes care of message reception
           # pass
   # except KeyboardInterrupt:
       # print("CAN error")
