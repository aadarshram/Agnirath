import cantools
from pprint import pprint
#f =  open("~/Agnirath_LVS_Strategy/aarush_ros/src/can_package/can_package/DBC_Files/mc_can.dbc",)
db = cantools.database.load_file('/home/ubuntu/Agnirath_LVS_Strategy/aarush_ros/src/can_package/can_package/DBC_Files/mc_can.dbc')
db.messages
while True:

	example_message = db.get_message_by_name('Bus_Measurement')
	pprint(example_message.signals)
