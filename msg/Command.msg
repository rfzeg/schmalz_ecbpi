# Equivalent of ProcessDataOut
uint8 suction
uint8 blow_off
uint8 control_mode
uint8 cm_autoset
uint8 epc_select
uint8 profile
uint8 demand

# Summary:
#
# suction: 1/0 - on/off vacuum pump
#
# blow_off: 1/0 - on/off ventilation
# suction and blow_off are mutually exclusive
#
# control_mode: 0/1 - 0: control by vacuum value (H1/H2/h2), 1: control by speed (in %)
#
# cm_autoset: 1/0
#
# epc_select: 0/1/2/3
#		0: 	epc_value1: power in %
#			epc_value2: vacuum
#		1:	epc_value1: cm-warning
#			epc_value2:	evac time t1
#		2:	epc_value1:	leakage
#			epc_value2:	free-flow vacuum
#		3:	epc_value1:	primary voltage
#			epc_value2:	energy consumption
#
# profile: 0/1/2/3
#
# target: 0-100, based on control mode:
#		control_mode = 0:	limit value for H2 in 10mbar
#		control_mode = 1: 	motor speed in %
