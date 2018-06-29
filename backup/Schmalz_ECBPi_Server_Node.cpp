#include "Schmalz_ECBPi_Server_Node.h"


int main(int argc, char* argv[]){
	ros::init(argc, argv, "schmalz_ecbpi_server_node");

	ros::NodeHandle nh_priv("~");


	getParameters();

	// Advertise the state (process data out) of the device
	ros::Publisher state_publisher = nh_priv.advertise<schmalz_ecbpi::State>("State", 10);

	// Subscribe to the command packet
	ros::Subscriber cmd_subscriber = nh_priv.subscribe("Command", 10, commandCB);


	// SERVICE SERVERS
	// TRIGGER TYPES
	ros::ServiceServer init_Srv 	= nh_priv.advertiseService("Init", init_SrvCB);
	ros::ServiceServer halt_Srv 	= nh_priv.advertiseService("Halt", halt_SrvCB);
	ros::ServiceServer recover_Srv 	= nh_priv.advertiseService("Recover", recover_SrvCB);

	ros::ServiceServer factoryReset_Srv 	= nh_priv.advertiseService("FactoryReset", factoryReset_SrvCB);
	ros::ServiceServer calibrateSensor_Srv 	= nh_priv.advertiseService("CalibrateSensor", calibrateSensor_SrvCB);
	ros::ServiceServer resetCounter_Srv 	= nh_priv.advertiseService("ResetCounter", resetCounter_SrvCB);
	ros::ServiceServer resetVoltage_Srv 	= nh_priv.advertiseService("ResetVoltage", resetVoltage_SrvCB);
	ros::ServiceServer resetVacuum_Srv 		= nh_priv.advertiseService("ResetVacuum", resetVoltage_SrvCB);

	// GET/SET TYPES
	ros::ServiceServer getInitialSettings_Srv 	= nh_priv.advertiseService("GetInitialSettings", getInitialSettings_SrvCB);
	ros::ServiceServer getProfile_Srv 			= nh_priv.advertiseService("GetProfile", getProfile_SrvCB);
	ros::ServiceServer getObservation_Srv 		= nh_priv.advertiseService("GetObservation", getObservation_SrvCB);

	ros::ServiceServer setInitialSettings_Srv 	= nh_priv.advertiseService("SetInitialSettings", setInitialSettings_SrvCB);
	ros::ServiceServer setProfile_Srv 			= nh_priv.advertiseService("SetProfile", setProfile_SrvCB);


	ros::Rate rate(20);

	while(nh_priv.ok()){
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}


int getParameters(){
	if(!ros::param::get("~device_name", port_attributes.device_name)){
		ROS_ERROR("[SERIAL PORT] PORT NAME NOT SET");
		return -1;
	}
	else{
		ROS_INFO("[Serial Port] Port Name set to %s", port_attributes.device_name.c_str());
	}

	if(!ros::param::get("~baud_rate", port_attributes.baud_rate))
		ROS_WARN("[Serial Port] BaudRate not found, use default value");
	else
		ROS_INFO("[Serial Port] BaudRate set to %d", port_attributes.baud_rate);


	if(ros::param::get("~parity", port_attributes.parity))
		ROS_INFO("[Serial Port] Parity type set to %d",port_attributes.parity);

	if(ros::param::get("~byte_size", port_attributes.byte_size))
		ROS_INFO("[Serial Port] Byte size set to %d", port_attributes.byte_size);
	
	if(ros::param::get("~stop_bit", port_attributes.stop_bit))
		ROS_INFO("[Serial Port] Stopbit type set to %d", port_attributes.stop_bit);
	
	if(ros::param::get("~time_out", port_attributes.time_out))
		ROS_INFO("[Serial Port] Read TimeOut set to %d ms", port_attributes.time_out*100);
	
	if(ros::param::get("~hardware_ctrl", port_attributes.hardware_ctrl))
		ROS_INFO("[Serial Port] Hardware Flow Control set to %d", port_attributes.hardware_ctrl);
	
	if(ros::param::get("~software_ctrl", port_attributes.software_ctrl))
		ROS_INFO("[Serial Port] Software Flow Control set to %d", port_attributes.software_ctrl);
	
	if(ros::param::get("~raw_input", port_attributes.raw_input))
		ROS_INFO("[Serial Port] Raw Input set to %d", port_attributes.raw_input);
	
	if(ros::param::get("~raw_output", port_attributes.raw_output))
		ROS_INFO("[Serial Port] Raw Output set to %d", port_attributes.raw_output);	


	return 0;
}

// PUBLISH
void publishState(){

}

// CALLBACKS
void commandCB(const schmalz_ecbpi::CommandConstPtr& ptr){

}


// SERVICES
// TRIGGER TYPE
// =====================================================================================

bool init_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	cobot_pump.init(port_attributes);
	if(cobot_pump.connect()){
		res.success = true;
		res.message = "Sucessfully initialized the device. Communication ready.";
	}
	else{
		res.success = false;
		res.message = "Failed to initialize the device.";
	}
	return true;
}


bool halt_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	ProcessDataOut pdo;
	pdo.suction = 0;
	cobot_pump.setProcessDataOut(pdo);
	if(!cobot_pump.uploadProcessDataOut()){
		res.success = false;
		res.message = "Faile to stop the pump.";
	}

	if(cobot_pump.disconnect()){
		res.success = true;
		res.message = "Sucessfully halt the device. Communication disabled.";
	}
	else{
		res.success = false;
		res.message = "Failed to halt the device.";
	}

	return true;
}


bool recover_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	if(cobot_pump.connect()){
		res.success = true;
		res.message = "Sucessfully recovered. Communication ready.";
	}
	else{
		res.success = false;
		res.message = "Failed to recover.";
	}
	return true;
}


bool factoryReset_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response & res){
	if(cobot_pump.sysCmdFactoryReset()){
		res.success = true;
		res.message = "Device is factory reset.";
	}
	else{
		res.success = false;
		res.message = "Failed to factory reset the device.";
	}
	return true;
}


bool calibrateSensor_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	if(cobot_pump.sysCmdCalibrateSensor()){
		res.success = true;
		res.message = "Device sensor is calibrated.";
	}
	else{
		res.success = false;
		res.message = "Failed to calibrate the device sensor.";
	}
	return true;
}


bool resetCounter_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	if(cobot_pump.sysCmdCounterReset()){
		res.success = true;
		res.message = "Device counters are reset.";
	}
	else{
		res.success = false;
		res.message = "Failed to reset the device counters.";
	}
	return true;
}


bool resetVoltage_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	if(cobot_pump.sysCmdVoltageReset()){
		res.success = true;
		res.message = "Device min/max voltage values are reset.";
	}
	else{
		res.success = false;
		res.message = "Failed to reset the device min/max voltage values";
	}
	return true;
}

bool resetVacuum_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	if(cobot_pump.sysCmdVacuumReset()){
		res.success = true;
		res.message = "Device min/max vacuum values are reset.";
	}
	else{
		res.success = false;
		res.message = "Failed to reset the device min/max vacuum values.";
	}
	return true;
}



bool getInitialSettings_SrvCB(schmalz_ecbpi::GetInitialSettings::Request &req, schmalz_ecbpi::GetInitialSettings::Response &res){
	if(cobot_pump.downloadInitialSettings()){
		res.success = true;
		InitialSettings ini_s;
		ini_s = cobot_pump.getInitialSettings();
		

	}
	else
		res.success = false;
	return true;
}
bool getProfile_SrvCB(schmalz_ecbpi::GetProfile::Request &req, schmalz_ecbpi::GetProfile::Response &res){
	// if(cobot_pump.downloadProfile(res.profile_no)){
	// 	res.success = true;
	// 	Profile p 	= cobot_pump.getProfile(res.profile_no);
		
	// 	res.profile.control_mode 		= p.control_mode;
	// 	res.profile.set_point_H1 		= p.set_point_H1;
	// 	res.profile.set_point_H2 		= p.set_point_H2;
	// 	res.profile.hyteresis_h2 		= p.hyteresis_h2;
	// 	res.profile.speed 				= p.speed;
	// 	res.profile.lay_down_time 		= p.lay_down_time;
	// 	res.profile.permit_evac_time	= p.permit_evac_time;
	// 	res.profile.permit_leakage_rate	= p.permit_leakage_rate;

	// }
	// else
	// 	res.success = false;
	// return true;
}

bool getObservation_SrvCB(schmalz_ecbpi::GetObservation::Request &req, schmalz_ecbpi::GetObservation::Response &res){
	return true;
}


bool setInitialSettings_SrvCB(schmalz_ecbpi::SetInitialSettings::Request &req, schmalz_ecbpi::SetInitialSettings::Response &res){
	// InitialSettings ini_s;

	// ini_s.blow_off_mode 		= req.initial_settings.blow_off_mode;
	// ini_s.soft_start			= req.initial_settings.soft_start;
	// ini_s.out2_function 		= req.initial_settings.out2_function;
	// ini_s.out3_function 		= req.initial_settings.out3_function;
	// ini_s.signal_type_input 	= req.initial_settings.signal_type_input;
	// ini_s.signal_type_output 	= req.initial_settings.signal_type_output;
	// ini_s.vacuum_unit 			= req.initial_settings.vacuum_unit;
	// ini_s.output_filter 		= req.initial_settings.output_filter;
	// ini_s.eco_mode 				= req.initial_settings.eco_mode;
	// ini_s.display_rotation 		= req.initial_settings.display_rotation;	

	// cobot_pump.setInitialSettings(ini_s);
	// if(cobot_pump.uploadInitialSettings())
	// 	res.success = true;
	// else
	// 	res.success = false;	
	// return true;
}
bool setProfile_SrvCB(schmalz_ecbpi::SetProfile::Request &req, schmalz_ecbpi::SetProfile::Response &res){
	// Profile p;

	// p.control_mode 					= req.profile.control_mode;
	// p.profile.set_point_H1 			= req.profile.set_point_H1;
	// p.profile.set_point_H2 			= req.profile.set_point_H2;
	// p.profile.hyteresis_h2 			= req.profile.hyteresis_h2;
	// p.profile.speed 				= req.profile.speed;
	// p.profile.lay_down_time 		= req.profile.lay_down_time;
	// p.profile.permit_evac_time		= req.profile.permit_evac_time;
	// p.profile.permit_leakage_rate	= req.profile.permit_leakage_rate;

	// cobot_pump.setProfile(p, req.profile_no);
	// if(cobot_pump.uploadProfile(req.profile_no))
	// 	res.success = true;
	// else
	// 	res.success = false;
	return true;
}
