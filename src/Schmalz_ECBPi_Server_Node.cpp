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

  // USER MADE TYPE
  ros::ServiceServer gripper_on_off_Srv 	= nh_priv.advertiseService("GripperOnOff", gripperOnOff_SrvCB);

	ros::Rate rate(20);

	while(nh_priv.ok()){
		ros::spinOnce();
		if(isInit){
			cobot_pump.downloadState();
			state_publisher.publish(cobot_pump.getState());
		}

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
// void publishState(){
// 	cobot_pump.downloadState();

// }

// CALLBACKS
void commandCB(const schmalz_ecbpi::CommandConstPtr& ptr){
	cobot_pump.setCommand(*ptr);
	cobot_pump.uploadCommand();
}


// SERVICES
// TRIGGER TYPE
// =====================================================================================

bool init_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	cobot_pump.init(port_attributes);
	if(cobot_pump.connect()==0){
		ROS_INFO("Communication established. Downloading Parameters...");
		cobot_pump.downloadParameters();
		ROS_INFO("Parameters downloaded. Ready.");
		isInit = true;
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
	schmalz_ecbpi::Command pdo;
	pdo.suction = 0;
	cobot_pump.setCommand(pdo);
	cobot_pump.uploadCommand();

	if(cobot_pump.disconnect()==0){
		res.success = true;
		res.message = "Sucessfully halt the device. Communication disabled.";
		isInit = false;
	}
	else{
		res.success = false;
		res.message = "Failed to halt the device.";
	}

	return true;
}


bool recover_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
	if(cobot_pump.connect()==0){
		res.success = true;
		res.message = "Sucessfully recovered. Communication ready.";
		isInit = true;
	}
	else{
		res.success = false;
		res.message = "Failed to recover.";
	}
	return true;
}


bool factoryReset_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response & res){
	if(cobot_pump.sysCmdFactoryReset()==0){
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
	if(cobot_pump.sysCmdCalibrateSensor()==0){
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
	if(cobot_pump.sysCmdCounterReset()==0){
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
	if(cobot_pump.sysCmdVoltageReset()==0){
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
	if(cobot_pump.sysCmdVacuumReset()==0){
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
	cobot_pump.downloadInitialSettings();
	res.initial_settings = cobot_pump.getInitialSettings();
	return true;
}
bool getProfile_SrvCB(schmalz_ecbpi::GetProfile::Request &req, schmalz_ecbpi::GetProfile::Response &res){
	cobot_pump.downloadProfile(req.profile_no);
	res.profile = cobot_pump.getProfile(req.profile_no);
	return true;
}

bool getObservation_SrvCB(schmalz_ecbpi::GetObservation::Request &req, schmalz_ecbpi::GetObservation::Response &res){
	cobot_pump.downloadObservation();
	res.observation = cobot_pump.getObservation();
	return true;
}


bool setInitialSettings_SrvCB(schmalz_ecbpi::SetInitialSettings::Request &req, schmalz_ecbpi::SetInitialSettings::Response &res){
	cobot_pump.setInitialSettings(req.initial_settings);
	cobot_pump.uploadInitialSettings();
	return true;
}
bool setProfile_SrvCB(schmalz_ecbpi::SetProfile::Request &req, schmalz_ecbpi::SetProfile::Response &res){
	cobot_pump.setProfile(req.profile, req.profile_no);
	cobot_pump.uploadProfile(req.profile_no);
	return true;
}



bool gripperOnOff_SrvCB(schmalz_ecbpi::GripperOnOff::Request &req, schmalz_ecbpi::GripperOnOff::Response &res)
{
  if (cobot_pump.connect() == 0)
  {
    schmalz_ecbpi::Command cmd;
    cmd.suction = req.start;
    cmd.blow_off = req.stop;

    cobot_pump.setCommand(cmd);
    cobot_pump.uploadCommand();

    res.message = "Sucessfully gripper activated.";
    res.success = true;
  }

  else
  {
    res.message = "Failed to activate gripper, need to initialize the device.";
    res.success = false;
    return false;
  }
  return true;
}
