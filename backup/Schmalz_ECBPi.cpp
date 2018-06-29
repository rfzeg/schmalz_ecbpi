#include "Schmalz_ECBPi.h"

IODevice::IODevice(){
}

IODevice::~IODevice(){
}


// INIT - CONNECT - DISCONNECT
// ===========================================================================================
void IODevice::init(const PortAttributes& pa){
	this->ser_port_.setPortAttributes(pa);
}

bool IODevice::connect(){
	if(this->ser_port_.connect()==0){
		std::cout<<"Serial Communication to Adaptor enabled"<<std::endl;
		this->ser_port_.writeVector(Disable_IOLink_Comm, Disable_IOLink_Comm.size());
		if(this->ser_port_.writeVector(Enable_IOLink_Comm, Enable_IOLink_Comm.size())==0){
			std::cout<<"IO-Link Communication to Device enabled"<<std::endl;
			ser_port_.resetIO();

			//Need to wait 1s
			usleep(1000000);
			return true;
		}		
	}
	return false;
}

bool IODevice::disconnect(){
    if(this->ser_port_.writeVector(Disable_IOLink_Comm, Enable_IOLink_Comm.size())==0){
    	this->ser_port_.writeVector(Disable_IOLink_Comm, Disable_IOLink_Comm.size());
       	std::cout<<"IO-Link Communication to Device disabled"<<std::endl;
		ser_port_.resetIO();
       	if(this->ser_port_.disconnect()==0){
       		std::cout<<"Serial Communication to Adaptor disabled"<<std::endl;
       		return true;
       	}
    }
    return false;
}


// DEVICE ID
// ========================================================================================================
bool IODevice::downloadDeviceID(){
	return 
	readStringParameter(ADDR_ID_DM_VENDOR_NAME, 	NO_SUB_INDEX, LEN_ID_DM_VENDOR_NAME, dev_ID_.vendor_name) 			&&
	readStringParameter(ADDR_ID_DM_VENDOR_TEXT,	NO_SUB_INDEX, LEN_ID_DM_VENDOR_TEXT, dev_ID_.vendor_text) 				&&
	readStringParameter(ADDR_ID_DM_PRODUCT_NAME, 	NO_SUB_INDEX, LEN_ID_DM_PRODUCT_NAME, dev_ID_.product_name) 		&&
	readStringParameter(ADDR_ID_DM_PRODUCT_ID, 	NO_SUB_INDEX, LEN_ID_DM_PRODUCT_ID, dev_ID_.product_id) 				&&
	readStringParameter(ADDR_ID_DM_PRODUCT_TEXT, 	NO_SUB_INDEX, LEN_ID_DM_PRODUCT_TEXT, dev_ID_.product_text) 		&&
	readStringParameter(ADDR_ID_DM_SERIAL_NUMBER, NO_SUB_INDEX, LEN_ID_DM_SERIAL_NUMBER, dev_ID_.serial_number) 		&&
	readStringParameter(ADDR_ID_DM_HARDWARE_REV, 	NO_SUB_INDEX, LEN_ID_DM_HARDWARE_REV, dev_ID_.hardware_revision) 	&&
	readStringParameter(ADDR_ID_DM_FIRMWARE_REV, 	NO_SUB_INDEX, LEN_ID_DM_FIRMWARE_REV, dev_ID_.firmware_revision) 	&&
	readStringParameter(ADDR_ID_DM_UNIQUE_DEV_ID, NO_SUB_INDEX, LEN_ID_DM_UNIQUE_DEV_ID, dev_ID_.unique_dev_id) 		&&
	readStringParameter(ADDR_ID_DM_FEATURE_LIST, 	NO_SUB_INDEX, LEN_ID_DM_FEATURE_LIST, dev_ID_.feature_list) 		&&
	readStringParameter(ADDR_ID_DM_ARTICLE_NUM, 	NO_SUB_INDEX, LEN_ID_DM_ARTICLE_NUM, dev_ID_.article_number) 		&&
	readStringParameter(ADDR_ID_DM_ARTICLE_REV, 	NO_SUB_INDEX, LEN_ID_DM_ARTICLE_REV, dev_ID_.article_revision) 		&&
	readStringParameter(ADDR_ID_DM_PRODUCTION_CODE, NO_SUB_INDEX, LEN_ID_DM_PRODUCTION_CODE, dev_ID_.product_code) 		&&	
	readStringParameter(ADDR_ID_DM_PRODUCTION_TEXT, NO_SUB_INDEX, LEN_ID_DM_PRODUCTION_TEXT, dev_ID_.product_text_detailed);

}



// INITIAL SETTINGS
// =========================================================================================================
bool IODevice::downloadInitialSettings(){
	return 
	readParameter(ADDR_PARAM_DEVSET_INIT_BLOWOFF, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_BLOWOFF, 		 dev_Param_.initial_settings.blow_off_mode)			&&
	readParameter(ADDR_PARAM_DEVSET_INIT_SOFTSTART, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_SOFTSTART, 	 dev_Param_.initial_settings.soft_start)			&&
	readParameter(ADDR_PARAM_DEVSET_INIT_OUT2_FCN, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_OUT2_FCN, 	 dev_Param_.initial_settings.out2_function)			&&
	readParameter(ADDR_PARAM_DEVSET_INIT_OUT3_FCN, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_OUT3_FCN, 	 dev_Param_.initial_settings.out3_function)			&&
	readParameter(ADDR_PARAM_DEVSET_INIT_SIG_TYPE, ADDR_PARAM_DEVSET_INIT_SIG_TYPE_SUB_IN, LEN_PARAM_DEVSET_INIT_SIG_TYPE_SUB_IN, 	 dev_Param_.initial_settings.signal_type_input)	&&
	readParameter(ADDR_PARAM_DEVSET_INIT_SIG_TYPE, ADDR_PARAM_DEVSET_INIT_SIG_TYPE_SUB_OUT, LEN_PARAM_DEVSET_INIT_SIG_TYPE_SUB_OUT,  dev_Param_.initial_settings.signal_type_output)	&&
	readParameter(ADDR_PARAM_DEVSET_INIT_VACUUM_UNIT, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_VACUUM_UNIT, 	 dev_Param_.initial_settings.vacuum_unit)		&&
	readParameter(ADDR_PARAM_DEVSET_INIT_OUT_FILTER, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_OUT_FILTER, 	 dev_Param_.initial_settings.output_filter)		&&
	readParameter(ADDR_PARAM_DEVSET_INIT_ECO_MODE, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_ECO_MODE, 		 dev_Param_.initial_settings.eco_mode)				&&
	readParameter(ADDR_PARAM_DEVSET_INIT_DISPLAY_ROTATION, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_DISPLAY_ROTATION, dev_Param_.initial_settings.display_rotation);
	
}


bool IODevice::uploadInitialSettings(){
	//to device
	return 
	writeParameter(ADDR_PARAM_DEVSET_INIT_BLOWOFF, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_BLOWOFF, dev_Param_.initial_settings.blow_off_mode)			&&
	writeParameter(ADDR_PARAM_DEVSET_INIT_SOFTSTART, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_SOFTSTART, dev_Param_.initial_settings.soft_start)			&&
	writeParameter(ADDR_PARAM_DEVSET_INIT_OUT2_FCN, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_OUT2_FCN, dev_Param_.initial_settings.out2_function)		&&
	writeParameter(ADDR_PARAM_DEVSET_INIT_OUT3_FCN, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_OUT3_FCN, dev_Param_.initial_settings.out3_function)		&&
	writeParameter(ADDR_PARAM_DEVSET_INIT_SIG_TYPE, ADDR_PARAM_DEVSET_INIT_SIG_TYPE_SUB_IN, LEN_PARAM_DEVSET_INIT_SIG_TYPE_SUB_IN, dev_Param_.initial_settings.signal_type_input)		&&
	writeParameter(ADDR_PARAM_DEVSET_INIT_SIG_TYPE, ADDR_PARAM_DEVSET_INIT_SIG_TYPE_SUB_OUT, LEN_PARAM_DEVSET_INIT_SIG_TYPE_SUB_OUT, dev_Param_.initial_settings.signal_type_output)	&&
	writeParameter(ADDR_PARAM_DEVSET_INIT_VACUUM_UNIT, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_VACUUM_UNIT, dev_Param_.initial_settings.vacuum_unit)	&&
	writeParameter(ADDR_PARAM_DEVSET_INIT_OUT_FILTER, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_OUT_FILTER, dev_Param_.initial_settings.output_filter)	&&
	writeParameter(ADDR_PARAM_DEVSET_INIT_ECO_MODE, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_ECO_MODE, dev_Param_.initial_settings.eco_mode)				&&
	writeParameter(ADDR_PARAM_DEVSET_INIT_DISPLAY_ROTATION, NO_SUB_INDEX, LEN_PARAM_DEVSET_INIT_DISPLAY_ROTATION, dev_Param_.initial_settings.display_rotation);

	
}

// PROFILES
// ==================================================================================================
bool IODevice::downloadProfile(const uint8_t& i){
	switch(i){
		case 0: return downloadProfile_0();
		case 1: return downloadProfile_1();
		case 2: return downloadProfile_2();
		case 3: return downloadProfile_3();
		case 4: {
					return 
					downloadProfile_0() &&
					downloadProfile_1() &&
					downloadProfile_2() &&
					downloadProfile_3();
				}
		default: return false;
	}
}

bool IODevice::uploadProfile(const uint8_t& i){
	switch(i){
		case 0: return uploadProfile_0();
		case 1: return uploadProfile_1();
		case 2: return uploadProfile_2();
		case 3: return uploadProfile_3();
		case 4: {
					return 
					uploadProfile_0() &&
					uploadProfile_1() &&
					uploadProfile_2() &&
					uploadProfile_3();
				}
		default: return false;
	}
}

bool IODevice::downloadProfile_0(){
	return 
	readParameter(ADDR_PARAM_PROCSET_P0_CONTROL_MODE, 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_CONTROL_MODE,		dev_Param_.profiles[0].control_mode) 		&&
	readParameter(ADDR_PARAM_PROCSET_P0_H1, 			 	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1, 				dev_Param_.profiles[0].set_point_H1)		&&
	readParameter(ADDR_PARAM_PROCSET_P0_H2, 			 	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H2, 				dev_Param_.profiles[0].set_point_H2)		&&
	readParameter(ADDR_PARAM_PROCSET_P0_HYTERESIS, 	 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_HYTERESIS, 		dev_Param_.profiles[0].hyteresis_h2)		&&
	readParameter(ADDR_PARAM_PROCSET_P0_SPEED, 		 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_SPEED, 		 	dev_Param_.profiles[0].speed)				&&
	readParameter(ADDR_PARAM_PROCSET_P0_LAYDOWN_TIME, 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_LAYDOWN_TIME, 	dev_Param_.profiles[0].lay_down_time)		&&
	readParameter(ADDR_PARAM_PROCSET_P0_PERMIT_EVAC_TIME,	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_EVAC_TIME, dev_Param_.profiles[0].permit_evac_time)	&&
	readParameter(ADDR_PARAM_PROCSET_P0_PERMIT_LEAKAGE,		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_LEAKAGE, 	dev_Param_.profiles[0].permit_leakage_rate);
	// dev_Param_.Profiles[0].Name
}
bool IODevice::downloadProfile_1(){
	return 
	readParameter(ADDR_PARAM_PROCSET_P1_CONTROL_MODE, 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_CONTROL_MODE,		dev_Param_.profiles[1].control_mode) 		&&
	readParameter(ADDR_PARAM_PROCSET_P1_H1, 			 	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1, 				dev_Param_.profiles[1].set_point_H1)		&&
	readParameter(ADDR_PARAM_PROCSET_P1_H2, 			 	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H2, 				dev_Param_.profiles[1].set_point_H2)		&&
	readParameter(ADDR_PARAM_PROCSET_P1_HYTERESIS, 	 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_HYTERESIS, 		dev_Param_.profiles[1].hyteresis_h2)		&&
	readParameter(ADDR_PARAM_PROCSET_P1_SPEED, 		 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_SPEED, 		 	dev_Param_.profiles[1].speed)				&&
	readParameter(ADDR_PARAM_PROCSET_P1_LAYDOWN_TIME, 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_LAYDOWN_TIME, 	dev_Param_.profiles[1].lay_down_time)		&&
	readParameter(ADDR_PARAM_PROCSET_P1_PERMIT_EVAC_TIME,	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_EVAC_TIME, dev_Param_.profiles[1].permit_evac_time)	&&
	readParameter(ADDR_PARAM_PROCSET_P1_PERMIT_LEAKAGE,		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_LEAKAGE, 	dev_Param_.profiles[1].permit_leakage_rate);

}
bool IODevice::downloadProfile_2(){
	return 
	readParameter(ADDR_PARAM_PROCSET_P2_CONTROL_MODE, 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_CONTROL_MODE,		dev_Param_.profiles[2].control_mode) 		&&
	readParameter(ADDR_PARAM_PROCSET_P2_H1, 			 	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1, 				dev_Param_.profiles[2].set_point_H1)		&&
	readParameter(ADDR_PARAM_PROCSET_P2_H2, 			 	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H2, 				dev_Param_.profiles[2].set_point_H2)		&&
	readParameter(ADDR_PARAM_PROCSET_P2_HYTERESIS, 	 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_HYTERESIS, 		dev_Param_.profiles[2].hyteresis_h2)		&&
	readParameter(ADDR_PARAM_PROCSET_P2_SPEED, 		 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_SPEED, 		 	dev_Param_.profiles[2].speed)				&&
	readParameter(ADDR_PARAM_PROCSET_P2_LAYDOWN_TIME, 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_LAYDOWN_TIME, 	dev_Param_.profiles[2].lay_down_time)		&&
	readParameter(ADDR_PARAM_PROCSET_P2_PERMIT_EVAC_TIME,	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_EVAC_TIME, dev_Param_.profiles[2].permit_evac_time)	&&
	readParameter(ADDR_PARAM_PROCSET_P2_PERMIT_LEAKAGE,		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_LEAKAGE, 	dev_Param_.profiles[2].permit_leakage_rate);

}
bool IODevice::downloadProfile_3(){
	return 
	readParameter(ADDR_PARAM_PROCSET_P3_CONTROL_MODE, 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_CONTROL_MODE,		dev_Param_.profiles[3].control_mode) 		&&
	readParameter(ADDR_PARAM_PROCSET_P3_H1, 			 	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1, 				dev_Param_.profiles[3].set_point_H1)		&&
	readParameter(ADDR_PARAM_PROCSET_P3_H2, 			 	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H2, 				dev_Param_.profiles[3].set_point_H2)		&&
	readParameter(ADDR_PARAM_PROCSET_P3_HYTERESIS, 	 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_HYTERESIS, 		dev_Param_.profiles[3].hyteresis_h2)		&&
	readParameter(ADDR_PARAM_PROCSET_P3_SPEED, 		 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_SPEED, 		 	dev_Param_.profiles[3].speed)				&&
	readParameter(ADDR_PARAM_PROCSET_P3_LAYDOWN_TIME, 		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_LAYDOWN_TIME, 	dev_Param_.profiles[3].lay_down_time)		&&
	readParameter(ADDR_PARAM_PROCSET_P3_PERMIT_EVAC_TIME,	NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_EVAC_TIME, dev_Param_.profiles[3].permit_evac_time)	&&
	readParameter(ADDR_PARAM_PROCSET_P3_PERMIT_LEAKAGE,		NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_LEAKAGE, 	dev_Param_.profiles[3].permit_leakage_rate);

}

bool IODevice::uploadProfile_0(){
	return 
	writeParameter(ADDR_PARAM_PROCSET_P0_CONTROL_MODE, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_CONTROL_MODE, 			dev_Param_.profiles[0].control_mode)		&&
	writeParameter(ADDR_PARAM_PROCSET_P0_H1, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1, 								dev_Param_.profiles[0].set_point_H1)		&&
	writeParameter(ADDR_PARAM_PROCSET_P0_H2, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H2, 								dev_Param_.profiles[0].set_point_H2)		&&
	writeParameter(ADDR_PARAM_PROCSET_P0_HYTERESIS, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_HYTERESIS, 				dev_Param_.profiles[0].hyteresis_h2)		&&
	writeParameter(ADDR_PARAM_PROCSET_P0_SPEED, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_SPEED, 						dev_Param_.profiles[0].speed)				&&
	writeParameter(ADDR_PARAM_PROCSET_P0_LAYDOWN_TIME, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_LAYDOWN_TIME, 			dev_Param_.profiles[0].lay_down_time)		&&
	writeParameter(ADDR_PARAM_PROCSET_P0_PERMIT_EVAC_TIME, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_EVAC_TIME, 	dev_Param_.profiles[0].permit_evac_time)	&&
	writeParameter(ADDR_PARAM_PROCSET_P0_PERMIT_LEAKAGE, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_LEAKAGE, 	dev_Param_.profiles[0].permit_leakage_rate);

}

bool IODevice::uploadProfile_1(){
	return
	writeParameter(ADDR_PARAM_PROCSET_P1_CONTROL_MODE, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_CONTROL_MODE, 			dev_Param_.profiles[1].control_mode)		&&
	writeParameter(ADDR_PARAM_PROCSET_P1_H1, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1, 								dev_Param_.profiles[1].set_point_H1)		&&
	writeParameter(ADDR_PARAM_PROCSET_P1_H2, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H2, 								dev_Param_.profiles[1].set_point_H2)		&&
	writeParameter(ADDR_PARAM_PROCSET_P1_HYTERESIS, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_HYTERESIS, 				dev_Param_.profiles[1].hyteresis_h2)		&&
	writeParameter(ADDR_PARAM_PROCSET_P1_SPEED, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_SPEED, 						dev_Param_.profiles[1].speed)				&&
	writeParameter(ADDR_PARAM_PROCSET_P1_LAYDOWN_TIME, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_LAYDOWN_TIME, 			dev_Param_.profiles[1].lay_down_time)		&&
	writeParameter(ADDR_PARAM_PROCSET_P1_PERMIT_EVAC_TIME, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_EVAC_TIME, 	dev_Param_.profiles[1].permit_evac_time)	&&
	writeParameter(ADDR_PARAM_PROCSET_P1_PERMIT_LEAKAGE, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_LEAKAGE, 	dev_Param_.profiles[1].permit_leakage_rate);
}


bool IODevice::uploadProfile_2(){
	return
	writeParameter(ADDR_PARAM_PROCSET_P2_CONTROL_MODE, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_CONTROL_MODE, 			dev_Param_.profiles[2].control_mode)		&&
	writeParameter(ADDR_PARAM_PROCSET_P2_H1, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1, 								dev_Param_.profiles[2].set_point_H1)		&&
	writeParameter(ADDR_PARAM_PROCSET_P2_H2, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H2, 								dev_Param_.profiles[2].set_point_H2)		&&
	writeParameter(ADDR_PARAM_PROCSET_P2_HYTERESIS, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_HYTERESIS, 				dev_Param_.profiles[2].hyteresis_h2)		&&
	writeParameter(ADDR_PARAM_PROCSET_P2_SPEED, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_SPEED, 						dev_Param_.profiles[2].speed)				&&
	writeParameter(ADDR_PARAM_PROCSET_P2_LAYDOWN_TIME, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_LAYDOWN_TIME, 			dev_Param_.profiles[2].lay_down_time)		&&
	writeParameter(ADDR_PARAM_PROCSET_P2_PERMIT_EVAC_TIME, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_EVAC_TIME, 	dev_Param_.profiles[2].permit_evac_time)	&&
	writeParameter(ADDR_PARAM_PROCSET_P2_PERMIT_LEAKAGE, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_LEAKAGE, 	dev_Param_.profiles[2].permit_leakage_rate);
}


bool IODevice::uploadProfile_3(){
	return 
	writeParameter(ADDR_PARAM_PROCSET_P3_CONTROL_MODE, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_CONTROL_MODE, 			dev_Param_.profiles[3].control_mode)		&&
	writeParameter(ADDR_PARAM_PROCSET_P3_H1, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1, 								dev_Param_.profiles[3].set_point_H1)		&&
	writeParameter(ADDR_PARAM_PROCSET_P3_H2, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H2, 								dev_Param_.profiles[3].set_point_H2)		&&
	writeParameter(ADDR_PARAM_PROCSET_P3_HYTERESIS, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_HYTERESIS, 				dev_Param_.profiles[3].hyteresis_h2)		&&
	writeParameter(ADDR_PARAM_PROCSET_P3_SPEED, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_SPEED, 						dev_Param_.profiles[3].speed)				&&
	writeParameter(ADDR_PARAM_PROCSET_P3_LAYDOWN_TIME, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_LAYDOWN_TIME, 			dev_Param_.profiles[3].lay_down_time)		&&
	writeParameter(ADDR_PARAM_PROCSET_P3_PERMIT_EVAC_TIME, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_EVAC_TIME, 	dev_Param_.profiles[3].permit_evac_time)	&&
	writeParameter(ADDR_PARAM_PROCSET_P3_PERMIT_LEAKAGE, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_PERMIT_LEAKAGE, 	dev_Param_.profiles[3].permit_leakage_rate);
}


// PROCESSDATA
// ====================================================================================================
bool IODevice::uploadProcessDataOut(){
	ser_port_.resetIO();
	std::vector<uint8_t>Write_PD_Out_Cmd = Write_PD_Cmd_Prefix;

	Write_PD_Out_Cmd.push_back(0x00);

	Write_PD_Out_Cmd.back() |= (pd_Out_.profile << 6); 		// Set Profile Bits
	Write_PD_Out_Cmd.back() |= (pd_Out_.epc_select << 4); 	// Set EPC Select
	Write_PD_Out_Cmd.back() |= (pd_Out_.cm_autoset << 3);		// Set CM
	Write_PD_Out_Cmd.back() |= (pd_Out_.control_mode << 2);	// Set Control Mode
	Write_PD_Out_Cmd.back() |= (pd_Out_.blow_off << 1);		// Set Blow_Off
	Write_PD_Out_Cmd.back() |= (pd_Out_.suction);				// Set suction

	Write_PD_Out_Cmd.push_back(pd_Out_.demand);

	std::vector<uint8_t> buffer;
	int ctr=0;
	do{
		ser_port_.writeVector(Write_PD_Out_Cmd, Write_PD_Out_Cmd.size());
		ser_port_.readVector(buffer);
		ctr++;
		if(ctr>3)
			return false;
	}while(buffer.size()!=WRITE_CMD_RETURN_SIZE);

	return true;
}

bool IODevice::downloadProcessDataIn(){
	// ser_port_.resetIO();
	std::vector<uint8_t> buffer;
	int ctr=0;
	do{
		ser_port_.writeVector(Read_PD_Cmd, Read_PD_Cmd.size());
		ser_port_.readVector(buffer);
		ctr++;
		if(ctr>3)
			return false;
	}while(buffer.size()!= READ_PD_RETURN_SIZE);

	pd_In_.H2 				= (buffer[5]) & 0x01;
	pd_In_.H1 				= (buffer[5] >> 1) & 0x01;
	pd_In_.control_mode 		= (buffer[5] >> 2) & 0x01;
	pd_In_.cm_autoset_ack	= (buffer[5] >> 3) & 0x01;
	pd_In_.epc_select_ack	= (buffer[5] >> 4) & 0x01;
	pd_In_.H3 				= (buffer[5] >> 5) & 0x01;
	pd_In_.status 			= (buffer[5] >> 6) & 0x03;

	pd_In_.epc_value_1		= buffer[6];
	pd_In_.epc_value_2		= buffer[8] + (uint16_t) (buffer[7] << 8);

	return true;
}

bool IODevice::downloadObservationData(){
	full_Obs_.pd_in = pd_In_;
	full_Obs_.pd_out = pd_Out_;

	return 
	readParameter(ADDR_OBS_PROCDATA_VACUUM, ADDR_OBS_PROCDATA_VACUUM_SUB_LIVE, LEN_OBS_PROCDATA_VACUUM_SUB_LIVE, full_Obs_.vacuum.live)		&&
	readParameter(ADDR_OBS_PROCDATA_VACUUM, ADDR_OBS_PROCDATA_VACUUM_SUB_MIN, LEN_OBS_PROCDATA_VACUUM_SUB_MIN, full_Obs_.vacuum.min)		&&
	readParameter(ADDR_OBS_PROCDATA_VACUUM, ADDR_OBS_PROCDATA_VACUUM_SUB_MAX, LEN_OBS_PROCDATA_VACUUM_SUB_MAX, full_Obs_.vacuum.max)		&&

	readParameter(ADDR_OBS_PROCDATA_PMR_VOLTAGE, ADDR_OBS_PROCDATA_PMR_VOLTAGE_SUB_LIVE, LEN_OBS_PROCDATA_PMR_VOLTAGE_SUB_LIVE, full_Obs_.primary_voltage.live)		&&
	readParameter(ADDR_OBS_PROCDATA_PMR_VOLTAGE, ADDR_OBS_PROCDATA_PMR_VOLTAGE_SUB_MIN, LEN_OBS_PROCDATA_PMR_VOLTAGE_SUB_MIN, full_Obs_.primary_voltage.min)		&&
	readParameter(ADDR_OBS_PROCDATA_PMR_VOLTAGE, ADDR_OBS_PROCDATA_PMR_VOLTAGE_SUB_MAX, LEN_OBS_PROCDATA_PMR_VOLTAGE_SUB_MAX, full_Obs_.primary_voltage.max)		&&

	readParameter(ADDR_OBS_PROCDATA_AUX_VOLTAGE, ADDR_OBS_PROCDATA_AUX_VOLTAGE_SUB_LIVE, LEN_OBS_PROCDATA_AUX_VOLTAGE_SUB_LIVE, full_Obs_.auxiliary_voltage.live)	&&
	readParameter(ADDR_OBS_PROCDATA_AUX_VOLTAGE, ADDR_OBS_PROCDATA_AUX_VOLTAGE_SUB_MIN, LEN_OBS_PROCDATA_AUX_VOLTAGE_SUB_MIN, full_Obs_.auxiliary_voltage.min)		&&
	readParameter(ADDR_OBS_PROCDATA_AUX_VOLTAGE, ADDR_OBS_PROCDATA_AUX_VOLTAGE_SUB_MAX, LEN_OBS_PROCDATA_AUX_VOLTAGE_SUB_MAX, full_Obs_.auxiliary_voltage.max)		&&

	readParameter(ADDR_OBS_PROCDATA_TEMPERATURE, ADDR_OBS_PROCDATA_TEMPERATURE_SUB_LIVE, LEN_OBS_PROCDATA_TEMPERATURE_LIVE, full_Obs_.temperature.live)	&&
	readParameter(ADDR_OBS_PROCDATA_TEMPERATURE, ADDR_OBS_PROCDATA_TEMPERATURE_SUB_MIN, LEN_OBS_PROCDATA_TEMPERATURE_MIN, full_Obs_.temperature.min)	&&
	readParameter(ADDR_OBS_PROCDATA_TEMPERATURE, ADDR_OBS_PROCDATA_TEMPERATURE_SUB_MAX, LEN_OBS_PROCDATA_TEMPERATURE_MAX, full_Obs_.temperature.max)	&&

	readParameter(ADDR_OBS_PROCDATA_EVAC_TIME_T0, NO_SUB_INDEX, LEN_OBS_PROCDATA_EVAC_TIME_T0, full_Obs_.evac_time_t0)				&&
	readParameter(ADDR_OBS_PROCDATA_EVAC_TIME_T1, NO_SUB_INDEX, LEN_OBS_PROCDATA_EVAC_TIME_T1, full_Obs_.evac_time_t1)				&&
	readParameter(ADDR_OBS_PROCDATA_LEAKAGE_RATE, NO_SUB_INDEX, LEN_OBS_PROCDATA_LEAKAGE_RATE, full_Obs_.leakage_rate)				&&
	readParameter(ADDR_OBS_PROCDATA_FREEFLOW_VACUUM, NO_SUB_INDEX, LEN_OBS_PROCDATA_FREEFLOW_VACUUM, full_Obs_.free_flow_vacuum)	&&
	readParameter(ADDR_OBS_PROCDATA_LAST_MAX_VACUUM, NO_SUB_INDEX, LEN_OBS_PROCDATA_LAST_MAX_VACUUM, full_Obs_.max_reached_vacuum);

}


// INDIVIDIUAL PARAMETER READ/WRITE
// ==================================================================================================
bool IODevice::writeParameter(uint8_t address, uint8_t sub_index, uint8_t len, uint16_t value){
	// ser_port_.resetIO();
	std::vector<uint8_t> Write_Param_Cmd = Write_Param_Prefix;

	//append address
	Write_Param_Cmd.push_back(address);
	Write_Param_Cmd.push_back(0x00);
	Write_Param_Cmd.push_back(sub_index);
	Write_Param_Cmd.push_back((uint8_t)len);
	//split (int) value into bytes (depend on length)
	for(int i=0;i<len;i++){
		Write_Param_Cmd.push_back((value >> (8*(len-i-1))) & 0xFF);
	}

	//update the size of the vector into the first element
	Write_Param_Cmd[0] = Write_Param_Cmd.size();

	std::vector<uint8_t> buffer;
	int ctr=0;
	do{
		ser_port_.writeVector(Write_Param_Cmd, Write_Param_Cmd.size());
		ser_port_.readVector(buffer);
		ctr++;
		if(ctr>3)
			return false;
	} while(buffer.size()!= WRITE_CMD_RETURN_SIZE);
	return true;
}

bool IODevice::writeParameter(uint8_t address, uint8_t sub_index, uint8_t len, uint8_t value){
	// ser_port_.resetIO();
	std::vector<uint8_t> Write_Param_Cmd = Write_Param_Prefix;

	//append address
	Write_Param_Cmd.push_back(address);
	Write_Param_Cmd.push_back(0x00);
	Write_Param_Cmd.push_back(sub_index);
	Write_Param_Cmd.push_back((uint8_t)len);
	//split (int) value into bytes (depend on length)
	for(int i=0;i<len;i++){
		Write_Param_Cmd.push_back((value >> (8*(len-i-1))) & 0xFF);
	}

	//update the size of the vector into the first element
	Write_Param_Cmd[0] = Write_Param_Cmd.size();

	std::vector<uint8_t> buffer;
	int ctr=0;
	do{
		ser_port_.writeVector(Write_Param_Cmd, Write_Param_Cmd.size());
		ser_port_.readVector(buffer);
		ctr++;
		if(ctr>3)
			return false;
	} while(buffer.size()!= WRITE_CMD_RETURN_SIZE);
	return true;
}

bool IODevice::readParameter(uint8_t address, uint8_t sub_index, uint8_t len, uint16_t &value){
	std::vector<uint8_t> Read_Param_Cmd = Read_Param_Prefix;

	std::vector<uint8_t> v;
	v.resize(1);

	Read_Param_Cmd.push_back(address);
	Read_Param_Cmd.push_back(0x00);
	Read_Param_Cmd.push_back(sub_index);

	// //Catch extra, unwanted feedback from device (usually started with 0x0C), disappears when cmd is repeated
	int ctr=0;
	do{
		ser_port_.writeVector(Read_Param_Cmd, Read_Param_Cmd.size());
		ser_port_.readVector(v);
		ctr++;
		if(ctr>3)
			return false;
	}while(v.size()!= (READ_CMD_RETURN_PREFIX_SIZE+len) || v[v.size()-len-1]!=len);
	int result=0;

	//Combine the bytes of the results
	for(int i=READ_CMD_RETURN_PREFIX_SIZE;i<v.size();i++){
		result += v[i]<<(8*(v.size()-i-1));;
	}
	value = result;
	return true;
}

bool IODevice::readParameter(uint8_t address, uint8_t sub_index, uint8_t len, uint8_t &value){
	std::vector<uint8_t> Read_Param_Cmd = Read_Param_Prefix;

	std::vector<uint8_t> v;
	v.resize(1);

	Read_Param_Cmd.push_back(address);
	Read_Param_Cmd.push_back(0x00);
	Read_Param_Cmd.push_back(sub_index);

	// //Catch extra, unwanted feedback from device (usually started with 0x0C), disappears when cmd is repeated
	int ctr=0;
	do{
		ser_port_.writeVector(Read_Param_Cmd, Read_Param_Cmd.size());
		ser_port_.readVector(v);
		ctr++;
		if(ctr>3)
			return false;
	}while(v.size()!= (READ_CMD_RETURN_PREFIX_SIZE+len) || v[v.size()-len-1]!=len);
	int result=0;

	//Combine the bytes of the results
	for(int i=READ_CMD_RETURN_PREFIX_SIZE;i<v.size();i++){
		result += v[i]<<(8*(v.size()-i-1));;
	}
	value = result;
	return true;
}


bool IODevice::readStringParameter(uint8_t address, uint8_t sub_index, uint8_t len, std::string &value){
	ser_port_.resetIO();
	std::vector<uint8_t> Read_Param_Cmd = Read_Param_Prefix;
	std::vector<uint8_t> v;
	v.resize(1);

	Read_Param_Cmd.push_back(address);
	Read_Param_Cmd.push_back(0x00);
	Read_Param_Cmd.push_back(sub_index);

	std::string str;

	//Catch extra, unwanted feedback from device (usually started with 0x0C), disappears when cmd is repeated
	int ctr=0;
	do{
		ser_port_.writeVector(Read_Param_Cmd, Read_Param_Cmd.size());
		ser_port_.readVector(value);
		ctr++;
		if(ctr>3)
			return false;
	}while(value[value.size()-len-1]!=len);

	for(int i=READ_CMD_RETURN_PREFIX_SIZE;i<value.size();i++){
		str+=(char)value[i];
	}
	value = str;
	return true;

}

bool IODevice::sysCmdFactoryReset(){
	return writeParameter(ADDR_PARAM_DEVSET_SYS_CMD, NO_SUB_INDEX, LEN_PARAM_DEVSET_SYS_CMD, PARAM_DEVSET_SYS_CMD_FACTORY_RESET);
}

bool IODevice::sysCmdCalibrateSensor(){
	return writeParameter(ADDR_PARAM_DEVSET_SYS_CMD, NO_SUB_INDEX, LEN_PARAM_DEVSET_SYS_CMD, PARAM_DEVSET_SYS_CMD_CALIBRATE_SENSOR);
}

bool IODevice::sysCmdCounterReset(){
	return writeParameter(ADDR_PARAM_DEVSET_SYS_CMD, NO_SUB_INDEX, LEN_PARAM_DEVSET_SYS_CMD, PARAM_DEVSET_SYS_CMD_RESET_COUNTER);
}

bool IODevice::sysCmdVoltageReset(){
	return writeParameter(ADDR_PARAM_DEVSET_SYS_CMD, NO_SUB_INDEX, LEN_PARAM_DEVSET_SYS_CMD, PARAM_DEVSET_SYS_CMD_RESET_VOLTAGE);
}

bool IODevice::sysCmdVacuumReset(){
	return writeParameter(ADDR_PARAM_DEVSET_SYS_CMD, NO_SUB_INDEX, LEN_PARAM_DEVSET_SYS_CMD, PARAM_DEVSET_SYS_CMD_RESET_VACUUM);
}

//Test
// int main(){
// 	PortAttributes testpa;
// 	testpa.device_name = "/dev/ttyACM0";
// 	IODevice testdev;
// 	testdev.init(testpa);
// 	testdev.connect();
// 	// testdev.writeParameter(ADDR_PARAM_PROCSET_P0_H1, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1, 567);
// 	// std::cout<<testdev.readParameter(ADDR_PARAM_PROCSET_P0_H1, NO_SUB_INDEX, LEN_PARAM_PROCSET_P_H1)<<std::endl;
// 	ProcessDataOut pdo;
// 	ProcessDataIn pdi;

// 	pdo.suction = 1;
// 	// pdo.blow_off = 1;
// 	pdo.control_mode = 0;
// 	pdo.demand = 26;
// 	testdev.setProcessDataOut(pdo);
// 	testdev.uploadProcessDataOut();	
// 	do{
// 		testdev.downloadProcessDataIn();
// 		pdi = testdev.getProcessDataIn();
// 		std::cout<<pdi.epc_value_2<<std::endl;
// 		// std::cout<<(int)pdi.control_mode<<std::endl;
// 	}while(!pdi.H2);

// 	pdo.suction = 0;
// 	testdev.setProcessDataOut(pdo);
// 	testdev.uploadProcessDataOut();	
// 	std::cin.get();
// 	testdev.disconnect();
// }