#ifndef __SCHMALZ_ECBPI_H__
#define __SCHMALZ_ECBPI_H__


//
//	VendorID			234
//	DeviceID			100310
//	SIO-Mode		 	Yes
//	Baudrate			38.4 KBd (COM2)
//	minimum Cycle Time	3.4ms
//	ProcessData Input	4bytes
//	ProcessData Output	2bytes

// IDENTIFICATION - DEVICE MANAGEMENT - READ ONLY
// =====================================================
// ADDRESS
#define ADDR_ID_DM_VENDOR_NAME					0x10
#define ADDR_ID_DM_VENDOR_TEXT					0x11
#define ADDR_ID_DM_PRODUCT_NAME					0x12
#define ADDR_ID_DM_PRODUCT_ID					0x13
#define ADDR_ID_DM_PRODUCT_TEXT					0x14
#define ADDR_ID_DM_SERIAL_NUMBER				0x15
#define ADDR_ID_DM_HARDWARE_REV					0x16
#define ADDR_ID_DM_FIRMWARE_REV					0x17
#define ADDR_ID_DM_UNIQUE_DEV_ID				0xF0
#define ADDR_ID_DM_FEATURE_LIST					0xF1
#define ADDR_ID_DM_ARTICLE_NUM					0xFA
#define ADDR_ID_DM_ARTICLE_REV					0xFB
#define ADDR_ID_DM_PRODUCTION_CODE				0xFC
#define ADDR_ID_DM_PRODUCTION_TEXT				0xFE

// LENGTH
#define LEN_ID_DM_VENDOR_NAME					15
#define LEN_ID_DM_VENDOR_TEXT					15
#define LEN_ID_DM_PRODUCT_NAME					32
#define LEN_ID_DM_PRODUCT_ID					32
#define LEN_ID_DM_PRODUCT_TEXT					30
#define LEN_ID_DM_SERIAL_NUMBER					9
#define LEN_ID_DM_HARDWARE_REV					2
#define LEN_ID_DM_FIRMWARE_REV					4
#define LEN_ID_DM_UNIQUE_DEV_ID					20
#define LEN_ID_DM_FEATURE_LIST					11
#define LEN_ID_DM_ARTICLE_NUM					14
#define LEN_ID_DM_ARTICLE_REV					2
#define LEN_ID_DM_PRODUCTION_CODE				3
#define LEN_ID_DM_PRODUCTION_TEXT				64

// IDENTIFICATION - DEVICE LOCALIZATION - READ/WRITE
// =====================================================
// ADDRESS
#define ADDR_ID_DL_APP_SPEC_TAG					0x18
#define ADDR_ID_DL_EQUIPMENT_ID					0xF2
#define ADDR_ID_DL_GEOLOCATION					0xF6
#define ADDR_ID_DL_WEBLINK_IODD					0xF7
#define ADDR_ID_DL_IOT_SERVER					0xF8
#define ADDR_ID_DL_STORAGE_LOCATION				0xF9
#define ADDR_ID_DL_INSTALLATION_DATE			0xFD

//LENGTH
#define LEN_ID_DL_APP_SPEC_TAG					32
#define LEN_ID_DL_EQUIPMENT_ID					64
#define LEN_ID_DL_GEOLOCATION					64
#define LEN_ID_DL_WEBLINK_IODD					64
#define LEN_ID_DL_IOT_SERVER					64
#define LEN_ID_DL_STORAGE_LOCATION				32
#define LEN_ID_DL_INSTALLATION_DATE				16


// PARAMETERS - DEVICE SETTINGS
// SYSTEM COMMAND - WRITE ONLY
// ====================================================
#define ADDR_PARAM_DEVSET_SYS_CMD				0x02
#define LEN_PARAM_DEVSET_SYS_CMD				1

#define PARAM_DEVSET_SYS_CMD_FACTORY_RESET		0x82
#define PARAM_DEVSET_SYS_CMD_CALIBRATE_SENSOR	0xA5
#define PARAM_DEVSET_SYS_CMD_RESET_COUNTER		0xA7
#define PARAM_DEVSET_SYS_CMD_RESET_VOLTAGE		0xA8
#define PARAM_DEVSET_SYS_CMD_RESET_VACUUM		0xA9



// INITIAL SETTINGS - READ/WRITE
// =====================================================
// ADDRESS
#define ADDR_PARAM_DEVSET_INIT_BLOWOFF			0x45
#define ADDR_PARAM_DEVSET_INIT_SOFTSTART		0x46
#define ADDR_PARAM_DEVSET_INIT_OUT2_FCN			0x47
#define ADDR_PARAM_DEVSET_INIT_OUT3_FCN			0x48
#define ADDR_PARAM_DEVSET_INIT_SIG_TYPE			0x49
#define ADDR_PARAM_DEVSET_INIT_SIG_TYPE_SUB_IN	1
#define ADDR_PARAM_DEVSET_INIT_SIG_TYPE_SUB_OUT	2
#define ADDR_PARAM_DEVSET_INIT_VACUUM_UNIT		0x4A
#define ADDR_PARAM_DEVSET_INIT_OUT_FILTER		0x4B
#define ADDR_PARAM_DEVSET_INIT_ECO_MODE			0x4C
#define ADDR_PARAM_DEVSET_INIT_DISPLAY_ROTATION	0x4F

// LENGTH
#define LEN_PARAM_DEVSET_INIT_BLOWOFF			1
#define LEN_PARAM_DEVSET_INIT_SOFTSTART			1
#define LEN_PARAM_DEVSET_INIT_OUT2_FCN			1
#define LEN_PARAM_DEVSET_INIT_OUT3_FCN			1
#define LEN_PARAM_DEVSET_INIT_SIG_TYPE			1
#define LEN_PARAM_DEVSET_INIT_SIG_TYPE_SUB_IN	1
#define LEN_PARAM_DEVSET_INIT_SIG_TYPE_SUB_OUT	1
#define LEN_PARAM_DEVSET_INIT_VACUUM_UNIT		1
#define LEN_PARAM_DEVSET_INIT_OUT_FILTER		1
#define LEN_PARAM_DEVSET_INIT_ECO_MODE			1
#define LEN_PARAM_DEVSET_INIT_DISPLAY_ROTATION	1


// PARAMETERS - PROCESS SETTINGS - READ/WRITE
// PROFILE 0
// =====================================================
// ADDRESS
#define ADDR_PARAM_PROCSET_P0_CONTROL_MODE		0x4E
#define ADDR_PARAM_PROCSET_P0_H1				0x64
#define ADDR_PARAM_PROCSET_P0_SPEED				0x65
#define ADDR_PARAM_PROCSET_P0_H2				0x66
#define ADDR_PARAM_PROCSET_P0_HYTERESIS			0x67
#define ADDR_PARAM_PROCSET_P0_LAYDOWN_TIME		0x6A
#define ADDR_PARAM_PROCSET_P0_PERMIT_EVAC_TIME	0x6B
#define ADDR_PARAM_PROCSET_P0_PERMIT_LEAKAGE	0x6C
#define ADDR_PARAM_PROCSET_P0_NAME 				0x77

// LENGTH
#define LEN_PARAM_PROCSET_P_CONTROL_MODE		1
#define LEN_PARAM_PROCSET_P_H1					2
#define LEN_PARAM_PROCSET_P_SPEED				1
#define LEN_PARAM_PROCSET_P_H2					2
#define LEN_PARAM_PROCSET_P_HYTERESIS			2
#define LEN_PARAM_PROCSET_P_LAYDOWN_TIME		2
#define LEN_PARAM_PROCSET_P_PERMIT_EVAC_TIME	2
#define LEN_PARAM_PROCSET_P_PERMIT_LEAKAGE		2
#define LEN_PARAM_PROCSET_P_NAME 				32

// PROFILE 1
// =====================================================
// ADDRESS
#define ADDR_PARAM_PROCSET_P1_CONTROL_MODE		0xB5
#define ADDR_PARAM_PROCSET_P1_H1				0xB6
#define ADDR_PARAM_PROCSET_P1_SPEED				0xB7
#define ADDR_PARAM_PROCSET_P1_H2				0xB8
#define ADDR_PARAM_PROCSET_P1_HYTERESIS			0xB9
#define ADDR_PARAM_PROCSET_P1_LAYDOWN_TIME		0xBA
#define ADDR_PARAM_PROCSET_P1_PERMIT_EVAC_TIME	0xBB
#define ADDR_PARAM_PROCSET_P1_PERMIT_LEAKAGE	0xBC
#define ADDR_PARAM_PROCSET_P1_NAME 				0xC7

// PROFILE 2
// =====================================================
// ADDRESS
#define ADDR_PARAM_PROCSET_P2_CONTROL_MODE		0xC9
#define ADDR_PARAM_PROCSET_P2_H1				0xCA
#define ADDR_PARAM_PROCSET_P2_SPEED				0xCB
#define ADDR_PARAM_PROCSET_P2_H2				0xCC
#define ADDR_PARAM_PROCSET_P2_HYTERESIS			0xCD
#define ADDR_PARAM_PROCSET_P2_LAYDOWN_TIME		0xCE
#define ADDR_PARAM_PROCSET_P2_PERMIT_EVAC_TIME	0xCF
#define ADDR_PARAM_PROCSET_P2_PERMIT_LEAKAGE	0xD0
#define ADDR_PARAM_PROCSET_P2_NAME 				0xDB

// PROFILE 3
// ====================================================
// ADDRESS
#define ADDR_PARAM_PROCSET_P3_CONTROL_MODE		0xDD
#define ADDR_PARAM_PROCSET_P3_H1				0xDE
#define ADDR_PARAM_PROCSET_P3_SPEED				0xDF
#define ADDR_PARAM_PROCSET_P3_H2				0xE0
#define ADDR_PARAM_PROCSET_P3_HYTERESIS			0xE1
#define ADDR_PARAM_PROCSET_P3_LAYDOWN_TIME		0xE2
#define ADDR_PARAM_PROCSET_P3_PERMIT_EVAC_TIME	0xE3
#define ADDR_PARAM_PROCSET_P3_PERMIT_LEAKAGE	0xE4
#define ADDR_PARAM_PROCSET_P3_NAME 				0xEF

// OBSERVATION - MONITORING - READ ONLY
// PROCESS DATA
// ====================================================
// ADDRESS
#define ADDR_OBS_PROCDATA_PD_IN_COPY			0x28
#define ADDR_OBS_PROCDATA_PD_OUT_COPY			0x29

#define ADDR_OBS_PROCDATA_VACUUM				0x40
#define ADDR_OBS_PROCDATA_VACUUM_SUB_ALL		0
#define ADDR_OBS_PROCDATA_VACUUM_SUB_LIVE		1
#define ADDR_OBS_PROCDATA_VACUUM_SUB_MIN		2
#define ADDR_OBS_PROCDATA_VACUUM_SUB_MAX		3

#define ADDR_OBS_PROCDATA_PMR_VOLTAGE			0x42
#define ADDR_OBS_PROCDATA_PMR_VOLTAGE_SUB_ALL	0
#define ADDR_OBS_PROCDATA_PMR_VOLTAGE_SUB_LIVE	1
#define ADDR_OBS_PROCDATA_PMR_VOLTAGE_SUB_MIN	2
#define ADDR_OBS_PROCDATA_PMR_VOLTAGE_SUB_MAX	3

#define ADDR_OBS_PROCDATA_AUX_VOLTAGE			0x43
#define ADDR_OBS_PROCDATA_AUX_VOLTAGE_SUB_ALL	0
#define ADDR_OBS_PROCDATA_AUX_VOLTAGE_SUB_LIVE	1
#define ADDR_OBS_PROCDATA_AUX_VOLTAGE_SUB_MIN	2
#define ADDR_OBS_PROCDATA_AUX_VOLTAGE_SUB_MAX	3

#define ADDR_OBS_PROCDATA_TEMPERATURE			0x44
#define ADDR_OBS_PROCDATA_TEMPERATURE_SUB_LIVE	1
#define ADDR_OBS_PROCDATA_TEMPERATURE_SUB_MIN	2
#define ADDR_OBS_PROCDATA_TEMPERATURE_SUB_MAX	3

#define ADDR_OBS_PROCDATA_EVAC_TIME_T0			0x94
#define ADDR_OBS_PROCDATA_EVAC_TIME_T1			0x95
#define ADDR_OBS_PROCDATA_LEAKAGE_RATE			0xA0
#define ADDR_OBS_PROCDATA_FREEFLOW_VACUUM		0xA1
#define ADDR_OBS_PROCDATA_LAST_MAX_VACUUM		0xA4


// LENGTH
#define LEN_OBS_PROCDATA_PD_IN_COPY				4
#define LEN_OBS_PROCDATA_PD_OUT_COPY			2

#define LEN_OBS_PROCDATA_VACUUM_SUB_ALL			6
#define LEN_OBS_PROCDATA_VACUUM_SUB_LIVE		2
#define LEN_OBS_PROCDATA_VACUUM_SUB_MIN			2
#define LEN_OBS_PROCDATA_VACUUM_SUB_MAX			2

#define LEN_OBS_PROCDATA_PMR_VOLTAGE_SUB_ALL	6
#define LEN_OBS_PROCDATA_PMR_VOLTAGE_SUB_LIVE	2
#define LEN_OBS_PROCDATA_PMR_VOLTAGE_SUB_MIN	2
#define LEN_OBS_PROCDATA_PMR_VOLTAGE_SUB_MAX	2

#define LEN_OBS_PROCDATA_AUX_VOLTAGE_SUB_ALL	6
#define LEN_OBS_PROCDATA_AUX_VOLTAGE_SUB_LIVE	2
#define LEN_OBS_PROCDATA_AUX_VOLTAGE_SUB_MIN	2
#define LEN_OBS_PROCDATA_AUX_VOLTAGE_SUB_MAX	2	

#define LEN_OBS_PROCDATA_TEMPERATURE_LIVE		2
#define LEN_OBS_PROCDATA_TEMPERATURE_MIN		2
#define LEN_OBS_PROCDATA_TEMPERATURE_MAX		2

#define LEN_OBS_PROCDATA_EVAC_TIME_T0			2
#define LEN_OBS_PROCDATA_EVAC_TIME_T1			2
#define LEN_OBS_PROCDATA_LEAKAGE_RATE			2
#define LEN_OBS_PROCDATA_FREEFLOW_VACUUM		2
#define LEN_OBS_PROCDATA_LAST_MAX_VACUUM		2

// DIAGNOSIS - DEVICE STATUS - READ ONLY
// =====================================================
// ADDRESS
#define ADDR_DIAG_DEV_STATUS_ERROR_COUNT		0x20
#define ADDR_DIAG_DEV_STATUS_SHORT				0x24
#define ADDR_DIAG_DEV_STATUS_DETAILED			0x25
#define ADDR_DIAG_DEV_STATUS_EXTENDED			0x8A
#define ADDR_DIAG_DEV_STATUS_EXTENDED_SUB_TYPE	1
#define ADDR_DIAG_DEV_STATUS_EXTENDED_SUB_ID	2
#define ADDR_DIAG_DEV_STATUS_NFC_STATUS			0x8B
#define ADDR_DIAG_DEV_STATUS_ACTIVE_ERROR_CODE	0x82

// LENGTH
#define LEN_DIAG_DEV_STATUS_ERROR_COUNT			2
#define LEN_DIAG_DEV_STATUS_SHORT				1
#define LEN_DIAG_DEV_STATUS_DETAILED			60
#define LEN_DIAG_DEV_STATUS_EXTENDED_SUB_TYPE	1
#define LEN_DIAG_DEV_STATUS_EXTENDED_SUB_ID		2
#define LEN_DIAG_DEV_STATUS_NFC_STATUS			1
#define LEN_DIAG_DEV_STATUS_ACTIVE_ERROR_CODE	1

// DIAGNOSIS - CONDITION MONITORING - READ ONLY
#define ADDR_DIAG_CONDITION_MONITORING			0x92
#define LEN_DIAG_CONDITION_MONITORING			1

// DIAGNOSIS - ENERGY MONITORING - READ ONLY
#define ADDR_DIAG_ENERGY_MONITORING				0x9D
#define LEN_DIAG_ENERGY_MONITORING				2

// DIAGNOSIS - PREDICTIVE MAINTENANCE
#define ADDR_DIAG_PREDICTIVE_MAINTENANCE_QLT	0xA2
#define ADDR_DIAG_PREDICTIVE_MAINTENANCE_PFM	0xA3
#define LEN_DIAG_PREDICTIVE_MAINTENANCE_QLT		1
#define LEN_DIAG_PREDICTIVE_MAINTENANCE_PFM		1

#define NO_SUB_INDEX							0x00
#define READ_CMD_RETURN_PREFIX_SIZE				0x06
#define WRITE_CMD_RETURN_SIZE					0x05
#define READ_PD_RETURN_SIZE						0x0A
// #define 


#include "SerialPort.h"

struct DeviceIdentification{
	std::string vendor_name;
	std::string vendor_text;
	std::string product_name;
	std::string product_id;
	std::string product_text;
	std::string serial_number;
	std::string hardware_revision;
	std::string firmware_revision;
	std::string unique_dev_id;
	std::string feature_list;
	std::string article_number;
	std::string article_revision;
	std::string product_code;
	std::string product_text_detailed;
};

struct InitialSettings{
	uint8_t 	blow_off_mode;		// 0: Externally Ctrl, 1: Internally Ctrl, 2: Ext. Ctrl. w/ time dependent
	uint8_t		soft_start;			// 0: Disable, 1: Enable (should be enabled always)
	uint8_t		out2_function;		// 0: Normally Open (NO), 1: Normally Close (NC)
	uint8_t		out3_function;		// 0: NO, 1: NC
	uint8_t		signal_type_input;	// 0: PNP, 1: NPN
	uint8_t		signal_type_output;	// 0: PNP, 1: NPN
	uint8_t		vacuum_unit;			// 0: mbar, 1: kPa, 2: inHg, 3: psi
	uint8_t 	output_filter;		// 0: Off, 1: 10ms, 2: 50ms, 3: 200ms
	uint8_t 	eco_mode;			// 0: Off, 1: On, 2: Low
	uint8_t		display_rotation;	// 0: Standard, 1: Rotated
};

struct Profile{
	uint8_t		control_mode;		// 0: vacuum, 1: Speed
	uint16_t	set_point_H1;		// less than 998
	uint16_t	set_point_H2;		// (h2+2)-(0.9*H1)
	uint16_t	hyteresis_h2;		// 10-(H2-2)
	uint8_t		speed;				// 0-100
	uint16_t	lay_down_time;		// 100-9999
	uint16_t	permit_evac_time;		// 0, 10-9999
	uint16_t	permit_leakage_rate;	// 1-999
	std::string	name;				// should be less than 32 bytes
};

struct ProcessDataIn{
	uint8_t		H1;						//True if actual vacuum value is over H1
	uint8_t		H2;						//True if actual vacuum value is over H2
	uint8_t		H3;						//True if part is detached after a suction cycle
	uint8_t		control_mode;	//True if in speed mode, in control mode otherwise
	uint8_t		epc_select_ack;
	uint8_t		cm_autoset_ack;
	uint8_t		epc_value_1;
	uint16_t	epc_value_2;
	uint8_t		status;

	ProcessDataIn():
		H1(0),
		H2(0),
		H3(0),
		control_mode(0),
		epc_select_ack(0),
		cm_autoset_ack(0),
		epc_value_1(0),
		epc_value_2(0),
		status(0){}

};

struct ProcessDataOut{
	uint8_t	suction;
	uint8_t	blow_off;
	uint8_t	control_mode;			//True: Speed Demand Control, False: Setpoint Control
	uint8_t	cm_autoset;
	uint8_t	epc_select;				
	uint8_t	profile;
	uint8_t	demand;					//Speed mode is True: 1-100 (capacity), otherwise, limit for H2 in 10mbar

	ProcessDataOut() : 
		suction(0),
		blow_off(0),
		control_mode(0),
		cm_autoset(0),
		epc_select(0),
		profile(0),
		demand(0){}
	//EPC Select:
	//0: epc_value_1: Actual Power in %, epc_value_2: vacuum in mbar
	//1: epc_value_1: CM_Warnings, epc_value_2: Evacuation Time T1 in ms
	//2: epc_value_1: Leakage last cycle, epc_value_2: Last measured free-flow vacuum in mbar
	//3: epc_value_1: Primary Voltage in 0.1V, epc_value_2: Energy Consumption.
};

struct MonitorValue{
	uint16_t live;
	uint16_t min;
	uint16_t max;
};

struct ProcessData{
	ProcessDataIn pd_in;
	ProcessDataOut pd_out;
	MonitorValue vacuum;
	MonitorValue primary_voltage;
	MonitorValue auxiliary_voltage;
	MonitorValue temperature;
	uint16_t evac_time_t0;
	uint16_t evac_time_t1;
	uint16_t leakage_rate;
	uint16_t free_flow_vacuum;
	uint16_t max_reached_vacuum;
};

struct Parameters{
	InitialSettings	initial_settings;
	Profile 		profiles[4];
};


class IODevice {
public:
	IODevice();
	~IODevice();

	void init(const PortAttributes& pa);
	bool connect();
	bool disconnect();


	// IO-LINK COMM b/w device and PC
	bool downloadParameters();	
	bool uploadParameters();

	bool downloadInitialSettings();	// from device
	bool uploadInitialSettings(); // to device

	bool downloadProfile(const uint8_t& i);
	bool uploadProfile(const uint8_t& i);

	bool downloadObservationData();	// read-only

	bool uploadProcessDataOut();
	bool downloadProcessDataIn();

	bool sysCmdFactoryReset();
	bool sysCmdCalibrateSensor();
	bool sysCmdCounterReset();
	bool sysCmdVoltageReset();
	bool sysCmdVacuumReset();

	//Internal
	DeviceIdentification getDeviceID(){return dev_ID_;};		// read only params from device

	void setParameters(const Parameters& p){dev_Param_ = p;};
	Parameters getParameters(){return dev_Param_;};

	void setInitialSettings(const InitialSettings& init_settings){dev_Param_.initial_settings = init_settings;};	//to private member
	InitialSettings getInitialSettings(){return dev_Param_.initial_settings;};
	
	void setProfile(const Profile& p, const uint8_t& i){dev_Param_.profiles[i] = p;};
	Profile getProfile(const int& i){return dev_Param_.profiles[i];}; 

	ProcessData getProcessData(){return full_Obs_;};

	void setProcessDataOut(const ProcessDataOut& pdo){pd_Out_ = pdo;};
	ProcessDataOut getProcessDataOut(){return pd_Out_;};

	ProcessDataIn getProcessDataIn(){return pd_In_;};

private:

	DeviceIdentification 	dev_ID_;
	Parameters 				dev_Param_;		//params of the device on PC
	ProcessDataIn 			pd_In_;			//Sensors value from device
	ProcessDataOut 			pd_Out_;		//Command from PC to device
	ProcessData 			full_Obs_;		//Observation and diagnosis
	SerialPort				ser_port_;


	bool writeParameter(uint8_t address, uint8_t sub_index, uint8_t len, uint16_t value);
	bool readParameter(uint8_t address, uint8_t sub_index, uint8_t len, uint16_t &value);
	bool readStringParameter(uint8_t address, uint8_t sub_index, uint8_t len, std::string &value);

	bool downloadDeviceID();

	bool downloadProfile_0();
	bool downloadProfile_1();
	bool downloadProfile_2();
	bool downloadProfile_3();

	bool uploadProfile_0();
	bool uploadProfile_1();
	bool uploadProfile_2();
	bool uploadProfile_3();

	//COMMANDS IN HEX - Only for SICK SiLink 2 USB to IO-Link Master
	std::vector<uint8_t> Enable_IOLink_Comm{0x24, 0x00, 0x02, 0x0F, 0x00, 0x00, 0x0B, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20};
	std::vector<uint8_t> Disable_IOLink_Comm{0x24, 0x00, 0x02, 0x0F, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20};
	//Write the PD_OUT, need 2 more bytes
	std::vector<uint8_t> Write_PD_Cmd_Prefix{0x07, 0x00, 0x02, 0x0B, 0x00};
	//Read the PD_IN, completed
	std::vector<uint8_t> Read_PD_Cmd{0x05, 0x00, 0x02, 0x0A, 0x00};

	//Write Param
	//First byte should be changed to be equal to the byte size of the data packet
	//Format:
	//<Total number of bytes> <0x00> <0x02> <0x03> <0x00> <Register Address> <0x00> <Dec Sub Index> <Number of bytes for value> <Value, might be more than 1 byte>
	//Remember to change the value of the first element accordingly
	std::vector<uint8_t> Write_Param_Prefix{0x0A, 0x00, 0x02, 0x03, 0x00};

	//Read Param: Need 3 more bytes: <register address> <0x00> <Decimal Sub Index>
	std::vector<uint8_t> Read_Param_Prefix{0x08, 0x00, 0x02, 0x02, 0x00};



};
//TODO
//DIAGNOSIS IN DETAIL



#endif //__SCHMALZ_ECBPI_H__