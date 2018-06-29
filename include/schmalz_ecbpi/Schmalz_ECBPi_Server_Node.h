#ifndef __SCHMALZ_ECBPI_NODE_H__
#define	__SCHMALZ_ECBPI_NODE_H__

#include <ros/ros.h>
#include <ros/time.h>

#include <std_srvs/Trigger.h>


#include "Schmalz_ECBPi.h"

#include "schmalz_ecbpi/DeviceID.h"
#include "schmalz_ecbpi/State.h"
#include "schmalz_ecbpi/Command.h"
#include "schmalz_ecbpi/InitialSettings.h"
#include "schmalz_ecbpi/Profile.h"
#include "schmalz_ecbpi/Observation.h"


#include "schmalz_ecbpi/GetInitialSettings.h"
#include "schmalz_ecbpi/GetProfile.h"
#include "schmalz_ecbpi/GetObservation.h"

#include "schmalz_ecbpi/SetInitialSettings.h"
#include "schmalz_ecbpi/SetProfile.h"


// GLOBAL VARIABLES
PortAttributes port_attributes;
IODevice cobot_pump;
bool isInit=false;

// Get params
int getParameters();

// PUBLISH
// void publishState();

// CALLBACKS
void commandCB(const schmalz_ecbpi::CommandConstPtr& ptr);


// SERVICES
bool init_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool halt_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool recover_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool factoryReset_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool calibrateSensor_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool resetCounter_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool resetVoltage_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
bool resetVacuum_SrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);


bool getInitialSettings_SrvCB(schmalz_ecbpi::GetInitialSettings::Request &req, schmalz_ecbpi::GetInitialSettings::Response &res);
bool getProfile_SrvCB(schmalz_ecbpi::GetProfile::Request &req, schmalz_ecbpi::GetProfile::Response &res);
bool getObservation_SrvCB(schmalz_ecbpi::GetObservation::Request &req, schmalz_ecbpi::GetObservation::Response &res);

bool setInitialSettings_SrvCB(schmalz_ecbpi::SetInitialSettings::Request &req, schmalz_ecbpi::SetInitialSettings::Response &res);
bool setProfile_SrvCB(schmalz_ecbpi::SetProfile::Request &req, schmalz_ecbpi::SetProfile::Response &res);

#endif //__SCHMALZ_ECBPI_NODE_H__