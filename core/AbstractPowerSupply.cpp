#include "AbstractPowerSupply.h"
#include <common/debug/core/debug.h>
 #define DESC_EVENT(x) \
case x: return std::string (# x);

using namespace common::powersupply;
std::string decodeEvent(PowerSupplyEvents ev){
    uint64_t al=ev;
    
    switch(ev){
        DESC_EVENT(POWER_SUPPLY_EVENT_DOOR_OPEN)
        DESC_EVENT(POWER_SUPPLY_EVENT_OVER_TEMP)
        DESC_EVENT(POWER_SUPPLY_FUSE_FAULT)
        DESC_EVENT(POWER_SUPPLY_EARTH_FAULT)
        DESC_EVENT(POWER_SUPPLY_OVER_VOLTAGE)
        DESC_EVENT(POWER_SUPPLY_OVER_CURRENT)
        DESC_EVENT(POWER_SUPPLY_COMMUNICATION_FAILURE)
        DESC_EVENT(POWER_SUPPLY_MAINUNIT_FAIL)
        DESC_EVENT(POWER_SUPPLY_EXTERNAL_INTERLOCK)
        DESC_EVENT(POWER_SUPPLY_SETPOINT_CARD_FAULT)
        DESC_EVENT(POWER_SUPPLY_CUBICLE_OVT)
        DESC_EVENT(POWER_SUPPLY_DCCT_OVT)
        DESC_EVENT(POWER_SUPPLY_DCCT_FAULT)
        DESC_EVENT(POWER_SUPPLY_ACTIVE_FILTER_FUSE)
        DESC_EVENT(POWER_SUPPLY_ACTIVE_FILTER_OVT)
        DESC_EVENT(    POWER_SUPPLY_DIODE_OVT)
        DESC_EVENT(    POWER_SUPPLY_DIODE_FAULT)
        DESC_EVENT(    POWER_SUPPLY_AC_UNBALANCE)
         DESC_EVENT(   POWER_SUPPLY_PHASE_LOSS)
         DESC_EVENT(   POWER_SUPPLY_AIR_FLOW)
        DESC_EVENT(    POWER_SUPPLY_TRANSFORMER_OVT)
         DESC_EVENT(   POWER_SUPPLY_SNUBBER_FUSE)
        DESC_EVENT(    POWER_SUPPLY_SCR_FUSE)
        DESC_EVENT(    POWER_SUPPLY_SCR_OVT)
        DESC_EVENT(    POWER_SUPPLY_CHOKE_OVT)
        DESC_EVENT(    POWER_SUPPLY_PASS_FILTER)
        DESC_EVENT(    POWER_SUPPLY_ALARM_UNDEF)
        default:
            return std::string("undefined");
    }

}


   
 int AbstractPowerSupply::setCurrentSensibility(float sens){
     DERR("not implemented");
     return POWER_SUPPLY_ERROR_SETTING_CHANNEL_SENSIBILITY;
 }
            
        
int AbstractPowerSupply::setVoltageSensibility(float sens){
     DERR("not implemented");
     return POWER_SUPPLY_ERROR_SETTING_CHANNEL_SENSIBILITY;
}