/* 
 * File:   AL250PowerSupply.h
 * Author: alex
 *
 * Created on 20 novembre 2015, 16.31
 */

#ifndef AL250POWERSUPPLY_H
#define	AL250POWERSUPPLY_H

#include <common/powersupply/core/AbstractPowerSupply.h>
#include <common/modbus/models/HAZEMEYER/HazemeyerLib.h>
#include <common/misc/driver/ConfigDriverMacro.h>
#include <map>
#include <vector>
#include <string>
#define NODE_OPERATION_NOT_SUPPORTED -10000

namespace chaos {
  namespace common{
    namespace data{
      class CDataWrapper;
    }
  }
}
/*

*/
//#define ALEDEBUG
namespace common
{
    class CompareStdStr{
public:
    bool operator() (const std::string A,const std::string B) const    {
        bool ret;
       //cout << "comparing " << A.c_str() << " with " << B.c_str();
     ret= strcmp(A.c_str(),B.c_str());
     //cout << " result = " << ret << endl;
     return ret;
    };
};
    namespace powersupply
    {
        
        class AL250: public AbstractPowerSupply 
        {
            static const int DEFAULT_NOT_ALLOWED=NODE_OPERATION_NOT_SUPPORTED;
            static const int NumChannels=9;
            
        public:
            
            class ChannelPhysicalMap
            {
            public:
                Hazemeyer::Corrector * driverPointer;
                std::vector<bool> ConnectedChannels;
                ChannelPhysicalMap() {
                    
                    this->driverPointer=NULL;
                    for (int i=0; i < AL250::NumChannels;i++)
                        this->ConnectedChannels.push_back(false);
                };
                ChannelPhysicalMap(Hazemeyer::Corrector* drv,std::vector<bool> cChannels) {
                    this->driverPointer=drv;
                    for (int i=0; i < AL250::NumChannels;i++)
                        this->ConnectedChannels.push_back(cChannels[i]);
                    
                };
                
                
            };
            
            
            
            /**
             * 
             * @brief Constructor.
               @param Parameters. The string with the serial parameter 
             * for the connection
             * @param sl.  The "channel" of the current power supply. Each channel
             * is mapped on a different power supply unit. ch 0 refers to Main
             */
            AL250(const std::string Parameters, int sl=0);
            #ifdef CHAOS
                AL250(::common::modbus::AbstractModbusChannel_psh channel,const chaos::common::data::CDataWrapper&config);
            #endif
            
            
            
            ~AL250();
            /**
             * Polarity is set only on the setpoint software emulated. Nothing
             * change to the hardware until the command startCurrentRamp
             * @param pol
             * @param timeo_ms
             * @return 
             */
            int setPolarity(int pol,uint32_t timeo_ms=0){   return 0;}
            int getPolarity(int* pol,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            /**
             * SET POINT is only Software Emulated. The Hardware doesn't have
             * nor ramps nor set points. It just sets currents when you ask, with a 
             * predefined and not changeable ramp time.
             * @param current
             * @param timeo_ms
             * @return 
             */
            int setCurrentSP(float current,uint32_t timeo_ms=0);
            /**
             * This member doesn't calls any hardware
             * @param current    the variable where current set point will be stored
             * @param timeo_ms   no sense.
             * @return 
             */
            int getCurrentSP(float* current,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            int startCurrentRamp(uint32_t timeo_ms=0);
            int getVoltageOutput(float* volt,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            int getCurrentOutput(float* current,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            int setCurrentRampSpeed(float asup,float asdown,uint32_t timeo_ms=0) {return DEFAULT_NOT_ALLOWED;}
            /**
             * @brief ResetAlarms by resetting the mapped channel. No selective
             * alarm reset is done.
             * @param alrm
             * @param timeo_ms
             * @return 
             */
            int resetAlarms(uint64_t alrm,uint32_t timeo_ms=0);
            /**
             * 
             * @param alrm =Here will be charged the FAULT_REGISTER of the mapped
             * channel
             * @param timeo_ms
             * @return 
             */
            int getAlarms(uint64_t*alrm,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            /**
             * Send the OFF command to the appropriate mapped channel
             * @param timeo_ms
             * @return 
             */
            int shutdown(uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            int standby(uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            int poweron(uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            int getState(int* state,std::string& desc,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            int init();
            int deinit();
            int getSWVersion(std::string& version,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT) {version = "AL250SW"; return DEFAULT_NOT_ALLOWED;}
            int getHWVersion(std::string& version,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT) {version = "AL250HW"; return DEFAULT_NOT_ALLOWED;}
            int getCurrentSensibility(float *sens);
            int getCurrentSensibilityOnSet(float *sens);
            int getVoltageSensibility(float *sens);
            int getMaxMinCurrent(float*max,float*min);
            /**
             * This is a Current Supply. You cannot set any voltage.
             * @param max   unuseful
             * @param min   unuseful
             * @return DEFAULT_NOT_ALLOWED
             */
            int getMaxMinVoltage(float*max,float*min)  {return DEFAULT_NOT_ALLOWED;}
            int getAlarmDesc(uint64_t* alarm);
            std::string getAlarmDescr(uint64_t alarm);
            int forceMaxCurrent(float max);
            int forceMaxVoltage(float max) {return DEFAULT_NOT_ALLOWED;}
            char* getConnectionParameters() { return this->ConnectionParameters;}
            
            uint64_t getFeatures()  {return POWER_SUPPLY_FEAT_BIPOLAR;}

            
            
        private:
            ChaosUniquePtr<chaos::common::data::CDataWrapper>driverJsonConfig;
            char* ConnectionParameters;
            unsigned short int slave;
            Hazemeyer::Corrector  *Hardware;
            bool initDone;
            float CurrentSP;
            float maxCurrent,minCurrent;
            float HwMaxCurrent,HwMinCurrent;
           
            static std::map<std::string,ChannelPhysicalMap,CompareStdStr> mainUnitTable;
            
            void printStaticTableContent();
    
        };
        
    }

}
#endif	/* AL250POWERSUPPLY_H */

