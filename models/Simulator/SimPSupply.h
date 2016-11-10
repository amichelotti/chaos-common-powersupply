//
//  SimPSupply.h
//  PowerSupply
//
//  Created by andrea michelotti on 10/11/13.
//  Copyright (c) 2013 andrea michelotti. All rights reserved.
//

#ifndef __PowerSupply__SimPSupply__
#define __PowerSupply__SimPSupply__


#include <iostream>
#include <common/powersupply/core/AbstractPowerSupply.h>
#include <string.h>
#include <stdint.h>

#ifndef SIMPSUPPLY_SELECT_TIMEOUT
#define SIMPSUPPLY_SELECT_TIMEOUT 1000
#endif

#ifndef SIMPSUPPLY_POLL_TIMEOUT
#define SIMPSUPPLY_POLL_TIMEOUT 1000
#endif

#ifndef SIMPSUPPLY_TERMINATOR
#define SIMPSUPPLY_TERMINATOR ""
#endif

#ifndef SIMPSUPPLY_INPUT_CHANNELS
#define SIMPSUPPLY_INPUT_CHANNELS 5
#endif

#ifndef SIMPSUPPLY_OUTPUT_CHANNELS
#define SIMPSUPPLY_OUTPUT_CHANNELS 5
#endif

#ifndef SIMPSUPPLY_ALARM_NUMBER
#define SIMPSUPPLY_ALARM_NUMBER 25
#endif

#ifndef SIMPSUPPLY_MAX_CURRENT
#define SIMPSUPPLY_MAX_CURRENT 100
#endif

#ifndef SIMPSUPPLY_MIN_CURRENT
#define SIMPSUPPLY_MIN_CURRENT 0
#endif

#ifndef SIMPSUPPLY_MAX_VOLTAGE
#define SIMPSUPPLY_MAX_VOLTAGE 10
#endif

#ifndef SIMPSUPPLY_MIN_VOLTAGE
#define SIMPSUPPLY_MIN_VOLTAGE 0
#endif

#ifndef SIMPSUPPLY_VOLTAGE_ADC
#define SIMPSUPPLY_VOLTAGE_ADC 8
#endif

#ifndef SIMPSUPPLY_CURRENT_ADC
#define SIMPSUPPLY_CURRENT_ADC 16
#endif

#ifndef SIMPSUPPLY_CURRENT_RAMP_ADC
#define SIMPSUPPLY_CURRENT_RAMP_ADC 12
#endif

#ifndef SIMPSUPPLY_WLAT
#define SIMPSUPPLY_WLAT 100000 // 100 ms
#endif

#ifndef SIMPSUPPLY_RLAT
#define SIMPSUPPLY_RLAT 300000 // 300 ms
#endif

#ifndef SIMPSUPPLY_UPDATE_DELAY
#define SIMPSUPPLY_UPDATE_DELAY 100000 // 100 ms
#endif

#include <map>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>


namespace common{
    namespace powersupply {
               
        
        
        class SimPSupply: public AbstractPowerSupply {
            
            
            
        protected:
            enum RegulatorState{
                REGULATOR_UKN,
                REGULATOR_ON,
                REGULATOR_STANDBY,
                REGULATOR_SHOULD_BE_OFF,
                REGULATOR_ERROR
            };

            /* connection parameters */
            std::string dev;
            std::string state_name;
            int slave_id;
            float max_current;
            float min_current;
            
            float max_voltage;
            float min_voltage;
            
            float ramp_speed_up,ramp_speed_down;
            int voltage_adc;
            int current_adc;
            int current_ramp_adc;
            float voltage_sensibility;
            float current_sensibility;
            
            int write_latency_min;
            int write_latency_max;
            int read_latency_min;
            int read_latency_max;
            boost::thread m_thread;
            uint64_t available_alarms;
            void run();
            /// actual values
            int polarity;
            uint32_t voltage;
            uint32_t current;
            int currSP;
            int regulator_state;
            int selector_state;

            int start_ramp;
            unsigned force_errors;
            uint64_t alarms;
            bool running;
            void update_state();
            int wait_write();
            int wait_read();
            unsigned update_delay;
            uint64_t feats;
        public:

            SimPSupply(const char *dev,int slave_id,uint64_t _feats,float min_current=0,float max_current=SIMPSUPPLY_MAX_CURRENT,float min_voltage=0,float max_voltage=SIMPSUPPLY_MAX_VOLTAGE, int write_latency_min=SIMPSUPPLY_WLAT,int write_latency_max=SIMPSUPPLY_WLAT,int read_latency_min=SIMPSUPPLY_RLAT,int read_latency_max=SIMPSUPPLY_RLAT,int current_adc=SIMPSUPPLY_CURRENT_ADC,int voltage_adc=SIMPSUPPLY_VOLTAGE_ADC,unsigned update_delay=SIMPSUPPLY_UPDATE_DELAY,unsigned force_errors=0);
            ~SimPSupply();
            
            
            
            virtual int setPolarity(int pol,uint32_t timeo_ms=0);
            virtual int getPolarity(int* pol,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            
            virtual int setCurrentSP(float current,uint32_t timeo_ms=0);
            virtual int getCurrentSP(float* current,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            
            virtual int startCurrentRamp(uint32_t timeo_ms=0);
            
            virtual int getCurrentOutput(float* curr,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            virtual int getVoltageOutput(float* volt,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            
            
            
            virtual int setCurrentRampSpeed(float asup,float asdown,uint32_t timeo_ms=0);
            
            virtual int resetAlarms(uint64_t alrm,uint32_t timeo_ms=0);
            virtual int getAlarms(uint64_t*alrm,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            virtual int getAlarmDesc(uint64_t*desc);

            virtual int getCurrentSensibility(float*sens);
            virtual int getVoltageSensibility(float*sens);
            
            virtual int getMaxMinCurrent(float*max,float*min);
            virtual int getMaxMinVoltage(float*max,float*min);
            virtual int getSWVersion(std::string&version,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            virtual int getHWVersion(std::string&version,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            
            /**
             @brief force max current, it is used to calculated max current of the powersupply
             @param max the max current that the power supply can output
             @return 0 if success or an error code
             */
            int forceMaxCurrent(float max);
            
            /**
             @brief force max voltage, it is used to calculated max current of the powersupply
             @param max the max voltage that the power supply can output
             @return 0 if success or an error code
             */
            int forceMaxVoltage(float max);
            

            
            /**
             \brief shuts down the powerSupply, the communication drops
             @return 0 if success
             */
            virtual int shutdown(uint32_t timeo_ms=0);
            virtual int poweron(uint32_t timeo_ms=0);
            virtual int standby(uint32_t timeo_ms=0);
            virtual int getState(int* state,std::string&,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            
            virtual int init();
            virtual int deinit();
            virtual uint64_t getFeatures() ;
            
            
            
        };
    };
};
#endif /* defined(__PowerSupply__SimPSupply__) */
