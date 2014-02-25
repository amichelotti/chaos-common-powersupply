//
//  OcemE642X.h
//  PowerSupply
//
//  Created by andrea michelotti on 10/11/13.
//  Copyright (c) 2013 andrea michelotti. All rights reserved.
//

#ifndef __PowerSupply__OcemE642X__
#define __PowerSupply__OcemE642X__


#include <iostream>
#include "common/powersupply/core/AbstractPowerSupply.h"
#include "common/serial/Ocem/OcemProtocol.h"
#include <string.h>

#ifndef OCEM_SELECT_TIMEOUT
#define OCEM_SELECT_TIMEOUT 1000
#endif

#ifndef OCEM_POLL_TIMEOUT
#define OCEM_POLL_TIMEOUT 1000
#endif

#ifndef OCEM_TERMINATOR
#define OCEM_TERMINATOR ""
#endif

#ifndef OCEM_INPUT_CHANNELS
#define OCEM_INPUT_CHANNELS 5
#endif

#ifndef OCEM_OUTPUT_CHANNELS
#define OCEM_OUTPUT_CHANNELS 5
#endif

#ifndef OCEM_ALARM_NUMBER
#define OCEM_ALARM_NUMBER 25
#endif

#ifndef OCEM_MAX_CURRENT
#define OCEM_MAX_CURRENT 100
#endif

#ifndef OCEM_MIN_CURRENT
#define OCEM_MIN_CURRENT 0
#endif

#ifndef OCEM_MAX_VOLTAGE
#define OCEM_MAX_VOLTAGE 10
#endif

#ifndef OCEM_MIN_VOLTAGE
#define OCEM_MIN_VOLTAGE 0
#endif

#ifndef OCEM_VOLTAGE_ADC
#define OCEM_VOLTAGE_ADC 8
#endif

#ifndef OCEM_CURRENT_ADC
#define OCEM_CURRENT_ADC 16
#endif

#ifndef OCEM_CURRENT_RAMP_ADC
#define OCEM_CURRENT_RAMP_ADC 12
#endif

#include <map>
#include <boost/shared_ptr.hpp>



namespace common{
    namespace powersupply {
        
        typedef boost::shared_ptr<common::serial::OcemProtocol> OcemProtocol_psh;
        
        
        
        class OcemE642X: public AbstractPowerSupply {
            
            static std::map<std::string,OcemProtocol_psh > unique_protocol;
            
            static pthread_mutex_t unique_ocem_core_mutex;
            
        protected:
            OcemProtocol_psh ocem_prot;
            
            /* connection parameters */
	    std::string dev;
            int baudrate;
            int parity;
            int bits;
            int stop;
            /*********/
            int slave_id;
            /*********/
            /* state */
            /** internal structure */
            struct ocem_channel {
                int min;
                int max;
                ocem_channel(int _min,int _max):min(_min),max(_max){}
                ocem_channel(){min = 0; max=0;}
            };
            
            struct ocem_version {
                char rilascio;
                int anno;
                int mese;
                int giorno;
                char type[8];
                ocem_version(char _r,int a,int m,int g,char*typ):rilascio(_r),anno(a),giorno(g){strncpy(type,typ,8);}
                ocem_version(){rilascio='x';anno=0;mese = 0;giorno = 0;*type=0;}
            };
            
            enum Polarity {
                POL_UKN,
                POL_POS,
                POL_NEG,
                POL_ZERO,
                POL_RUN
            };
            enum RegulatorState{
                REGULATOR_UKN,
                REGULATOR_ON,
                REGULATOR_STANDBY,
                REGULATOR_SHOULD_BE_OFF,
                REGULATOR_ERROR
            };
            
            enum SelectorState{
                SELECTOR_UKN,
                SELECTOR_PRE,
                SELECTOR_LOC,
                SELECTOR_ERROR
            };
            
            enum ChannelType {
                CHANNEL_IN=0,
                CHANNEL_OUT
            };
            
            enum ChannelTypeIn {
                CHANNEL_CURR_VAL=0,
                CHANNEL_VOLT_VAL=1
            };
            
            enum ChannelTypeOut {
                CHANNEL_CURR_SP=0,
                CHANNEL_RAMP_UP=1,
                CHANNEL_RAMP_DOWN=1,
            };
	    enum OcemModels {
	      OCEM_NOMODEL,
	      OCEM_MODEL234,
	      OCEM_MODEL5A5B
	    };

            common::debug::timed_value<ocem_channel> ichannel[OCEM_INPUT_CHANNELS];
            common::debug::timed_value<ocem_channel> ochannel[OCEM_OUTPUT_CHANNELS];
            common::debug::timed_value<unsigned> current;
            common::debug::timed_value<unsigned> voltage;
            common::debug::timed_value<Polarity> polarity;
            common::debug::timed_value<uint64_t> alarms;
            common::debug::timed_value<RegulatorState> regulator_state;
            common::debug::timed_value<SelectorState> selector_state;
            common::debug::timed_value<ocem_version> version;
            
            common::debug::timed_value<unsigned> sp_current;
            
            // performs polls for a given time
            // returns numbero of items updated
            int update_status(uint32_t timeout,int msxpoll=100);
            // update the internal data from stringa
            // return the number of internal data updated
            int updateInternalData(char * stringa);
            
            int force_update(uint32_t timeout);
            
            
            /**
             update the status of the template
             @return the number of data updated, 0 or negative if nothing has been updated
             */
            int update_status(common::debug::basic_timed*data ,char *cmd,uint32_t timeout);
            
            static OcemProtocol_psh getOcemProtocol(std::string &dev,int baudrate=9600,int parity=0,int bits=8,int stop=1);
            static void removeOcemProtocol(std::string &dev);
            float max_current;
            float min_current;
            
            float max_voltage;
            float min_voltage;
            
            static const int voltage_adc=OCEM_VOLTAGE_ADC;
            static const int current_adc=OCEM_CURRENT_ADC;
            static const int current_ramp_adc=OCEM_CURRENT_RAMP_ADC;
            float voltage_sensibility;
            float current_sensibility;
	    uint64_t available_alarms;
	    OcemModels model;
	    void updateParamsByModel(OcemModels model);
        public:

            OcemE642X(const char *dev,int slave_id,int baudrate=9600,int parity=0,int bits=8,int stop=1);
            ~OcemE642X();
            
            
            
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
             \brief shuts down the powerSupply, the communication drops
             @return 0 if success
             */
            virtual int shutdown(uint32_t timeo_ms=0);
            virtual int poweron(uint32_t timeo_ms=0);
            virtual int standby(uint32_t timeo_ms=0);
            virtual int getState(int* state,std::string&,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT);
            
            virtual int init();
            virtual int deinit();
            
            
            /** OCEM E642 commands ***/
            
            /**
             @brief send a command (perform a select), with a given timeout
             @param cmd a zero terminated string containing the command
             @param timeout the time in ms waiting for the completion of the command
             @return the number of characters successfully transmitted, negative if error
             */
            
            int send_command(char*cmd,uint32_t timeout,int *timeo);
            // return the number of sucessfully data read or an error
            /**
             @brief send a command (perform a select) and waits for an answer (performs a poll)
             @param cmd a zero terminated string containing the command
             @param buf the output buffer that receive the result of the command
             @param size max size of the output buffer
             @param timeos timeout in ms waiting for the completion of the command
             @param timeop the timeout in ms waiting for the answer to the command
             @return the number of data received or negative if error
             */
            int send_receive(char*cmd,char*buf,int size,uint32_t timeos,uint32_t timeop,int *timeo);
            
            /**
             @brief acquire data from (performs poll) for a given number of ms (timeout)
             @param buf the output buffer that receive the result of the command
             @param size max size of the output buffer
             @param timeo the time in ms performing poll of data
             @return the number of data received or negative if error
             */
            
            int receive_data(char*buf, int size,uint32_t timeout,int *timeo);
            
            /**
             @brief set the given OCEM(input/output) channel
             @param inout select the input(=0) or output (=1) channel
             @param number the channel number
             @param min min value
             @param max max value
             @return 0 if success 
             */
            int setChannel(int inout,int number, int min,int max,uint32_t timeo);
            
            /**
             @brief get the given OCEM(input/output) channel
             @param inout select the input(=0) or output (=1) channel
             @param number the channel number
             @param min output min value
             @param max output max value
             @return 0 if success 
             */
            int getChannel(int inout,int number, int* min,int* max,uint32_t timeo_ms);
            
            
            /**
             @brief set the OCEM channel threashold 
             @param number (0=current, 1=voltage) the channel number
             @param value value
             @return 0 if success 
             */
            int setThreashold(int channel,float value,uint32_t timeo_ms);
            int setThreashold(int channel,int value,uint32_t timeo_ms);
            
            
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

            
            
        };
    };
};
#endif /* defined(__PowerSupply__OcemE642X__) */
