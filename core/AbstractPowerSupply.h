/*
 *	PowerSupply.h
 *	!CHAOS
 *	Created by automatically
 *
 *    	Copyright 2012 INFN, National Institute of Nuclear Physics
 *
 *    	Licensed under the Apache License, Version 2.0 (the "License");
 *    	you may not use this file except in compliance with the License.
 *    	You may obtain a copy of the License at
 *
 *    	http://www.apache.org/licenses/LICENSE-2.0
 *
 *    	Unless required by applicable law or agreed to in writing, software
 *    	distributed under the License is distributed on an "AS IS" BASIS,
 *    	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    	See the License for the specific language governing permissions and
 *    	limitations under the License.
 */
#ifndef __common_AbstractPowerSupply_h__
#define __common_AbstractPowerSupply_h__


// include your class/functions headers here
#include <stdint.h>
#include <string>

#ifndef POWER_SUPPLY_DEFAULT_TIMEOUT
#define POWER_SUPPLY_DEFAULT_TIMEOUT 1000
#endif



namespace common {
    namespace powersupply {
        
        
        typedef struct _calibdata {
            double sp;
            double readout;
            _calibdata(){sp=0;readout=0;}
        } calibdata_t;
        
        /**
         * 
         * @param calibration_data calibra
         * @param size
         * @param sp
         * @param readout_expected
         * @param error_expected
         * @return 
         */
        int fitData(calibdata_t *calibration_data,int size,double sp,double& readout_expected,double& error_expected);
        
        /**
         \enum PowerSupplyError
         \brief Power supply function error return codes
         */
        enum PowerSupplyError{
            POWER_SUPPLY_OUT_OF_SET=1, // the driver function detects out of set

            POWER_SUPPLY_BAD_INPUT_PARAMETERS=-300, /// parameters not correct
            POWER_SUPPLY_BAD_OUT_OF_RANGE_PARAMETERS, /// parameters out of range
            POWER_SUPPLY_TIMEOUT,   /// timeout during operation
            POWER_SUPPLY_NOT_READY, /// power supply cannot execute command
            POWER_SUPPLY_READOUT_INSTABLE, /// the readout is not stable
            POWER_SUPPLY_READOUT_OUTDATED, /// the readout value is older than the command
            POWER_SUPPLY_OFFLINE, /// the power supply communication is interrupted
            POWER_SUPPLY_EXCEPTION, /// an exception/error avoid the completion of the command
            POWER_SUPPLY_READBACK_FAILED, /// read and written value don't match
            POWER_SUPPLY_COMMAND_ERROR, /// an error occur during a command
            POWER_SUPPLY_RECEIVE_ERROR,/// an error occur during read of data
            POWER_SUPPLY_COMMAND_IN_BAD_STATE,
            POWER_SUPPLY_ERROR_SETTING_CHANNEL_SENSIBILITY            
        };
        
        /**
         Power supply supported events
         */
        enum PowerSupplyEvents{
            POWER_SUPPLY_EVENT_DOOR_OPEN=0x1,
            POWER_SUPPLY_EVENT_OVER_TEMP=0x2,
            POWER_SUPPLY_FUSE_FAULT=0x4,
            POWER_SUPPLY_EARTH_FAULT=0x8,
            POWER_SUPPLY_OVER_VOLTAGE=0x10,
            POWER_SUPPLY_OVER_CURRENT=0x20,
            POWER_SUPPLY_COMMUNICATION_FAILURE=0x40,
            POWER_SUPPLY_MAINUNIT_FAIL=0x80,   // only for multichannels powersupply
            POWER_SUPPLY_EXTERNAL_INTERLOCK=0x100,
            POWER_SUPPLY_SETPOINT_CARD_FAULT=0x200,
            POWER_SUPPLY_CUBICLE_OVT=0x400,
            POWER_SUPPLY_DCCT_OVT=0x800,
            POWER_SUPPLY_DCCT_FAULT=0x1000,
            POWER_SUPPLY_ACTIVE_FILTER_FUSE=0x2000,
            POWER_SUPPLY_ACTIVE_FILTER_OVT=0x4000,
            POWER_SUPPLY_DIODE_OVT=0x8000,
            POWER_SUPPLY_DIODE_FAULT=0x10000,
            POWER_SUPPLY_AC_UNBALANCE=0x20000,
            POWER_SUPPLY_PHASE_LOSS=0x40000,
            POWER_SUPPLY_AIR_FLOW=0x80000,
            POWER_SUPPLY_TRANSFORMER_OVT=0x100000,
            POWER_SUPPLY_SNUBBER_FUSE=0x200000,
            POWER_SUPPLY_SCR_FUSE=0x400000,
            POWER_SUPPLY_SCR_OVT=0x800000,
            POWER_SUPPLY_CHOKE_OVT=0x1000000,
            POWER_SUPPLY_PASS_FILTER=0x2000000,
            POWER_SUPPLY_ALARM_UNDEF=0x4000000
                                  
        };
        
        
        /**
         Power supply supported States
         */
        enum PowerSupplyStates{
            POWER_SUPPLY_STATE_OFF=0x1, /// the power suply is off, do no outputs corrent and it could not answer to protocol
            POWER_SUPPLY_STATE_ON=0x2, /// power supply is ON and ouputs current
            POWER_SUPPLY_STATE_ERROR=0x4, /// an error occured
            POWER_SUPPLY_STATE_STANDBY=0x8, /// the power supply is not supplying current but the it answer to the protocol
            POWER_SUPPLY_STATE_OPEN=0x10, /// the power supply is on but is not supplying current
            POWER_SUPPLY_STATE_ALARM=0x20, /// an event occurred
            POWER_SUPPLY_STATE_LOCAL=0x40, /// the powersupply in in local mode
            POWER_SUPPLY_STATE_MAINUNIT_NOT_ON=0x80, /// powersupply main unit not on
            POWER_SUPPLY_STATE_TRIGGER_ARMED=0x100, /// powersupply trigger armed

            POWER_SUPPLY_STATE_UKN /// it's not possible to know the state (no answer)
        };
        /**
         * */
         enum PowerSupplyFeature{
            POWER_SUPPLY_FEAT_MONOPOLAR=0x1, /// monopolar
            POWER_SUPPLY_FEAT_BIPOLAR=0x2, /// bipolar
            POWER_SUPPLY_FEAT_PULSED=0x4, /// pulsed
            POWER_SUPPLY_FEAT_UKN /// it's not possible to know the type
        };
        /**
         base class for all powersupplys
         */
        class AbstractPowerSupply {
            
            
        public:
            
            AbstractPowerSupply(){}
            static std::string decodeEvent(PowerSupplyEvents ev);
            virtual ~AbstractPowerSupply(){};
            /**
             @brief sets the current polarity
             @param pol if >0 sets positive current polarity, if <0 sets negative current polarity, =0 opens the circuit, no current
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success or an error code
             */
            virtual int setPolarity(int pol,uint32_t timeo_ms=0)=0;
            
            /**
             @brief gets the current polarity
             @param pol returns the polarity if >0 positive current polarity, if <0 negative current polarity, =0 circuit is open, no current
             @param polsp returns the polarity setpoint (if supported) if >0 positive current polarity, if <0 negative current polarity, =0 circuit is open, no current

             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success or an error code
             */
            virtual int getPolarity(int* pol,int*polsp=NULL, uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            /**
             @brief sets the current set point
             @param current the current set point to reach expressed in ampere
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success or an error code
             */
            
            virtual int setCurrentSP(float current,uint32_t timeo_ms=0)=0;
            
            
            /**
             @brief gets the actual current set point
             @param current returns the current readout expressed in ampere
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success or an error code
             */
            
            virtual int getCurrentSP(float* current,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            /**
             @brief start ramp toward the predefined current set point
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success (current readout equal to set) or an error code
             */
            
            virtual int startCurrentRamp(uint32_t timeo_ms=0)=0;
            
            /**
             @brief gets the voltage output
             @param volt gets the voltage readout expressed in volt
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success or an error code
             */
            
            virtual int getVoltageOutput(float* volt,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            
            /**
             @brief gets the current output
             @param current returns the current readout expressed in ampere
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success or an error code
             */
            
            virtual int getCurrentOutput(float* current,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            
            /**
             @brief set the current rising/falling ramp speed
             @param asup rising ramp speed expressed in ampere/second
             @param asdown rising ramp speed expressed in ampere/second
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success or an error code
             */
            virtual int setCurrentRampSpeed(float asup,float asdown,uint32_t timeo_ms=0)=0;
            
            
            /**
             @brief resets alarms
             @param alrm a 64 bit field containing the alarms to be reset (-1 all alarms)
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success
             */
            virtual int resetAlarms(uint64_t alrm,uint32_t timeo_ms=0)=0;
            
            /**
             @brief get alarms
             @param alrm returns a 64 bit field PowerSupplyEvents containing the alarms
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success
             */
            virtual int getAlarms(uint64_t*alrm,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            
            /**
             @brief shuts down the power supply, the communication could drop
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success
             */
            virtual int shutdown(uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            
            /**
             @brief standby the power supply
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success
             */
            virtual int standby(uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            /**
             @brief poweron the power supply after a standby or shutdown (if possible)
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success
             */
            virtual int poweron(uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            
            /**
             @brief gets the power supply state
             @param state returns a bit field of PowerSupplyStates
             @param statesp returns a bit field of PowerSupplyStates Setpoint

             @param desc return a string description
             @param timeo_ms timeout in ms for the completion of the operation (0 wait indefinitively)
             @return 0 if success or an error code
             */
            virtual int getState(int* state,std::string& desc,int*statesp=NULL,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            
            /**
             @brief initialize and poweron the power supply
             @return 0 if success
             */
            virtual int initPS()=0;
            
            /**
             @brief de-initialize the power supply and close the communication
             @return 0 if success
             */
            virtual int deinitPS()=0;
            
            /**
             @brief returns the SW/FW version of the driver/FW
             @param version returning string
             @return 0 if success or an error code
             */
            virtual int getSWVersion(std::string& version,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            /**
             @brief returns the HW version of the powersupply
             @param version returning string
             @return 0 if success or an error code
             
             */
            virtual int getHWVersion(std::string& version,uint32_t timeo_ms=POWER_SUPPLY_DEFAULT_TIMEOUT)=0;
            
            /**
             @brief returns the current sensibility of the power supply
             @param sens sensibility in ampere
             @return 0 if success or an error code
             
             */
            virtual int getCurrentSensibility(float *sens)=0;
            /**
             @brief returns the voltage sensibility of the power supply
             @param sens sensibility in volt
             @return 0 if success or an error code
             */
            virtual int getVoltageSensibility(float *sens)=0;
            
            
             /**
             @brief set the current sensibility of the power supply
             @param sens sensibility in ampere
             @return 0 if success or an error code
             
             */
            virtual int setCurrentSensibility(float sens);
            
            /**
             @brief set the voltage sensibility of the power supply
             @param sens sensibility in volt
             @return 0 if success or an error code
             */
            virtual int setVoltageSensibility(float sens);
            /**
             @brief returns the max min current of the power suppy
             @param max returns the max current that the power supply can output
             @param min returns the min current that the power supply can output
             @return 0 if success or an error code
             
             */
            virtual int getMaxMinCurrent(float*max,float*min)=0;
            
            /**
             @brief returns the max min voltage of the power suppy
             @param max returns the max voltage that the power supply can output
             @param min returns the min voltage that the power supply can output
             @return 0 if success or an error code
             
             */
            virtual int getMaxMinVoltage(float*max,float*min)=0;
            /**
             @brief returns the bitfield of implemented alarms
             @param alarm 64bit bitfield containing the implemented alarms
             @return 0 if success or an error code
             
             */
            virtual int getAlarmDesc(uint64_t* alarm)=0;
            
            /**
             @brief force max current, it is used to calculated max current of the powersupply
             @param max the max current that the power supply can output
             @return 0 if success or an error code
             */
            virtual int forceMaxCurrent(float max)=0;
            
            /**
             @brief force max voltage, it is used to calculated max current of the powersupply
             @param max the max voltage that the power supply can output
             @return 0 if success or an error code
             */
            virtual int forceMaxVoltage(float max)=0;
            virtual uint64_t getFeatures() ;
            
        };
        
        
    };
};

#endif
