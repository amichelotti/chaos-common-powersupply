//
//  OcemCore.h
//  PowerSupply
//
//  Created by andrea michelotti on 10/11/13.
//  Copyright (c) 2013 andrea michelotti. All rights reserved.
//

#ifndef __PowerSupply__OcemCore__
#define __PowerSupply__OcemCore__

#include <iostream>
#include "common/powersupply/core/AbstractPowerSupply.h"
#include "common/serial/Ocem/OcemProtocol.h"
#ifndef OCEM_SELECT_TIMEOUT
#define OCEM_SELECT_TIMEOUT 1000
#endif

#ifndef OCEM_POLL_TIMEOUT
#define OCEM_POLL_TIMEOUT 1000
#endif

#ifndef OCEM_TERMINATOR
#define OCEM_TERMINATOR "\r\n"
#endif

#include <map>
#include <boost/shared_ptr.hpp>



namespace common{
        namespace powersupply {
            
            typedef boost::shared_ptr<common::serial::OcemProtocol> OcemProtocol_psh;

            class OcemCore: public AbstractPowerSupply {
            
                static std::map<std::string,OcemProtocol_psh > unique_protocol;
                
                static pthread_mutex_t unique_ocem_core_mutex;
                
            protected:
                OcemProtocol_psh ocem_prot;

                /* connection parameters */
                const char *dev;
                int baudrate;
                int parity;
                int bits;
                int stop;
                /*********/
                int slave_id;
                /*********/
                /* state */
                
                enum PowerSupplyState{
                    UNKNOWN,
                    OFF,
                    ON,
                    STANDBY
                } state;
                
                
                // return the number of characters sucessfully written or an error
                int send_command(char*cmd,int timeout=OCEM_SELECT_TIMEOUT);
                // return the number of characters sucessfully read or an error
                int send_receive(char*cmd,char*buf,int size,int timeos=OCEM_SELECT_TIMEOUT,int timeop=OCEM_POLL_TIMEOUT);
                // return the number of characters sucessfully read or an error
                int check_data(char*buf, int size,int timeout=OCEM_POLL_TIMEOUT);
                
            public:
                OcemCore(const char *dev,int slave_id,int baudrate=9600,int parity=0,int bits=8,int stop=1);
                ~OcemCore();
                
                static OcemProtocol_psh getOcemProtocol(const char *dev,int baudrate=9600,int parity=0,int bits=8,int stop=1);
                static void removeOcemProtocol(const char*dev);
                /**
                 sets the current polarity
                 @param pol if >0 sets positive current polarity, if <0 sets negative current polarity, =0 opens the circuit, no current
                 @return 0 if success or an error code
                 */
                virtual int setCurrentPolarity(int pol);
                
                /**
                 gets the current polarity
                 @param pol returns the polarity if >0 positive current polarity, if <0 negative current polarity, =0 circuit is open, no current
                
                 
                 
                 @return 0 if success or an error code
                 */
                virtual int getCurrentPolarity(int* pol);
                /**
                 sets the current set point
                 @param current the current set point to reach expressed in ampere
                 @return 0 if success or an error code
                 */
                
                virtual int setCurrentPoint(float current);
                
                
                /**
                 gets the actual current set point
                 @param current returns the current readout expressed in ampere
                 @return 0 if success or an error code
                 */
                
                virtual int getCurrentPoint(float* current);
                
                /**
                 start ramp toward the predefined current set point
                 @return 0 if success (current readout equal to set) or an error code
                 */
                
                virtual int startCurrentRamp();
                
                /**
                 sets the voltage set point
                 @param voltage the voltage set point to reach expressed in volt
                 @return 0 if success or an error code
                 */
                
                virtual int setVoltagePoint(float volt);
                
                
                /**
                 gets the actual voltage set point
                 @param voltage returns the voltage readout expressed in volt
                 @return 0 if success or an error code
                 */
                
                virtual int getVoltagePoint(float* current);
                
                /**
                 start ramp toward the predefined voltage set point
                 @return 0 if success or an error code
                 */
                
                virtual int startVoltageRamp();
                
                /**
                 set the current ramp speed
                 @param as ramp speed expressed in ampere/second
                 @return 0 if success or an error code
                 */
                virtual int setCurrentRampSpeed(float as);
                
                /**
                 set the voltage ramp speed
                 @param vs ramp speed expressed in volt/second
                 @return 0 if success or an error code
                 */
                virtual int setVoltageRampSpeed(float vs);
                
                
                /**
                 set the maximum/minimum current threashold
                 @param maxcurr max current threashold expressed in ampere
                 @param mincurr min current threashold expressed in ampere
                 @return 0 if success or an error code
                 */
                virtual int setCurrentThreashold(float maxcurr,float mincurr=0);
                
                
                /**
                 set the maximum/minimum voltage threashold
                 @param maxvolt max voltage threashold epressed in volts
                 @param minvolt min voltage threashold epressed in volts
                 @return 0 if success or an error code
                 */
                virtual int setVoltageThreashold(float maxvolt,float minvolt);
                
                
                /**
                 get the maximum/minimum current threashold
                 @param maxcurr outputs max current threashold expressed in ampere
                 @param mincurr outputs min current threashold expressed in ampere
                 @return 0 if success or an error code
                 */
                virtual int getCurrentThreashold(float *maxcurr,float *mincurr=0);
                
                
                /**
                 get the maximum/minimum voltage threashold
                 @param maxvolt outputs max voltage threashold epressed in volts
                 @param minvolt outputs min voltage threashold epressed in volts
                 @return 0 if success or an error code
                 */
                virtual int getVoltageThreashold(float *maxvolt,float *minvolt=0);
                
                /**
                 resets alarms
                 @param alrm a 64 bit field containing the alarms to be reset (-1 all alarms)
                 @return 0 if success
                 */
                virtual int resetAlarms(uint64_t alrm);
                
                /**
                 get alarms
                 @param alrm returns a 64 bit field containing the alarms
                 @return 0 if success
                 */
                virtual int getAlarms(uint64_t*alrm);
                
                
                /**
                 shuts down the powerSupply, the communication could drop
                 @return 0 if success
                 */
                virtual int shutdown();
                
                
                /**
                 standby the powerSupply
                 @return 0 if success
                 */
                virtual int standby();
                
                /**
                 initialize and power on the powerSupply
                 @return 0 if success
                 */
                virtual int init();
                
                /**
                 de-initialize the powerSupply
                 @return 0 if success
                 */
                virtual int deinit();
                
                virtual std::string getSWVersion();
                
                virtual std::string getHWVersion();

                
            };
        };
};
#endif /* defined(__PowerSupply__OcemCore__) */
