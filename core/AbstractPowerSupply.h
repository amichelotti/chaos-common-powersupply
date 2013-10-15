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
namespace common {
  namespace powersupply {
    // put your code here
      class AbstractPowerSupply {
      
      
      public:
          /**
            sets the current polarity
            @param pol if >0 sets positive current polarity, if <0 sets negative current polarity, =0 opens the circuit, no current
           @return 0 if success or an error code
           */
          virtual int setCurrentPolarity(int pol)=0;
      
          /**
           gets the current polarity
           @param pol returns the polarity if >0 positive current polarity, if <0 negative current polarity, =0 circuit is open, no current
           @return 0 if success or an error code
           */
          virtual int getCurrentPolarity(int* pol)=0;
          /**
           sets the current set point
           @param current the current set point to reach expressed in ampere
           @return 0 if success or an error code
           */
          
          virtual int setCurrentPoint(float current)=0;
         
          
          /**
           gets the actual current set point
           @param current returns the current readout expressed in ampere
           @return 0 if success or an error code
           */
          
          virtual int getCurrentPoint(float* current)=0;

          /**
           start ramp toward the predefined current set point
           @return 0 if success (current readout equal to set) or an error code
           */
          
          virtual int startCurrentRamp()=0;
          
          /**
           sets the voltage set point
           @param voltage the voltage set point to reach expressed in volt
           @return 0 if success or an error code
           */
          
          virtual int setVoltagePoint(float volt)=0;
          
          
          /**
           gets the actual voltage set point
           @param voltage returns the voltage readout expressed in volt
           @return 0 if success or an error code
           */
          
          virtual int getVoltagePoint(float* current)=0;
          
          /**
           start ramp toward the predefined voltage set point
           @return 0 if success or an error code
           */
          
          virtual int startVoltageRamp()=0;
          
          /**
           set the current ramp speed
           @param as ramp speed expressed in ampere/second
           @return 0 if success or an error code
           */
          virtual int setCurrentRampSpeed(float as)=0;
          
          /**
           set the voltage ramp speed
           @param vs ramp speed expressed in volt/second
           @return 0 if success or an error code
           */
          virtual int setVoltageRampSpeed(float vs)=0;
          

          /**
           set the maximum/minimum current threashold
           @param maxcurr max current threashold expressed in ampere
           @param mincurr min current threashold expressed in ampere
           @return 0 if success or an error code
           */
          virtual int setCurrentThreashold(float maxcurr,float mincurr=0)=0;

     
          /**
           set the maximum/minimum voltage threashold
           @param maxvolt max voltage threashold epressed in volts
           @param minvolt min voltage threashold epressed in volts
           @return 0 if success or an error code
           */
          virtual int setVoltageThreashold(float maxvolt,float minvolt)=0;
          
       
          /**
           get the maximum/minimum current threashold
           @param maxcurr outputs max current threashold expressed in ampere
           @param mincurr outputs min current threashold expressed in ampere
           @return 0 if success or an error code
           */
          virtual int getCurrentThreashold(float *maxcurr,float *mincurr=0)=0;
          
          
          /**
           get the maximum/minimum voltage threashold
           @param maxvolt outputs max voltage threashold epressed in volts
           @param minvolt outputs min voltage threashold epressed in volts
           @return 0 if success or an error code
           */
          virtual int getVoltageThreashold(float *maxvolt,float *minvolt=0)=0;
          
          /**
           resets alarms
           @param alrm a 64 bit field containing the alarms to be reset (-1 all alarms)
           @return 0 if success
           */
          virtual int resetAlarms(uint64_t alrm)=0;
          
          /**
           get alarms
           @param alrm returns a 64 bit field containing the alarms
           @return 0 if success
           */
          virtual int getAlarms(uint64_t*alrm)=0;


          /**
           shuts down the powerSupply, the communication could drop
           @return 0 if success
           */
          virtual int shutdown()=0;
          
          
          /**
           standby the powerSupply
           @return 0 if success
           */
          virtual int standby()=0;
          
          /**
           initialize and power on the powerSupply
           @return 0 if success
           */
          virtual int init()=0;
          
          /**
           de-initialize the powerSupply
           @return 0 if success
           */
          virtual int deinit()=0;
         
          /**
           returns the SW/FW version of the driver/FW
           @return a non empty string containing the information
           */
          virtual std::string getSWVersion()=0;

          /**
           returns the HW version of the powersupply
           @return a non empty string containing the information
           */
          virtual std::string getHWVersion()=0;

      };
    

  };
};

#endif
