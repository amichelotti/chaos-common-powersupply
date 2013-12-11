//
//  SimPSupply.cpp
//  PowerSupply
//
//  Created by andrea michelotti on 10/11/13.
//  Copyright (c) 2013 andrea michelotti. All rights reserved.
//
#ifdef SIMPSUPPLY_DEBUG
#define DEBUG
#endif
#include "common/debug/debug.h"
#include <math.h>

#include "SimPSupply.h"
#include "common/debug/debug.h"
#include <boost/regex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <sstream>
using namespace common::powersupply;

#define MIN(x,y) ((x<y)?x:y)
SimPSupply::SimPSupply(const char *_dev,int _slave_id,int _write_latency_min,int _write_latency_max,int _read_latency_min,int _read_latency_max,float _max_current,float _max_voltage,int _current_adc,int _voltage_adc,unsigned _update_delay,unsigned _force_errors):dev(_dev),slave_id(_slave_id),write_latency_min(_write_latency_min),write_latency_max(_write_latency_max),read_latency_min(_read_latency_min),read_latency_max(_read_latency_max),max_current(_max_current),max_voltage(_max_voltage),current_adc(_current_adc),voltage_adc(_voltage_adc),update_delay(_update_delay),force_errors(_force_errors)
{
    min_voltage=0;
    min_current=0;
    assert((voltage_adc>0) && (current_adc>0) && (max_voltage>0) && (max_current>0));
    voltage_sensibility=((max_voltage -min_voltage)*1.0)/(1<<voltage_adc);
    current_sensibility = ((max_current-min_current)*1.0)/(1<<current_adc);
    available_alarms =0;
    
    voltage = 0;
    current =0;
    polarity =0;
    regulator_state = REGULATOR_UKN;
    alarms = 0;
    currSP=0;
    start_ramp=0;
    boost::regex rp("\\/");
    std::stringstream sm;
    
    std::string rep=boost::regex_replace(dev, rp,std::string("_"));
    sm<<"SimPSupplyState_" << rep << "_"<<slave_id<<".txt";
    state_name = sm.str();
    ramp_speed_up=5/voltage_sensibility;
    ramp_speed_down=5/current_sensibility;
    

}

SimPSupply::~SimPSupply(){
    
    deinit();
}

void SimPSupply::update_state(){
    FILE *f;
    f= fopen(state_name.c_str(),"w");
    if(f){
        // get last state from file
        fprintf(f,"voltage:%d current:%d currS:%d pol:%d state:%d alarms:%llu\n",voltage,current,currSP,polarity,regulator_state,alarms);
        fclose(f);
    }

}
void SimPSupply::run(){
    DPRINT("Starting SimSupply service\n");
    while(running){
        if(start_ramp&& (regulator_state == REGULATOR_ON)){
            if(currSP>current){
                DPRINT("Ramp up adccurr %d (%f) set point %f increments %f\n",current,current*current_sensibility,currSP*current_sensibility,ramp_speed_up*current_sensibility*update_delay/1000000);

                current+=MIN(currSP-current,ramp_speed_up*((float)update_delay/1000000.0));
                
            } else if(currSP<current){
                DPRINT("Ramp down adccurr %d (%f) set point %f increments %f\n",current,current*current_sensibility,currSP*current_sensibility,ramp_speed_down*current_sensibility*update_delay/1000000);

                current-=MIN(current-currSP,ramp_speed_down*((float)update_delay/1000000.0));
            } else {
                start_ramp=false;
            }
        }
	update_state();
        usleep(update_delay);
    }
    DPRINT("Closing SimSupply service\n");

}
int SimPSupply::init(){
    char buf[2048];
    FILE *f;
    *buf = 0;
    f= fopen(state_name.c_str(),"r");
    regulator_state = REGULATOR_STANDBY;

    if(f){
        DPRINT("writing state\n");
        // get last state from file
        fscanf(f,"voltage:%d current:%d currS:%d pol:%d state:%d alarms:%llu",&voltage,&current,&currSP,&polarity,&regulator_state,&alarms);
        fclose(f);
    }
    running = true;
    start_ramp=0;
    m_thread = boost::thread(&SimPSupply::run,this);
    m_thread.detach();
    return 0;
}

int SimPSupply::deinit(){
    
    running = false;
    m_thread.join();
    update_state();
    return 0;
}

int SimPSupply::wait_write(){
    if((write_latency_max<=0)|| (write_latency_max<write_latency_min)) return 0;
    if(write_latency_max==write_latency_min){
        usleep(write_latency_max);
    }
    int wait = (rand()*1.0/RAND_MAX)*write_latency_max + write_latency_min;
    usleep(wait);
    return wait;
}

int SimPSupply::wait_read(){
    if((read_latency_max<=0)|| (read_latency_max<read_latency_min)) return 0;
    if(read_latency_max==read_latency_min){
        usleep(read_latency_max);
    }
    int wait = (rand()*1.0/RAND_MAX)*read_latency_max + read_latency_min;
    usleep(wait);
    return wait;

}

int SimPSupply::getSWVersion(std::string &ver,uint32_t timeo_ms){
    char stringa[1024];
    sprintf(stringa,"Driver:SimPSupply max current:%f max voltage:%f curr_adc:%d volt_adc:%d current sensibility:%f\n",max_current,max_voltage,current_adc,voltage_adc,current_sensibility);
    
    ver = stringa;
    if((wait_read()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    
    return 0;
}

int SimPSupply::getHWVersion(std::string&version,uint32_t timeo_ms){
    return getSWVersion(version,timeo_ms);
}


int SimPSupply::setPolarity(int pol,uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;
    if((wait_write()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    if(regulator_state!=REGULATOR_STANDBY){
        DERR("change of polarity not in STANDBY!!!!\n");
        
    }
    polarity=pol;
    return 0;
}

int SimPSupply::getPolarity(int* pol,uint32_t timeo_ms){
  boost::mutex::scoped_lock lock;

  if((wait_read()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    *pol = polarity;
  return 0;
}

int SimPSupply::setCurrentSP(float current,uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;

    if(current<min_current || current>max_current)
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    if((wait_write()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    
    currSP = current/current_sensibility;
    
    return 0;
}


int SimPSupply::getCurrentSP(float* current,uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;

    if((wait_read()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    *current = currSP*current_sensibility;
    return 0;
}

int  SimPSupply::getCurrentOutput(float* curr,uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;

    if((wait_read()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
  
    if(polarity==0){
        *curr = 0;
    } else {
        *curr = current*current_sensibility;
    }
    return 0;
}
int  SimPSupply::getVoltageOutput(float* volt,uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;

    if((wait_read()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    
    *volt = voltage*current_sensibility;
    return 0;
}

int SimPSupply::startCurrentRamp(uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;

    uint32_t simdel=wait_write();
    
    if((simdel>(timeo_ms*1000))&&(timeo_ms>0)){
        DERR("timeout spent %u us max was %u us\n",simdel,timeo_ms);
        return POWER_SUPPLY_TIMEOUT;
    }
    
    start_ramp=1;

    return 0;
}



int SimPSupply::setCurrentRampSpeed(float asup,float asdown,uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;
    
    if(asup<min_current || asup > max_current){
      DERR("bad input parameters asup %f\n",asup);
      return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    if(asdown<min_current || asdown > max_current){
      DERR("bad input parameters asdown %f\n",asdown);
      return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    if((wait_write()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    
    ramp_speed_up = asup/current_sensibility;
    ramp_speed_down = asdown/current_sensibility;
    
    return 0;
}



int SimPSupply::resetAlarms(uint64_t alrm,uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;

    if((wait_write()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    alarms = 0;
    return 0;
}

int SimPSupply::getAlarms(uint64_t*alrm,uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;

    if((wait_write()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    
    *alrm = alarms;
    return 0;
}


int SimPSupply::shutdown(uint32_t timeo_ms){
   
    // because we lose control once done this command
    DPRINT("SHUTDOWN command, we are going to lose control!!\n");
    if((wait_write()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    regulator_state = REGULATOR_SHOULD_BE_OFF;
    return 0;
}
int SimPSupply::poweron(uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;

    if((wait_write()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    regulator_state= REGULATOR_ON;
    return 0;
}

int SimPSupply::standby(uint32_t timeo_ms){
    boost::mutex::scoped_lock lock;

    if((wait_write()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    regulator_state= REGULATOR_STANDBY;

    return 0;
}


int  SimPSupply::getState(int* state,std::string &desc,uint32_t timeo_ms){
     boost::mutex::scoped_lock lock;

    if((wait_read()>(timeo_ms*1000))&&(timeo_ms>0)) return POWER_SUPPLY_TIMEOUT;
    *state = 0;
    if(regulator_state == REGULATOR_SHOULD_BE_OFF) {
        *state |=POWER_SUPPLY_STATE_OFF;
        desc += "off ";
    }
    
    if(regulator_state == REGULATOR_STANDBY){
        *state |= POWER_SUPPLY_STATE_STANDBY;
        desc += "standby ";
    }
    
    if(regulator_state == REGULATOR_ON){
        *state |= POWER_SUPPLY_STATE_ON;
        desc += "on ";
    }
    if(regulator_state == REGULATOR_UKN){
        *state|=POWER_SUPPLY_STATE_UKN;
        desc += "ukwnown ";
    }
    
    if(regulator_state == REGULATOR_ERROR){
        *state|=POWER_SUPPLY_STATE_ERROR;
        desc += "error ";
    }
    if(alarms!=0){
        *state |= POWER_SUPPLY_STATE_ALARM;
        desc += "Alarm ";
    }
    return 0;
}


int SimPSupply::getCurrentSensibility(float*sens){
    *sens =current_sensibility;
    return 0;
}

int  SimPSupply::getVoltageSensibility(float *sens){
    *sens = voltage_sensibility;
    return 0;
}

int SimPSupply::getMaxMinCurrent(float*max,float*min){
    *min=min_current;
    *max=max_current;
    return 0;
}
int SimPSupply::getMaxMinVoltage(float*max,float*min){
    *min=min_voltage;
    *max=max_voltage;
    return 0;
}

int SimPSupply::getAlarmDesc(uint64_t*desc){
    *desc = POWER_SUPPLY_EVENT_DOOR_OPEN;
    return 0;
}			
