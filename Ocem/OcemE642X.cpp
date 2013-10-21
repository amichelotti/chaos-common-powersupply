//
//  OcemE642X.cpp
//  PowerSupply
//
//  Created by andrea michelotti on 10/11/13.
//  Copyright (c) 2013 andrea michelotti. All rights reserved.
//
#ifdef OCEM_CORE_DEBUG
#define DEBUG
#endif
#include "common/debug/debug.h"


#include "OcemE642X.h"
#include "common/debug/debug.h"
using namespace common::powersupply;


pthread_mutex_t OcemE642X::unique_ocem_core_mutex=PTHREAD_MUTEX_INITIALIZER;
std::map<std::string,OcemProtocol_psh > OcemE642X::unique_protocol;

float  OcemE642X::voltage_sensibility=0;
float  OcemE642X::current_sensibility=0;

#define CMD_WRITE_AND_CHECK(value,cmdw,cmdr,cmdv)			\
  int ret;								\
  DPRINT("apply command \"" cmdw "\" and verify with \"" cmdr "\", expected \"" #cmdv "\" [%d]\n",(unsigned)cmdv); \
  ret =send_command((char*)cmdw);						\
if(ret>0){					\
  if((ret=update_status(&value,(char*)cmdr))>0){	\
    if(value.get()==cmdv){			\
      DPRINT("Command sucessfully applied\n");	\
      return 0;					\
    } else {								\
      DERR("Value %d != %d \"" #cmdv "\"\n",value.get(),cmdv);		\
      return POWER_SUPPLY_READBACK_FAILED;				\
    }									\
  } else {								\
    DERR("data is not in sync\n");					\
    return ret;								\
  }									\
 }									\
DERR("error sending command\n");					\
return POWER_SUPPLY_COMMAND_ERROR;					\


#define GET_VALUE(value,cmdw)			\
  int ret;								\
  DPRINT("apply command \"" cmdw "\""); \
  if((ret=update_status((common::debug::basic_timed*)&value,(char*)cmdw))<=0){			\
    DERR("data is not in sync\n");					\
    return ret;								\
  }									\


void OcemE642X::removeOcemProtocol(const char*dev){
    std::string mydev(dev);
    pthread_mutex_lock(&unique_ocem_core_mutex);
    std::map<std::string,OcemProtocol_psh >::iterator i=unique_protocol.find(mydev);
    unique_protocol.erase(i);
    pthread_mutex_unlock(&unique_ocem_core_mutex);
}
OcemProtocol_psh OcemE642X::getOcemProtocol(const char *dev,int baudrate,int parity,int bits,int stop){
    std::string mydev(dev);
    pthread_mutex_lock(&unique_ocem_core_mutex);
    std::map<std::string,OcemProtocol_psh >::iterator i=unique_protocol.find(mydev);
    if(i!=unique_protocol.end() && i->second!=NULL){
        pthread_mutex_unlock(&unique_ocem_core_mutex);
        DPRINT("RETRIVING serial ocem protocol on \"%s\" @x%x\n",mydev.c_str(),i->second.get());
        return i->second;
    }
    unique_protocol[mydev] =OcemProtocol_psh(new common::serial::OcemProtocol(dev,POSIX_SERIAL_COMM_DEFAULT_MAX_BUFFER_WRITE_SIZE,baudrate,parity,bits,stop));
    DPRINT("creating NEW serial ocem protocol on \"%s\" @x%x\n",mydev.c_str(),unique_protocol[mydev].get());

    pthread_mutex_unlock(&unique_ocem_core_mutex);

    return unique_protocol[mydev];
}

int OcemE642X::update_status(common::debug::basic_timed*data ,char *cmd){
  char buf[1024];
  int ret;
  uint64_t stamp=common::debug::getUsTime();
  if((ret=send_receive((char*)cmd,buf,sizeof(buf)))>0){
    if(data->mod_time()>stamp){
      return 1;
    } else {
      DPRINT("Data is not updated yet\n");
      // try again
      update_status(1000,10);
      if(data->mod_time()>stamp){
	return 1;
      }
      DERR("Data is not updated yet\n");
      return POWER_SUPPLY_READOUT_OUTDATED;
    }
  }
  return ret;
}


OcemE642X::OcemE642X(const char *_dev,int _slave_id,int _baudrate,int _parity,int _bits,int _stop): dev(_dev),baudrate(_baudrate),parity(_parity),bits(_bits),stop(_stop),slave_id(_slave_id)
{
    ocem_prot = getOcemProtocol(dev,baudrate,parity,bits,stop);
    regulator_state = REGULATOR_UKN;
    selector_state = SELECTOR_UKN;
    polarity = POL_UKN;
    voltage_sensibility = ((max_voltage -min_voltage)*1.0)/(1<<voltage_adc);
    current_sensibility = ((max_current-min_current)*1.0)/(1<<current_adc);
}

OcemE642X::~OcemE642X(){
    deinit();
}

int OcemE642X::updateInternalData(char * stringa){
  unsigned channelnum, channelmin,channelmax,datau;
  char chtype,unit0,unit1;
  int parsed=0;
  char* pnt=strtok(stringa,"\n\r"),*pnta;
  char datac[5];
  char rilascio;
  int anno,mese,giorno;
  char tipo[256];
  while(pnt){
      if(sscanf(pnt,"PRG %c%u %u %u %c%c",&chtype,&channelnum,&channelmin,&channelmax,&unit0,&unit1)==6){
	if((chtype == 'I')&&(channelnum<OCEM_INPUT_CHANNELS)){
	  uint64_t tm;
	  ichannel[channelnum] = ocem_channel(channelmin,channelmax);
	  tm =  ichannel[channelnum].mod_time();
	  DPRINT("setting input channel %d at %.10llu us to (%d,%d)\n",channelnum,  tm, ichannel[channelnum].get().min, ichannel[channelnum].get().max);
	  parsed++;
	} else if((chtype == 'O')&&(channelnum<OCEM_OUTPUT_CHANNELS)){
	  ochannel[channelnum] = ocem_channel(channelmin,channelmax);
	  uint64_t tm=  ochannel[channelnum].mod_time();
	  DPRINT("setting output channel %d at %.10llu us to (%d,%d)\n",channelnum,  tm, ochannel[channelnum].get().min, ochannel[channelnum].get().max);
	  parsed++;
	} else {
	  DERR("error parsing PRG\n");
	}	
      } else if(sscanf(pnt,"POL %3s",datac)==1){
	if(!strncmp(datac,"POS",3)){
	  polarity = POL_POS;
	  DPRINT("got Polarity POS\n");
	  parsed++;
	} else if(!strncmp(datac,"NEG",3)){
	  polarity = POL_NEG;
	  DPRINT("got Polarity NEG\n");
	  parsed++;
	} else if(!strncmp(datac,"OPN",3)){
	  polarity = POL_ZERO;
	  DPRINT("got Polarity OPEN\n");
	  parsed++;
	} else if(!strncmp(datac,"RUN",3)){
	  polarity = POL_RUN;
	  DPRINT("got Polarity INSTABLE\n");
	  parsed++;
	} else {
	  DERR("error parsing POL\n");
	}
      } else if(sscanf(pnt,"SEL %3s",datac)==1){
	if(!strncmp(datac,"PRE",3)){
	  selector_state = SELECTOR_PRE;
	  DPRINT("got Selector PRE\n");
	  parsed++;
	} else if(!strncmp(datac,"LOC",3)){
	  selector_state = SELECTOR_LOC;
	  DPRINT("got Selector LOC\n");
	  parsed++;
	} else if(!strncmp(datac,"ERR",3)){
	  selector_state = SELECTOR_ERROR;
	  DPRINT("got Selector ERROR\n");
	  parsed++;
	} else {
	  DERR("error parsing SEL\n");
	}
      } else if(sscanf(pnt,"STA %3s",datac)==1){
	if(!strncmp(datac,"ON",2)){
	  regulator_state = REGULATOR_ON;
	  DPRINT("got Regulator ON\n");
	  parsed++;
	} else if(!strncmp(datac,"STB",3)){
	  regulator_state = REGULATOR_STANDBY;
	  DPRINT("got Regulator Standby\n");
	  parsed++;
	} else if(!strncmp(datac,"ERR",3)){
	  DPRINT("got Regulator Error\n");
	  regulator_state = REGULATOR_ERROR;
	  parsed++;
	} else {
	  DERR("error parsing STA\n");
	}	
      } else if(sscanf(pnt,"COR %u",&datau)==1){
	current = datau;
	DPRINT("got Corrente %d\n",datau);
	parsed++;
      } else if(sscanf(pnt,"TEN %u",&datau)==1){
	voltage = datau;
	DPRINT("got Tensione %d\n",datau);
	parsed++;
      } else if(sscanf(pnt,"VER OCEM SPA TELEOPERATORE %c%2d%2d%2d%8s",&rilascio,&anno,&mese,&giorno,tipo)==5){
	DPRINT("got version rilascio \"%c\" %.2d/%.2d/%.2d tipo:\"%s\"\n",rilascio,giorno,mese,anno,tipo);
	version= ocem_version(rilascio,giorno,mese,anno,tipo);
	parsed++;
      } else if((pnta=strstr(pnt,"ALL"))){
	char *ptr_tmp;
	char *pntt=strtok_r(pnta," \n",&ptr_tmp);
	alarms = 0;
	while (pntt){
	  if(isdigit(*pntt)){
	    int al=atoi(pntt);
	    if((al>=0) && (al <OCEM_ALARM_NUMBER)){
	      alarms = (1LL<<al) | (uint64_t) alarms;
	      DPRINT("setting alarm \"%d\", alarm mask 0x%llx\n",al, (uint64_t)alarms);
	    }
	  }
	  pntt=strtok_r(NULL," \n",&ptr_tmp);
	}
      }
      
      pnt = strtok(NULL,"\n\r");
  }
  return parsed;
}

int OcemE642X::update_status(int timeout,int msxpoll){
  uint64_t tstart;
  uint64_t totPollTime=0;			
  char buf[2048];
  int retry=0;
  int ret;
  int updated=0;
  do{
    tstart= common::debug::getUsTime();
    ret = receive_data( buf, sizeof(buf),timeout);
    totPollTime+= common::debug::getUsTime()-tstart;
    DPRINT("Check returned %d retry %d checking result %d, tot Poll time %.10llu us\n",ret,retry++,ret,totPollTime);
    if(ret>0){
      updated+=ret;
    }
    if(msxpoll)
      usleep(msxpoll*1000);
  } while((ret>0)&& ((totPollTime/1000)<(unsigned)timeout));
  return updated;
}


int OcemE642X::init(){
    int ret=-1;
    char buf[2048];
    *buf = 0;
    regulator_state = REGULATOR_UKN;
    selector_state = SELECTOR_UKN;
    polarity = POL_UKN;
    DPRINT("initialising\n");
    if(ocem_prot){
        ret = ocem_prot->init();
    } else {
        ocem_prot = getOcemProtocol(dev,baudrate,parity,bits,stop);
        if(ocem_prot){
            ret = ocem_prot->init();
        }
    }
    if(ret==0){

      // poll trial
      update_status(1000,10);

   
      // update status
      if((ret=force_update())>0){
	DPRINT("Ocem status has been refreshed %d\n",ret);
	ret = 0;
      } else {
	DERR("Nothing has been refreshed  %d\n",ret);
	return -1;
      }

    }
    
    return ret;
}
int OcemE642X::force_update(){
  int rett;
  char buf[2048];
  DPRINT("forcing update\n");
  if((rett=send_receive((char*)"RMT",buf,sizeof(buf),1000,1000))<=0){
    DERR("RMT command failed %d \n",rett);
    return rett;
  }
  DPRINT("updated %d data, check if something else\n",rett);
  rett+=update_status(1000,100);
  DPRINT("totale updated %d data\n",rett);
  return rett;
}
int OcemE642X::deinit(){
    ocem_prot.reset();
    removeOcemProtocol(dev);
    
    return 0;
}



// return the number of characters sucessfully written or an error
int OcemE642X::send_command(char*cmd,int timeout){
    char command[1024];
    int tim=0,ret;
    if(cmd==NULL) 
      return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    DPRINT("sending command \"%s\" to slave %d\n",cmd,slave_id);
    snprintf(command,sizeof(command),"%s" OCEM_TERMINATOR,cmd);
    ret =ocem_prot->select(slave_id, command,timeout,&tim);
    if(ret>0){
      DPRINT("command sent %d bytes (timeout %d) sucessfully \n",ret,tim);
    } else if(ret == 0){
      DPRINT("nothing has been sent, timeout %d\n",tim);
    } else {
      DERR("error %d occurred (timeout %d)\n",ret,tim);
      return POWER_SUPPLY_COMMAND_ERROR;
    }

    return ret;
}
// return the number of characters sucessfully read or an error
int OcemE642X::send_receive(char*cmd,char*buf,int size,int timeos,int timeop){
  int ret,retry=0;
  uint64_t tstart;
  uint64_t totPollTime=0;
  DPRINT("sending command timeout set to %u ms\n",timeos);
  ret=send_command(cmd,timeos);
 
  if(ret>0){
    DPRINT("command returned %d, check timeout set to %u ms\n",ret,timeop);
    do{
      tstart= common::debug::getUsTime();
      ret = receive_data( buf, size,timeop);
      totPollTime+= common::debug::getUsTime()-tstart;
      DPRINT("retry %d checking result %d, tot Poll time %.10llu us, polling tim %.10llu ms max poll time %u ms\n",retry++,ret,totPollTime,(totPollTime/1000),timeop);
    } while((ret<=0)&& (totPollTime/1000)<(unsigned)timeop);
  } 
  return ret;
}
// return the number of data sucessfully read or an error
int OcemE642X::receive_data(char*buf, int size,int timeout){
    int ret,tim=0;
    int data_read=0;
    if(buf==NULL) return -1;
    DPRINT("wait for messages from slave %d\n",slave_id);
    ret=ocem_prot->poll(slave_id, buf, size,timeout,&tim);
    if(ret>0){
        DPRINT("received %d bytes (timeout %d) \"%s\"\n",ret,tim,buf);
	data_read=updateInternalData(buf);
    } else if(ret == 0){
        DPRINT("nothing received, timeout %d\n",tim);
    } else {
        DERR("error %d occurred (timeout %d)\n",ret,tim);
	data_read =ret;
	return POWER_SUPPLY_RECEIVE_ERROR;
    }
    
    return data_read;
}

std::string OcemE642X::getSWVersion(){
    char stringa[1024];
    int ret;
    if((ret=update_status(&version,(char*)"VER"))<=0){ 
      DERR("data is not in sync\n");					
      return std::string();							
    }	

    sprintf(stringa,"Release '%c' %.2d/%.2d/%.2d type:%s\n",version.get().rilascio,version.get().giorno,version.get().mese,version.get().anno, version.get().type);

    return std::string(stringa);
}

std::string OcemE642X::getHWVersion(){
    return getHWVersion();
}


int OcemE642X::setPolarity(int pol){
  if(pol>0){
    CMD_WRITE_AND_CHECK(polarity,"POS","POL",POL_POS);
  } else if(pol<0){
    CMD_WRITE_AND_CHECK(polarity,"NEG","POL",POL_NEG);
  } else if(pol==0){
    CMD_WRITE_AND_CHECK(polarity,"OPN","POL",POL_ZERO);
  }
  return 0;
}

 int OcemE642X::getPolarity(int* pol){
   *pol = polarity;
   GET_VALUE(polarity,"POL");
   *pol = polarity;
   return 0;
 }

int OcemE642X::setCurrentSP(float current){
  char stringa[256];
  int ret;
  if(current<min_current || current>max_current)
    return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
  
  snprintf(stringa,sizeof(stringa),"SP %d",(int)(current/current_sensibility));
  
  sp_current = current/current_sensibility;
  if((ret=send_command(stringa))<=0){
    DERR("cannot execute \"%s\"\n",stringa);
    return ret;
  }

  return 0;
 }


 int OcemE642X::getCurrentSP(float* current){
   *current = sp_current*current_sensibility;
   return 0;
 }

int  OcemE642X::getCurrentOutput(float* curr){
  *curr = current*current_sensibility;
   GET_VALUE(current,"COR");
   *curr = current*current_sensibility;
  return 0;
}
int  OcemE642X::getVoltageOutput(float* volt){
  *volt = voltage*current_sensibility;
  GET_VALUE(voltage,"TEN");
  *volt = voltage*current_sensibility;
  return 0;
}

 int OcemE642X::startCurrentRamp(){
   return send_command((char*)"STR");
 }



int OcemE642X::setCurrentRampSpeed(float asup,float asdown){
  int rsup,rsdown,ret;
  if(asup<OCEM_MIN_VOLTAGE || asup > OCEM_MAX_VOLTAGE)
    return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
  
  if(asdown<OCEM_MIN_VOLTAGE || asdown > OCEM_MAX_VOLTAGE)
    return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
  rsup=(asup/OCEM_MAX_VOLTAGE)*(1<<OCEM_CURRENT_RAMP_ADC);
  rsdown=(asdown/OCEM_MAX_VOLTAGE)*(1<<OCEM_CURRENT_RAMP_ADC);
  DPRINT("setting ramp rising speed to %f as (0x%x), falling speed to %f (0x%x)\n",asup,rsup,asdown,rsdown);
  if((ret=setChannel(CHANNEL_OUT,CHANNEL_RAMP_UP,0,rsup))<0){
    DERR("error setting current RAMP UP \n");
    return ret;
  }
  if((ret=setChannel(CHANNEL_OUT,CHANNEL_RAMP_DOWN,0,rsdown))<0){
    DERR("error setting current RAMP DOWN \n");
    return ret;
  }
  return 0;
}



 int OcemE642X::resetAlarms(uint64_t alrm){
   return send_command((char*)"RES");
 }

 int OcemE642X::getAlarms(uint64_t*alrm){
   *alrm = alarms;
   GET_VALUE(alarms,"ALL");
   *alrm = alarms;
   return 0;
 }


 int OcemE642X::shutdown(){
   regulator_state = REGULATOR_SHOULD_BE_OFF;
   // because we lose control once done this command
   DPRINT("SHUTDOWN command, we are going to lose control!!\n");
   return send_command((char*)"OFF");
 }
int OcemE642X::poweron(){
  CMD_WRITE_AND_CHECK(regulator_state,"ON","SEL",REGULATOR_ON);
}

 int OcemE642X::standby(){
   CMD_WRITE_AND_CHECK(regulator_state,"STB","SEL",REGULATOR_STANDBY);
 }
  

int  OcemE642X::getState(int* state,std::string &desc){
  GET_VALUE(regulator_state,"SL");
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

int OcemE642X::setChannel(int inout,int number, int min,int max){
  char chtype;
  char um[3];
  char cmd[256];
  *um=0;
  if((inout==0) && (number < OCEM_INPUT_CHANNELS )) {
    chtype = 'I';
    if(number==0)
      strcpy(um,"mA");
  } else if((inout==1) && (number < OCEM_OUTPUT_CHANNELS )) {
    chtype = 'O';
    if(number==0)
      strcpy(um,"mA");
    else if(number==1 || number==2){
      strcpy(um,"AS");
    }

  } else{
    return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
  }

  snprintf(cmd,sizeof(cmd),"PRG %c%d %.7d %.7d %s",chtype,number,min,max,um);
  DPRINT("%s\n",cmd);
  return send_command(cmd);
}
int OcemE642X::getChannel(int inout,int number, int* min,int* max){
  char buf[2048];
  uint64_t stamp=common::debug::getUsTime();
  if(!(min && max)){
    return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
  }
  send_receive((char*)"PRG S",buf,sizeof(buf));
  update_status(1000,10);
  if((inout==0) && (number < OCEM_INPUT_CHANNELS )) {
    *min = ichannel[number].get().min;
    *max = ichannel[number].get().max;
    if(ichannel[number].mod_time()> stamp){
      DPRINT("got input channel %d update %.7d %.7d\n",number,*min,*max);
      return 0;
    } else {
      DERR("data is not available yet\n");
      return POWER_SUPPLY_READOUT_OUTDATED;
    }
  } else if((inout==1) && (number < OCEM_OUTPUT_CHANNELS )) {
    *min = ochannel[number].get().min;
    *max = ochannel[number].get().max;
    if(ochannel[number].mod_time()> stamp){
      DPRINT("got output channel %d update %.7d %.7d\n",number,*min,*max);
      return 0;
    } else {
      DERR("data is not available yet\n");
      return POWER_SUPPLY_READOUT_OUTDATED;
    }
  } else {
    return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
  }

  return 0;
}
int OcemE642X::setThreashold(int channel,int value){
  char cmd[256];
  if(channel>=4){
    return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
  }
  snprintf(cmd,sizeof(cmd),"TH I%d %d\n",channel,value);
  
  return send_command(cmd);
}


float  OcemE642X::getCurrentSensibility(){

  return current_sensibility;
}		
float  OcemE642X::getVoltageSensibility(){

  return  voltage_sensibility;
}

void OcemE642X::getMaxMinCurrent(float*max,float*min){
  *min=min_current;
  *max=max_current;
}
void OcemE642X::getMaxMinVoltage(float*max,float*min){
  *min=min_voltage;
  *max=max_voltage;
}
			
