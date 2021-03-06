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

#include <common/debug/core/debug.h>

#include <math.h>

#include "OcemE642X.h"
using namespace common::powersupply;


pthread_mutex_t OcemE642X::unique_ocem_core_mutex=PTHREAD_MUTEX_INITIALIZER;
std::map<std::string,OcemE642X::OcemProtocol_psh > OcemE642X::unique_protocol;

#define INIT_RETRY 10

#define CONV2ADC(what,x) (x/OCEM_MAX_ ## what)*(1<<OCEM_ ## what ## _ADC)
#define ADC2CURRENT(what,x) (x/(1<<OCEM_ ## what ## _ADC))*OCEM_MAX_ ## what

#define CMD_WRITE(_cmdw,_timeo) \
{									\
int _ret,_timeout=0;							\
DPRINT("[%d] CMD_WRITE apply command \"%s\" timeout %d",slave_id,_cmdw,_timeo);			\
_ret =send_command((char*)_cmdw,_timeo,&_timeout);	\
if(_timeout==1) {DERR("timeout sending command %s",_cmdw);return POWER_SUPPLY_TIMEOUT;};				\
if(_ret<0) return POWER_SUPPLY_COMMAND_ERROR;			\
}

#ifdef OCEM_CHECK_ENABLED
#define CMD_WRITE_AND_CHECK(value,_timeo,cmdw,cmdr,cmdv) \
{									\
int _ret;								\
DPRINT("apply command \"" cmdw "\" and verify with \"" cmdr "\", expected \"" #cmdv "\" [%d]",(unsigned)cmdv); \
_ret =send_command((char*)cmdw,_timeo);					\
if(_ret>0){					\
if((_ret=update_status(&value,(char*)cmdr,_timeo))>0){	\
if(value.get()==cmdv){			\
DPRINT("Command sucessfully applied");	\
return 0;					\
} else {								\
DERR("Value %d != %d \"" #cmdv "\"",value.get(),cmdv);		\
return POWER_SUPPLY_READBACK_FAILED;				\
}									\
} else {								\
DERR("data is not in sync");					\
return _ret;								\
}									\
}									\
DERR("error sending command");					\
return POWER_SUPPLY_COMMAND_ERROR;					\
}
#else
#define CMD_WRITE_AND_CHECK(value,_timeo,cmdw,cmdr,cmdv)			\
CMD_WRITE(cmdw,_timeo)
#endif


#define GET_VALUE(value,_timeo,cmdw) {									\
    int _ret;								\
    uint64_t last=(common::debug::getUsTime()-cmdw.mod_time());		\
 if(last<OCEM_REFRESH_TIME) {		\
   DPRINT("[%d] NOT APPLY (too near %llu ago) command \"%s\" timeout %d",slave_id, last,# cmdw,_timeo);	\
 update_status(500,10);}					\
 else{DPRINT("[%d] apply command \"%s\" last command %llu us ago timeout %d",slave_id, # cmdw,last,_timeo); cmdw=2; \
   if((_ret=update_status((common::debug::basic_timed*)&value,(char*)# cmdw,_timeo))<=0){ \
  DERR("[%d] cmd \"%s\" for slaveid %d data is not in sync, ret %d",slave_id,# cmdw,_ret);		\
return _ret;							\
 }}								\
 }



 void* OcemE642X::update_thread(void* p){
    OcemE642X* pnt = (OcemE642X*)p;
    return (void*)pnt->updateSchedule();
}
void* OcemE642X::updateSchedule(){
    int ret;
    uint64_t doneat0,doneat1;
    char buf[2048];
    int s=0;
    DPRINT("[%d] OCEM  STARTED 0x%x ",slave_id,pthread_self());
    run=1;
    while(run){
      DPRINT("[%d] SLAVE SCHEDULE",slave_id);
      ret = receive_data( buf, sizeof(buf),1000,0);
      uint64_t t=common::debug::getUsTime();
      if(current.mod_time()>OCEM_REFRESH_TIME){
          if((ocem_prot->getWriteSize(slave_id)==0)&&(ocem_prot->getReadSize(slave_id)==0)){
            DPRINT("[%d] REFRESH ANALOGIC, because current has been modified %llu ms ago",slave_id,current.mod_time()/1000);
            send_command((const char*)"SA",1000,0);
          }
      }
      if(regulator_state.mod_time()>(2*OCEM_REFRESH_TIME)){
          if((ocem_prot->getWriteSize(slave_id)==0)&&(ocem_prot->getReadSize(slave_id)==0)){
            DPRINT("[%d] REFRESH LOGIC  because state has been modified %llu ms",slave_id,regulator_state.mod_time()/1000);
            send_command((const char*)"SL",1000,0);
          }
      }
      /*
      if((ret<=0) && ((t-doneat0)>OCEM_REFRESH_TIME)){
	if(s==0){
	  DPRINT("[%d] REFRESH STATE",slave_id);
	  send_command((const char*)"SL",1000,0);
	  } else{
	  
	}
	s=!s;
        doneat0=t;
      }
       */
      
    }
      
    DPRINT("[%d] EXITING SLAVE UPDATE THREAD",slave_id);
}

void OcemE642X::removeOcemProtocol(std::string& mydev){
  DPRINT(" removing protocol on \"%s\"",mydev.c_str());
  pthread_mutex_lock(&unique_ocem_core_mutex);
  std::map<std::string,OcemProtocol_psh >::iterator i=unique_protocol.find(mydev);
  if(i!=unique_protocol.end()){
    DPRINT("found removing ");
    unique_protocol.erase(i);
  }
  pthread_mutex_unlock(&unique_ocem_core_mutex);
}
OcemE642X::OcemProtocol_psh OcemE642X::getOcemProtocol(std::string& mydev,int baudrate,int parity,int bits,int stop){
  DPRINT("getting protocol for \"%s\" baudrate %d , parity %d bits %d stop %d",mydev.c_str(),baudrate,parity,bits,stop);
    pthread_mutex_lock(&unique_ocem_core_mutex);
    std::map<std::string,OcemProtocol_psh >::iterator i=unique_protocol.find(mydev);
    if(i!=unique_protocol.end() && i->second!=NULL){
        pthread_mutex_unlock(&unique_ocem_core_mutex);
        DPRINT("RETRIVING serial ocem protocol on \"%s\" @x%Lx",mydev.c_str(),(unsigned long long)i->second.get());
        return i->second;
    }
    //    unique_protocol[mydev] =OcemProtocol_psh(new common::serial::ocem::OcemProtocol(mydev.c_str(),POSIX_SERIAL_COMM_DEFAULT_MAX_BUFFER_WRITE_SIZE,baudrate,parity,bits,stop));
    unique_protocol[mydev] =OcemProtocol_psh(new common::serial::ocem::OcemProtocolScheduleCFQ(mydev.c_str(),POSIX_SERIAL_COMM_DEFAULT_MAX_BUFFER_WRITE_SIZE,baudrate,parity,bits,stop));
    DPRINT("creating NEW serial ocem protocol on \"%s\" @x%Lx",mydev.c_str(),(unsigned long long)unique_protocol[mydev].get());
    
    pthread_mutex_unlock(&unique_ocem_core_mutex);
    
    return unique_protocol[mydev];
}

int OcemE642X::update_status(common::debug::basic_timed*data ,char *cmd,uint32_t timeout){
    
    //char buf[2048];
    //uint64_t stamp=common::debug::getUsTime();
  if((common::debug::getUsTime()-data->mod_time())>(OCEM_REFRESH_TIME)){
        CMD_WRITE(cmd,timeout);
    }
    update_status(600,200);
    DPRINT("[%d] data type \"%s\" last update at %10llu us ago",slave_id,data->get_name(),(common::debug::getUsTime()- data->mod_time()));
    
    return 1;

#if 0
    while(((ret_time=data->mod_time())<stamp)){
       int ret;
       
       if((ret=receive_data( buf, sizeof(buf),timeout,&tim))>0){
       
	  DPRINT("read again ");
 
       } else {

	 if(((timeout>0) && ((common::debug::getUsTime()-stamp)>=(timeout*1000))) || (tim > 0)){
	   DPRINT("Timeout reading data type \"%s\" Start operation %10llu (duration %10llu) > %d timeout",data->get_name(),stamp,(common::debug::getUsTime()-stamp)/1000,timeout);
	   tim++;
	   break;
	 }
       }
     
    }
    
    if(data->mod_time()>stamp){
        DPRINT("[%d] data type \"%s\" has been updated at %10llu",slave_id,data->get_name(),data->mod_time());
        return 1;
    } else {
        if(tim>0){
	  DERR("[%d] Timeout retrieving data \"%s\"",slave_id,data->get_name());
            return POWER_SUPPLY_TIMEOUT;
        } 
    }
    DERR("Data is not updated yet");
    return POWER_SUPPLY_READOUT_OUTDATED;
#endif 
}


void OcemE642X::init_internal(){
  int cnt;
  model = OCEM_NOMODEL;
    INITIALIZE_TIMED(regulator_state,REGULATOR_UKN);
    INITIALIZE_TIMED(selector_state,SELECTOR_UKN);
    INITIALIZE_TIMED(polarity, POL_UKN);
    INITIALIZE_TIMED(current, 0U);
    INITIALIZE_TIMED(voltage, 0U);
    INITIALIZE_TIMED(sp_current, 0U);
    INITIALIZE_TIMED(alarms, 0ULL);
    INITIALIZE_TIMED(version, ocem_version());
    // default values to be determined by type
    /*
      max_current=OCEM_MAX_CURRENT;
      min_current=OCEM_MIN_CURRENT;
      max_voltage=OCEM_MAX_VOLTAGE;
      min_voltage=OCEM_MIN_VOLTAGE;
      
    adc_voltage_conversion=((max_voltage -min_voltage)*1.0)/(1<<voltage_adc);
    adc_current_conversion = ((max_current-min_current)*1.0)/(1<<current_adc);
    */
    available_alarms =0;
    for(cnt=0;cnt<OCEM_INPUT_CHANNELS;cnt++){
        INITIALIZE_TIMED(ichannel[cnt],ocem_channel(0,0));
    }
    for(cnt=0;cnt<OCEM_OUTPUT_CHANNELS;cnt++){
        INITIALIZE_TIMED(ochannel[cnt],ocem_channel(0,0));
	
    }
}
OcemE642X::OcemE642X(const char *_dev,int _slave_id,int _baudrate,int _parity,int _bits,int _stop): dev(_dev),baudrate(_baudrate),parity(_parity),bits(_bits),stop(_stop),slave_id(_slave_id)
{
  
    DPRINT("CREATE %s id %d",_dev,_slave_id);
    ocem_prot = getOcemProtocol(dev,baudrate,parity,bits,stop);
    max_current=0;
    min_current=0;
    max_voltage=0;
    min_voltage=0;
    initialized=0;
    init_internal();
    
}

OcemE642X::OcemE642X(const char *_dev,int _slave_id,float maxcurr,float maxvoltage): dev(_dev),baudrate(9600),parity(0),bits(8),stop(1),slave_id(_slave_id){
    DPRINT("CREATE %s id %d",_dev,_slave_id);
    initialized=0;
    ocem_prot = getOcemProtocol(dev,baudrate,parity,bits,stop);

    if(maxcurr>0)
      forceMaxCurrent(maxcurr);
    if(maxvoltage>0)
      forceMaxVoltage(maxvoltage);
    init_internal();
}

OcemE642X::~OcemE642X(){
    DPRINT("[%d] deinitializing",slave_id)
    deinit();
}
uint64_t OcemE642X::getFeatures() {
    return POWER_SUPPLY_FEAT_MONOPOLAR;
}

int OcemE642X::updateInternalData(char * stringa){
    unsigned channelnum=0, channelmin=0,channelmax=0,datau=0;
    char chtype=0,unit0=0,unit1=0;
    int parsed=0;
    char*pnta=0;
    char* pnt=0;
    char datac[5];
    char rilascio=0;
    int anno=0,mese=0,giorno=0;
    char tipo[256];
    *tipo=0;
    pnt =strtok(stringa,"\n\r");
    while(pnt){
        if(sscanf(pnt,"PRG %c%u %u %u %c%c",&chtype,&channelnum,&channelmin,&channelmax,&unit0,&unit1)==6){
            if((chtype == 'I')&&(channelnum<OCEM_INPUT_CHANNELS)){
                ichannel[channelnum] = ocem_channel(channelmin,channelmax);
               
                DPRINT("setting input channel %d at %.10llu us to (%d,%d)",channelnum,  ichannel[channelnum].mod_time(), ichannel[channelnum].get().min, ichannel[channelnum].get().max);
                parsed++;
            } else if((chtype == 'O')&&(channelnum<OCEM_OUTPUT_CHANNELS)){
                ochannel[channelnum] = ocem_channel(channelmin,channelmax);

                DPRINT("setting output channel %d at %.10llu us to (%d,%d)",channelnum,  ochannel[channelnum].mod_time(), ochannel[channelnum].get().min, ochannel[channelnum].get().max);
                parsed++;
            } else {
                DERR("error parsing PRG");
            }
        } else if(sscanf(pnt,"POL %3s",datac)==1){
            if(!strncmp(datac,"POS",3)){
                polarity = POL_POS;
                DPRINT("[%d] got Polarity POS",slave_id);
                parsed++;
            } else if(!strncmp(datac,"NEG",3)){
                polarity = POL_NEG;
                DPRINT("[%d] got Polarity NEG",slave_id);
                parsed++;
            } else if(!strncmp(datac,"OPN",3)){
                polarity = POL_ZERO;
                DPRINT("[%d] got Polarity OPEN",slave_id);
                parsed++;
            } else if(!strncmp(datac,"RUN",3)){
                polarity = POL_RUN;
                DPRINT("[%d] got Polarity INSTABLE",slave_id);
                parsed++;
            } else {
                DERR("error parsing POL");
            }
        } else if(sscanf(pnt,"SEL %3s",datac)==1){
            if(!strncmp(datac,"PRE",3)){
                selector_state = SELECTOR_PRE;
                DPRINT("[%d] got Selector PRE",slave_id);
                parsed++;
            } else if(!strncmp(datac,"LOC",3)){
                selector_state = SELECTOR_LOC;
                DPRINT("[%d] got Selector LOC",slave_id);
                parsed++;
            } else if(!strncmp(datac,"ERR",3)){
                selector_state = SELECTOR_ERROR;
                DPRINT("[%d] got Selector ERROR",slave_id);
                parsed++;
            } else {
                DERR("error parsing SEL");
            }
        } else if(sscanf(pnt,"STA %3s",datac)==1){
            if(!strncmp(datac,"ATT",2)){
                regulator_state=REGULATOR_ON;
                DPRINT("[%d] got Regulator ON",slave_id);
                parsed++;
            } else if(!strncmp(datac,"STB",3)){
                regulator_state = REGULATOR_STANDBY;
                DPRINT("[%d] got Regulator Standby",slave_id);
                parsed++;
            } else if(!strncmp(datac,"ERR",3)){
                DPRINT("[%d] got Regulator Error",slave_id);
                regulator_state = REGULATOR_ERROR;
                parsed++;
            } else {
                DERR("error parsing STA");
            }
        } else if(sscanf(pnt,"COR %u",&datau)==1){
            current = datau;
            DPRINT("[%d] got Corrente %d",slave_id,datau);
            parsed++;
        } else if(sscanf(pnt,"TEN %u",&datau)==1){
            voltage = datau;
            DPRINT("[%d] got Tensione %d",slave_id,datau);
            parsed++;
        } else if(sscanf(pnt,"VER OCEM SPA TELEOPERATORE %c%2d%2d%2d%8s",&rilascio,&anno,&mese,&giorno,tipo)==5){
            DPRINT("[%d] got version rilascio \"%c\" %.2d/%.2d/%.2d tipo:\"%s\"",slave_id,rilascio,giorno,mese,anno,tipo);
            version= ocem_version(rilascio,giorno,mese,anno,tipo);
            parsed++;
        } else if((pnta=strstr(pnt,"ALL"))){
            char *ptr_tmp=0;
            char *pntt=strtok_r(pnta," \n",&ptr_tmp);
            alarms = 0ULL;
            while (pntt){
                if(isdigit(*pntt)){
                    int al=atoi(pntt);
                    if((al>=0) && (al <OCEM_ALARM_NUMBER)){
                        switch(al){
                            case AC_UNBALANCE:
                                alarms=common::powersupply::POWER_SUPPLY_AC_UNBALANCE | alarms;
                                DPRINT("AC unbalance");
                                break;
                            case PHASE_LOSS:
                                alarms=common::powersupply::POWER_SUPPLY_PHASE_LOSS | alarms;
                                DPRINT("PHASE LOSS");
                                break;
                            case AIR_FLOW:
                                alarms=common::powersupply::POWER_SUPPLY_AIR_FLOW | alarms;
                                DPRINT("AIR FLOW");
                                break;
                            case DOORS:
                                alarms=common::powersupply::POWER_SUPPLY_EVENT_DOOR_OPEN | alarms;
                                DPRINT("DOOR OPEN");
                                break;
                                
                            case TRASFORMER_OVT:
                                alarms=common::powersupply::POWER_SUPPLY_TRANSFORMER_OVT | alarms;
                                DPRINT("TRANSFORMER OVT");
                                break;
                             case SNUBBER_FUSE:
                                alarms=common::powersupply::POWER_SUPPLY_SNUBBER_FUSE | alarms;
                                DPRINT("SNUBBER_FUSE");
                                break; 
                             case SCR_FUSE:
                                alarms=common::powersupply::POWER_SUPPLY_SCR_FUSE | alarms;
                                DPRINT("SCR_FUSE");
                                break; 
                                
                             case SCR_OVT:
                                alarms=common::powersupply::POWER_SUPPLY_SCR_OVT | alarms;
                                DPRINT("SCR_OVT");
                                break; 
                             case CHOKE_OVT:
                                alarms=common::powersupply::POWER_SUPPLY_CHOKE_OVT | alarms;
                                DPRINT("CHOKE_OVT");
                                break; 
                                
                             case PASS_FILTER:
                                alarms=common::powersupply::POWER_SUPPLY_PASS_FILTER | alarms;
                                DPRINT("PASS_FILTER");
                                break;
                             case DIODE_FAULT:
                                alarms=common::powersupply::POWER_SUPPLY_DIODE_FAULT | alarms;
                                DPRINT("DIODE_FAULT");
                                break;
                            case DIODE_OVT:
                            alarms=common::powersupply::POWER_SUPPLY_DIODE_OVT | alarms;
                            DPRINT("DIODE_OVT");
                            break;
                            
                            case ACTIVE_FILTER_OVT:
                            alarms=common::powersupply::POWER_SUPPLY_ACTIVE_FILTER_OVT | alarms;
                            DPRINT("ACTIVE_FILTER_OVT");
                            break;
                            
                            case ACTIVE_FILTER_FUSE:
                            alarms=common::powersupply::POWER_SUPPLY_ACTIVE_FILTER_FUSE | alarms;
                            DPRINT("ACTIVE_FILTER_FUSE");
                            break;
                            
                            case DCCT_FAULT:
                            alarms=common::powersupply::POWER_SUPPLY_DCCT_FAULT | alarms;
                            DPRINT("DCCT_FAULT");
                            break;
                            
                            case DCCT_OVT:
                            alarms=common::powersupply::POWER_SUPPLY_DCCT_OVT | alarms;
                            DPRINT("DCCT_OVT");
                            break;

                            case EARTH_FAULT:
                            alarms=common::powersupply::POWER_SUPPLY_EARTH_FAULT | alarms;
                            DPRINT("EARTH_FAULT");
                            break;
                            
                          /*  case CUBICLE_OVT:
                            alarms=common::powersupply::POWER_SUPPLY_CUBICLE_OVT | alarms;
                            DPRINT("CUBICLE_OVT");
                            break;
                            */
                            case SETPOINT_CARD_FAULT:
                            alarms=common::powersupply::POWER_SUPPLY_SETPOINT_CARD_FAULT | alarms;
                            DPRINT("SETPOINT_CARD_FAULT");
                            break;
                            
                            case EXTERNAL_INTERLOCK:
                            case CUBICLE_OVT:
                                alarms=common::powersupply::POWER_SUPPLY_EXTERNAL_INTERLOCK | alarms;
                                DPRINT("EXTERNAL_INTERLOCK");
                            break;
                                
                            default:
                                alarms=common::powersupply::POWER_SUPPLY_ALARM_UNDEF | alarms;
                                DPRINT("POWER_SUPPLY_ALARM_UNDEF");
                            break;
                        }
                        
                        DPRINT("setting alarm \"%d\", alarm mask 0x%llx",al, (unsigned long long)alarms);
                    }
                }
                pntt=strtok_r(NULL," \n",&ptr_tmp);
            }
        }
        
        pnt = strtok(NULL,"\n\r");
    }
    return parsed;
}

int OcemE642X::update_status(uint32_t timeout,int msxpoll){
    uint64_t tstart;
    uint64_t totPollTime=0;
    char buf[2048];
    int ret;
    int updated=0;
    int timeo=0;

    do{
        tstart= common::debug::getUsTime();
        ret = receive_data( buf, sizeof(buf),timeout,&timeo);
        totPollTime+= common::debug::getUsTime()-tstart;
        if(ret>0){
            updated+=ret;
            DPRINT("Checking result %d, tot Poll time %.10llu us",ret,totPollTime);

        }
        if(msxpoll)
            usleep(msxpoll*1000);
    } while((ret>0)&& ((totPollTime/1000)<(unsigned)timeout) && (timeo==0));
    return updated;
}


void OcemE642X::updateParamsByModel(OcemModels model){
  min_current=OCEM_MIN_CURRENT;
  min_voltage=OCEM_MIN_VOLTAGE;

  switch(model){
  case OCEM_MODEL234:{
    if(max_current==0){
      max_current = 100;
      DPRINT("MODEL 234 auto detecting max current %f",max_current);
    } else {
      DPRINT("MODEL 234 max current forced to %f",max_current);
    }
    if(max_voltage==0)
      max_voltage = 10;
    
    model = OCEM_MODEL234;
    break;
  }
  case OCEM_MODEL5A5B:{
    if(max_current==0){
      max_current = 700;
      DPRINT("MODEL 5A5B auto detecting max current %f",max_current);
    } else {
      DPRINT("MODEL 5A5B forced to max current %f",max_current);
    }
    max_voltage = 10;
    model = OCEM_MODEL5A5B;
    break;
  }
  default:
    break;
  }
    
  
  forceMaxCurrent(max_current);
  forceMaxVoltage(max_voltage);
  available_alarms =0;
}
///////
int OcemE642X::ocemInitialization(){
    int timeo;
    int ret;
    int result=0;
    char buf[2048];
    int cnt=2;
    int retry =10;
    DPRINT("[%d] OCEM CUSTOM INITIALIZATION",slave_id);
    do{
       DPRINT("[%d] performing poll %d",slave_id,cnt);
  
      ret=ocem_prot->OcemProtocol::poll(slave_id, buf, sizeof(buf),1000,&timeo);
      DPRINT("[%d] returned %d",slave_id,ret);

      if(ret>0){
	updateInternalData(buf);
        result++;
      } else if(ret==common::serial::ocem::OcemProtocol::OCEM_NO_TRAFFIC){
        DPRINT("[%d] NO traffic",slave_id);

      }
      usleep(500000);
    } while(cnt--);

    DPRINT("[%d] performing RMT",slave_id);
    ret=ocem_prot->OcemProtocol::select(slave_id, "RMT",1000,&timeo);
    DPRINT("[%d] returned %d",slave_id,ret);

    sleep(3);
    cnt=0;
    do{
        DPRINT("[%d] performing cleaning poll %d",slave_id,cnt);

        ret=ocem_prot->OcemProtocol::poll(slave_id, buf, sizeof(buf),1000,&timeo);
        DPRINT("[%d] returned %d",slave_id,ret);
        //at the beggining some Ocem generate crc error
	if((ret>0) || (ret==common::serial::ocem::OcemProtocol::OCEM_POLL_ANSWER_CRC_FAILED)) {
	  updateInternalData(buf);
          result++;
	}
	      
        usleep(500000);
        cnt++;
        if((ret==common::serial::ocem::OcemProtocol::OCEM_NO_TRAFFIC)){
             DPRINT("[%d] NO traffic",slave_id);
             result++;
             break;
        }
    } while (((ret>0)||(retry--)));
   DPRINT("[%d] END OCEM CUSTOM INITIALIZATION sucessfully operations:%d",slave_id,result);

   if(result==0){
       DERR("[%d] no activity",slave_id);
       return -3;
   }
   ocem_prot->OcemProtocol::select(slave_id, "SL",1000,&timeo);

   return 0;
    
}

/////////
int OcemE642X::init(){
    int ret=-1;
    char buf[2048];
    int cnt,max,min;
      pthread_attr_t attr;

    *buf = 0;
    regulator_state = REGULATOR_UKN;
    selector_state = SELECTOR_UKN;
    polarity = POL_UKN;
    
    DPRINT("INITIALIZING");
    if(ocem_prot){
      ocem_prot->registerSlave(slave_id);
      ret = ocem_prot->init();

	DPRINT("ocem protocol initialized ret=%d",ret)
    } else {
        ocem_prot = getOcemProtocol(dev,baudrate,parity,bits,stop);
        if(ocem_prot){
	  ocem_prot->registerSlave(slave_id);
	  ret = ocem_prot->init();

	  DPRINT("ocem protocol created and initialized ret=%d",ret)
        } else {
	  ERR("CANNOT ALLOCATE OCEM PROTOCOL");
	  return -12;
	}
	
    }
    if(ret!=0){
      ERR("CANNOT INITIALIZE SERIAL PORT");
      return ret;
    }
    
    if(ocemInitialization()<0){
        return -1000;
    }
    ocem_prot->start();
    if(initialized==0){
        

        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

        if(pthread_create(&rpid,&attr,update_thread,this)<0){
          DERR("cannot create schedule_thread thread");
          return -1;
        }
    }
    initialized=1;


    if(strstr(version.get().type,"/2-3-4")){
      DPRINT("type 2-3-4 detected max current 100A");
      updateParamsByModel(OCEM_MODEL234);
    } else if(strstr(version.get().type,"/1-5A-5B")){
      DPRINT("type 2-5A-5B detected max current 700A");
      updateParamsByModel(OCEM_MODEL5A5B);
    } else {
      DPRINT("uknown type:\"%s\"",version.get().type);
      updateParamsByModel(OCEM_UKNOWN);
    }
    
    if(adc_current_conversion==0 || max_current==0){
      ERR("No max current set");
      return -2;
    }


    if((ret=send_command("PRG C",5000,0))<0)return ret;
   

   
    if((ret=setCurrentSensibility(1))<0)return ret;
    if((ret=setThreashold(1,255,1000)<0))return ret;
    //setVoltageSensibility(1.0);
    usleep(500000);

    DPRINT("Initialisation returned %d",ret);
    /** setting channels to maximum sensibility*/
    /**setting */
   /*
      setChannel(0,0,0,(1<<current_adc)-1,10000); // current
    usleep(500000);
    setChannel(0,1,0,(1<<voltage_adc)-1,10000); // voltage
    usleep(500000);
    setChannel(1,0,0,(1<<current_adc)-1,10000); // set point corrente
    usleep(500000);
    setChannel(1,1,0,(1<<current_ramp_adc)-1,10000); // ramp up
    usleep(500000);
    setChannel(1,2,0,(1<<current_ramp_adc)-1,10000); // ramp down
    usleep(500000);
    */
        
    /*
    // check parameters
    getChannel(0,0,&min,&max,10000);
    if(min!=0 || max!=((1<<current_adc)-1)){
        ERR("## failed to set current input channel sensibility now min 0x%x max 0x%x",min,max);
        return POWER_SUPPLY_ERROR_SETTING_CHANNEL_SENSIBILITY;
    }
    
    getChannel(0,1,&min,&max,10000);
    if(min!=0 || max!=((1<<voltage_adc)-1)){
        ERR("## failed to set voltage input channel sensibility now min 0x%x max 0x%x",min,max);
        return POWER_SUPPLY_ERROR_SETTING_CHANNEL_SENSIBILITY;
    }
    
    getChannel(1,0,&min,&max,10000);
    if(min!=0 || max!=((1<<current_adc)-1)){
        ERR("## failed to set SP output channel sensibility now min 0x%x max 0x%x",min,max);
        return POWER_SUPPLY_ERROR_SETTING_CHANNEL_SENSIBILITY;
    }
    
    getChannel(1,1,&min,&max,10000);
    if(min!=0 || max!=((1<<current_ramp_adc)-1)){
        ERR("## failed to set RAMP UP output channel sensibility now min 0x%x max 0x%x",min,max);
        return POWER_SUPPLY_ERROR_SETTING_CHANNEL_SENSIBILITY;
    }
    getChannel(1,2,&min,&max,10000);
    if(min!=0 || max!=((1<<current_ramp_adc)-1)){
        ERR("## failed to set RAMP DOWN output channel sensibility now min 0x%x max 0x%x",min,max);
        return POWER_SUPPLY_ERROR_SETTING_CHANNEL_SENSIBILITY;
    }
    */
    
    return ret;
}
int OcemE642X::force_update(uint32_t timeout){
    DPRINT("forcing update timeout %d",timeout);
    GET_VALUE(version,timeout,RMT);
    
    return 0;
}
int OcemE642X::deinit(){
    int* ret;
    
    DPRINT("[%d] DEINITIALIZING",slave_id);
    if(run){
        run=0;
        DPRINT("[%d] STOPPING THREAD",slave_id);
        pthread_join(rpid,(void**)&ret);
    }
    ocem_prot->unRegisterSlave(slave_id);

    ocem_prot.reset();
    removeOcemProtocol(dev);
    initialized=0;
    return 0;
}



// return the number of characters sucessfully written or an error
int OcemE642X::send_command(char*cmd,uint32_t timeout,int*tim){
    char command[1024];
    int ret;
    if(cmd==NULL)
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    if(tim)
        *tim=0;
    DPRINT("sending command \"%s\" to slave %d, timeout %d",cmd,slave_id,timeout);
    snprintf(command,sizeof(command),"%s" OCEM_TERMINATOR,cmd);
    ret =ocem_prot->select(slave_id, command,timeout,tim);
    if(ret>0){
      DPRINT("command sent %d bytes to slave %d sucessfully",ret,slave_id);
    } else if(ret == 0){
        DPRINT("nothing has been sent");
    } else {
      ERR("error %d occurred slave id %d",ret,slave_id);
        
    }
    
    return ret;
}
// return the number of characters sucessfully read or an error
int OcemE642X::send_receive(char*cmd,char*buf,int size,uint32_t timeos,uint32_t timeop,int *timeo){
    int ret;
    uint64_t tstart;
    uint64_t totPollTime=0;
    DPRINT("sending command timeout set to %u ms",timeos);
    ret=send_command(cmd,timeos,timeo);
    
    if(ret>0){
        DPRINT("command returned %d, check timeout set to %u ms",ret,timeop);
        do{
            tstart= common::debug::getUsTime();
            ret = receive_data( buf, size,timeop,timeo);
            totPollTime+= common::debug::getUsTime()-tstart;
            DPRINT("checking result %d, tot Poll time %.10llu us, polling tim %.10llu ms max poll time %u ms, timeout %d",ret,totPollTime,(totPollTime/1000),timeop,*timeo);
        } while((ret<=0)&& (*timeo==0)&& (totPollTime/1000)<(unsigned)timeop);
    }
    return ret;
}
// return the number of data sucessfully read or an error
int OcemE642X::receive_data(char*buf, int size,uint32_t timeout,int *timeo){
    int ret;
    int data_read=0;
    if(buf==NULL) return -1;

    ret=ocem_prot->poll(slave_id, buf, size,timeout,timeo);
    if(ret>0){
      DPRINT("received %d bytes from %d (timeout %d) \"%s\"",ret,slave_id,timeout,buf);
        data_read=updateInternalData(buf);
    } else if(ret == 0){
      DPRINT("nothing received from slave id %d, timeout %d",slave_id,timeout);
        return 0;
    } else if(ret == common::serial::ocem::OcemProtocol::OCEM_NO_TRAFFIC){
        DPRINT("[%d] NO DATA",slave_id);
        return 0;
    } else {
      ERR("error %d occurred, slave id %d (timeout %d)",ret,slave_id,timeout);
      return ret;
    }
    
    return data_read;
}

int OcemE642X::getSWVersion(std::string &ver,uint32_t timeo_ms){
    char stringa[1024];
    int ret;
    if(regulator_state==REGULATOR_UKN){
        ver ="Driver:OcemE642x";
        return 0;
    }
#if 0 
    if(version.mod_time()<=0){
      if((ret=update_status(&version,(char*)"VER",timeo_ms))<=0){
        ERR("data is not in sync");
        return ret;
      }
    }
#endif
    sprintf(stringa,"Driver:OcemE642x, HW Release '%c' %2.2d/%2.2d/%2.2d type:%.8s",version.get().rilascio,version.get().giorno,version.get().mese,version.get().anno, version.get().type);
    
    ver = stringa;
    return 0;
}

int OcemE642X::getHWVersion(std::string&version,uint32_t timeo_ms){
    
    return getSWVersion(version,timeo_ms);
}


int OcemE642X::setPolarity(int pol,uint32_t timeo_ms){
  DPRINT("[%d] Set polarity %d",slave_id,pol);
    if(pol>0){
        CMD_WRITE_AND_CHECK(polarity,timeo_ms,"POS","POL",POL_POS);
    } else if(pol<0){
        CMD_WRITE_AND_CHECK(polarity,timeo_ms,"NEG","POL",POL_NEG);
    } else if(pol==0){
        CMD_WRITE_AND_CHECK(polarity,timeo_ms,"OPN","POL",POL_ZERO);
    }
    return 0;
}

int OcemE642X::getPolarity(int* pol,uint32_t timeo_ms){

  #if 0
  *pol = (polarity == POL_POS)?1:(polarity == POL_NEG)?-1:0;
  GET_VALUE(polarity,timeo_ms,POL);
  *pol = (polarity == POL_POS)?1:(polarity == POL_NEG)?-1:0;
#else
  //CMD_WRITE("POL",timeo_ms);
   
  // GET_VALUE(polarity,timeo_ms,SL);

  *pol = (polarity == POL_POS)?1:(polarity == POL_NEG)?-1:0;
#endif
  return 0;
}

int OcemE642X::setCurrentSP(float current,uint32_t timeo_ms){
    char stringa[256];
    int val;
    if(current<min_current || current>max_current)
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    
    val =(int)round(current/adc_current_conversion);
    snprintf(stringa,sizeof(stringa),"SP %.7d",val);
    printf("-> %.7d (0x%x)",val,val); 
    sp_current = current/adc_current_conversion;
    
    CMD_WRITE(stringa,timeo_ms);
    return 0;
}


int OcemE642X::getCurrentSP(float* current,uint32_t timeo_ms){
    *current = sp_current*adc_current_conversion;
    return 0;
}

int  OcemE642X::getCurrentOutput(float* curr,uint32_t timeo_ms){
#if 0
    *curr = current*adc_current_conversion;
    GET_VALUE(current,timeo_ms,COR);
    *curr = current*adc_current_conversion;
#else
   /* if((common::debug::getUsTime()-current.mod_time())>OCEM_REFRESH_TIME){
        CMD_WRITE("COR",timeo_ms);
    }*/
    
  //  GET_VALUE(current,timeo_ms,SA);
    
    *curr = current*adc_current_conversion;
#endif
    return 0;
}
int  OcemE642X::getVoltageOutput(float* volt,uint32_t timeo_ms){
    //*volt = voltage*adc_current_conversion;
    //GET_VALUE(voltage,timeo_ms,SA);
    

    *volt = voltage*adc_current_conversion;
    return 0;
}

int OcemE642X::startCurrentRamp(uint32_t timeo_ms){
    CMD_WRITE("STR",timeo_ms);
    return 0;
}



int OcemE642X::setCurrentRampSpeed(float asup,float asdown,uint32_t timeo_ms){
    int rsup,rsdown;
    char stringa[256];
    float sensibility=((max_current/10.0 -1.0)*1.0)/(1<<current_ramp_adc);
    if(asup<1.0 || asup > (max_current/10.0)){
      ERR("bad input parameters asup %f",asup);
      return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    if(asdown<1.0 || asdown > (max_current/10.0)){
      ERR("bad input parameters asdown %f",asdown);
      return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    rsup = asup/sensibility;

    rsdown = asdown/sensibility;
    sprintf(stringa,"VRUP %.7d",rsup);
    DPRINT("setting ramp rising speed to %f as (0x%x), falling speed to %f (0x%x)",asup,rsup,asdown,rsdown);
    CMD_WRITE(stringa,timeo_ms);
    sprintf(stringa,"VRDW %.7d",rsdown);
    CMD_WRITE(stringa,timeo_ms);
   
    return 0;
}



int OcemE642X::resetAlarms(uint64_t alrm,uint32_t timeo_ms){
    CMD_WRITE("RES",timeo_ms);
    return 0;
}

int OcemE642X::getAlarms(uint64_t*alrm,uint32_t timeo_ms){
   // *alrm = alarms;
   // GET_VALUE(alarms,timeo_ms,SL);
     

    *alrm = alarms;
    return 0;
}


int OcemE642X::shutdown(uint32_t timeo_ms){
    regulator_state = REGULATOR_SHOULD_BE_OFF;
    // because we lose control once done this command
    DPRINT("SHUTDOWN command, we are going to lose control!!");
    CMD_WRITE("OFF",timeo_ms);
    return 0;
}
int OcemE642X::poweron(uint32_t timeo_ms){
    CMD_WRITE_AND_CHECK(regulator_state,timeo_ms,"ON","SEL",REGULATOR_ON);
    return 0;
}

int OcemE642X::standby(uint32_t timeo_ms){
    CMD_WRITE_AND_CHECK(regulator_state,timeo_ms,"STB","SEL",REGULATOR_STANDBY);
    return 0;
}


int  OcemE642X::getState(int* state,std::string &desc,uint32_t timeo_ms){
  *state = 0;
  
//  GET_VALUE(regulator_state,timeo_ms,SL);
      


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
  
  if(selector_state == SELECTOR_LOC){
        *state|=POWER_SUPPLY_STATE_LOCAL;
        desc += "local ";
    }
    if(alarms!=0){
        *state |= POWER_SUPPLY_STATE_ALARM;
        desc += "Alarm ";
    }
    return 0;
}

int OcemE642X::setChannel(int inout,int number, int min,int max,uint32_t timeo_ms){
    char chtype;
    char um[3];
    char cmd[256];
    int ret;
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
    CMD_WRITE(cmd,timeo_ms);
    return 0;
    
}
int OcemE642X::getChannel(int inout,int number, int* min,int* max,uint32_t timeout){
    
    uint64_t stamp=common::debug::getUsTime();
    if(!(min && max)){
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    //  send_receive((char*)"PRG S",buf,sizeof(buf));
    //
    CMD_WRITE("PRG S",timeout);
    
    if((inout==0) && (number < OCEM_INPUT_CHANNELS )) {
        DPRINT("get channel INPUT number %d, timeo %d",number,timeout);

        *min = ichannel[number].get().min;
        *max = ichannel[number].get().max;
        if(ichannel[number].mod_time()> stamp){
            DPRINT("got input channel %d update %.7d %.7d",number,*min,*max);
            return 0;
        } else {
            ERR("data is not available yet");
            return POWER_SUPPLY_READOUT_OUTDATED;
        }
    } else if((inout==1) && (number < OCEM_OUTPUT_CHANNELS )) {
        DPRINT("get channel OUTPUT number %d, timeo %d",number,timeout);
        *min = ochannel[number].get().min;
        *max = ochannel[number].get().max;
        if(ochannel[number].mod_time()> stamp){
            DPRINT("got output channel %d update %.7d %.7d",number,*min,*max);
            return 0;
        } else {
            ERR("data is not available yet");
            return POWER_SUPPLY_READOUT_OUTDATED;
        }
    } else {
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    
    return 0;
}
int OcemE642X::setThreashold(int channel,int val,uint32_t timeout){
    char cmd[256];
    int ret;
    if(channel>=4){
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }

    snprintf(cmd,sizeof(cmd),"TH I%d %.5d\n",channel,val);
    CMD_WRITE(cmd,timeout);
    
    return 0;
}
int OcemE642X::setThreashold(int channel,float value,uint32_t timeout){
    
    int val;
    if(channel>=4){
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    if(channel==0){
      // current;
      val = value/adc_current_conversion;
    } else if(channel==1){
      val = value/adc_voltage_conversion;
    } else {
      return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }

    return setThreashold(channel,val,timeout);
}


int OcemE642X::getCurrentSensibility(float*sens){
    *sens =current_sensibility;
    return 0;
}

int  OcemE642X::getVoltageSensibility(float *sens){
    *sens = voltage_sensibility;
    return 0;
}

int OcemE642X::getMaxMinCurrent(float*max,float*min){
    *min=min_current;
    *max=max_current;
    return 0;
}
int OcemE642X::getMaxMinVoltage(float*max,float*min){
    *min=min_voltage;
    *max=max_voltage;
    return 0;
}

int OcemE642X::forceMaxCurrent(float max){
    max_current = max;
    adc_current_conversion=((max_current-min_current)*1.0)/(1<<current_adc);
    return 0;
}

int OcemE642X::forceMaxVoltage(float max){
    max_voltage = max;
    adc_voltage_conversion=((max_voltage -min_voltage)*1.0)/(1<<voltage_adc);
    
    return 0;
}


int OcemE642X::getAlarmDesc(uint64_t*desc){
    *desc = POWER_SUPPLY_EVENT_DOOR_OPEN;
    return 0;
}

int OcemE642X::setCurrentSensibility(float sens){
  int ret;
    DPRINT("set current sensibility to %f A",sens);
    current_sensibility=sens;
    ret=setThreashold(0,sens,1000);
    if(ret<0)return ret;
    return 0;
}        
int OcemE642X::setVoltageSensibility(float sens){
  int ret;
    voltage_sensibility=sens;
     DPRINT("set voltage sensibility to %f A",sens);

    ret=setThreashold(1,sens,1000);
    if(ret<0)return ret;
    return 0;
    
}
