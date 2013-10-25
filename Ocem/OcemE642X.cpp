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

#define CMD_WRITE(_cmdw,_timeo) \
{									\
int _ret,_timeout=0;							\
DPRINT("CMD_WRITE apply command \"%s\" timeout %d\n",_cmdw,_timeo);			\
_ret =send_command((char*)_cmdw,_timeo,&_timeout);			\
if(_timeout==1) return POWER_SUPPLY_TIMEOUT;				\
if(_ret<=0) return POWER_SUPPLY_COMMAND_ERROR;			\
}

#ifdef OCEM_CHECK_ENABLED
#define CMD_WRITE_AND_CHECK(value,_timeo,cmdw,cmdr,cmdv) \
{									\
int _ret;								\
DPRINT("apply command \"" cmdw "\" and verify with \"" cmdr "\", expected \"" #cmdv "\" [%d]\n",(unsigned)cmdv); \
_ret =send_command((char*)cmdw,_timeo);					\
if(_ret>0){					\
if((_ret=update_status(&value,(char*)cmdr,_timeo))>0){	\
if(value.get()==cmdv){			\
DPRINT("Command sucessfully applied\n");	\
return 0;					\
} else {								\
DERR("Value %d != %d \"" #cmdv "\"\n",value.get(),cmdv);		\
return POWER_SUPPLY_READBACK_FAILED;				\
}									\
} else {								\
DERR("data is not in sync\n");					\
return _ret;								\
}									\
}									\
DERR("error sending command\n");					\
return POWER_SUPPLY_COMMAND_ERROR;					\
}
#else
#define CMD_WRITE_AND_CHECK(value,_timeo,cmdw,cmdr,cmdv)			\
CMD_WRITE(cmdw,_timeo)
#endif

#define GET_VALUE(value,_timeo,cmdw) \
{									\
int _ret;								\
DPRINT("apply command \"%s\" timeout %d\n",cmdw,_timeo);		\
if((_ret=update_status((common::debug::basic_timed*)&value,(char*)cmdw,_timeo))<=0){ \
DERR("data is not in sync, ret %d\n",_ret);				\
return _ret;							\
}									\
}

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

int OcemE642X::update_status(common::debug::basic_timed*data ,char *cmd,uint32_t timeout){
    
    char buf[2048];
    uint64_t stamp=common::debug::getUsTime();
    int tim=0;
    uint64_t ret_time;
    timeout*=1000;
    CMD_WRITE(cmd,timeout);
    while(((ret_time=data->mod_time())<stamp)){
        if(((timeout>0) && ((common::debug::getUsTime()-stamp)>=timeout)) || (tim > 0)){
            DPRINT("Timeout reading data type \"%s\" Start operation %10llu (duration %10llu) > %d timeout\n",data->get_name(),stamp,(common::debug::getUsTime()-stamp)/1000,timeout/1000);
            tim++;
            break;
        }
        DPRINT("data type \"%s\" last update at %10llu, command issued at %10llu, timeout occur:%d\n",data->get_name(),ret_time,stamp, tim);
        receive_data( buf, sizeof(buf),timeout,&tim);
    }
    
    if(data->mod_time()>stamp){
        DPRINT("data type \"%s\" has been updated at %10llu\n",data->get_name(),data->mod_time());
        return 1;
    } else {
        if(tim>0){
            DERR("Timeout retriving data \n");
            return POWER_SUPPLY_TIMEOUT;
        }
    }
    DERR("Data is not updated yet\n");
    return POWER_SUPPLY_READOUT_OUTDATED;
}


OcemE642X::OcemE642X(const char *_dev,int _slave_id,int _baudrate,int _parity,int _bits,int _stop): dev(_dev),baudrate(_baudrate),parity(_parity),bits(_bits),stop(_stop),slave_id(_slave_id)
{
    int cnt;
    ocem_prot = getOcemProtocol(dev,baudrate,parity,bits,stop);
    
    INITIALIZE_TIMED(regulator_state,REGULATOR_UKN);
    INITIALIZE_TIMED(selector_state,SELECTOR_UKN);
    INITIALIZE_TIMED(polarity, POL_UKN);
    INITIALIZE_TIMED(current, 0U);
    INITIALIZE_TIMED(voltage, 0U);
    INITIALIZE_TIMED(sp_current, 0U);
    INITIALIZE_TIMED(alarms, 0ULL);
    INITIALIZE_TIMED(version, ocem_version());
    voltage_sensibility=((max_voltage -min_voltage)*1.0)/(1<<voltage_adc);
    current_sensibility = ((max_current-min_current)*1.0)/(1<<current_adc);
    for(cnt=0;cnt<OCEM_INPUT_CHANNELS;cnt++){
        INITIALIZE_TIMED(ichannel[cnt],ocem_channel(0,0));
    }
    for(cnt=0;cnt<OCEM_OUTPUT_CHANNELS;cnt++){
        INITIALIZE_TIMED(ochannel[cnt],ocem_channel(0,0));
    }
    
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
            alarms = 0ULL;
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

int OcemE642X::update_status(uint32_t timeout,int msxpoll){
    uint64_t tstart;
    uint64_t totPollTime=0;
    char buf[2048];
    int retry=0;
    int ret;
    int updated=0;
    int timeo=0;
    timeout*=1000;
    do{
        tstart= common::debug::getUsTime();
        ret = receive_data( buf, sizeof(buf),timeout,&timeo);
        totPollTime+= common::debug::getUsTime()-tstart;
        DPRINT("Check returned %d retry %d checking result %d, tot Poll time %.10llu us\n",ret,retry++,ret,totPollTime);
        if(ret>0){
            updated+=ret;
        }
        if(msxpoll)
            usleep(msxpoll*1000);
    } while((ret>0)&& ((totPollTime)<(unsigned)timeout) && (timeo==0));
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
        if((ret=force_update(10000))<0){
            DERR("Nothing has been refreshed  %d\n",ret);
            return ret;
            
        } else {
            DPRINT("Ocem status has been refreshed %d\n",ret);
            ret = 0;
        }
        
    }
    
    return ret;
}
int OcemE642X::force_update(uint32_t timeout){
    DPRINT("forcing update timeout %d\n",timeout);
    GET_VALUE(regulator_state,timeout,"RMT");
    
    return 0;
}
int OcemE642X::deinit(){
    ocem_prot.reset();
    removeOcemProtocol(dev);
    
    return 0;
}



// return the number of characters sucessfully written or an error
int OcemE642X::send_command(char*cmd,uint32_t timeout,int*tim){
    char command[1024];
    int ret;
    if(cmd==NULL || tim==NULL)
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    *tim=0;
    DPRINT("sending command \"%s\" to slave %d, timeout %d\n",cmd,slave_id,timeout);
    snprintf(command,sizeof(command),"%s" OCEM_TERMINATOR,cmd);
    ret =ocem_prot->select(slave_id, command,timeout,tim);
    if(ret>0){
        DPRINT("command sent %d bytes (timeout %d, occured %d) sucessfully \n",ret,timeout,*tim);
    } else if(ret == 0){
        DPRINT("nothing has been sent, (timeout %d, occured %d)\n",timeout,*tim);
    } else {
        DERR("error %d occurred (timeout %d, occured %d)\n",ret,timeout,*tim);
        
    }
    
    return ret;
}
// return the number of characters sucessfully read or an error
int OcemE642X::send_receive(char*cmd,char*buf,int size,uint32_t timeos,uint32_t timeop,int *timeo){
    int ret,retry=0;
    uint64_t tstart;
    uint64_t totPollTime=0;
    DPRINT("sending command timeout set to %u ms\n",timeos);
    ret=send_command(cmd,timeos,timeo);
    
    if(ret>0){
        DPRINT("command returned %d, check timeout set to %u ms\n",ret,timeop);
        do{
            tstart= common::debug::getUsTime();
            ret = receive_data( buf, size,timeop,timeo);
            totPollTime+= common::debug::getUsTime()-tstart;
            DPRINT("retry %d checking result %d, tot Poll time %.10llu us, polling tim %.10llu ms max poll time %u ms, timeout %d\n",retry++,ret,totPollTime,(totPollTime/1000),timeop,*timeo);
        } while((ret<=0)&& (*timeo==0)&& (totPollTime/1000)<(unsigned)timeop);
    }
    return ret;
}
// return the number of data sucessfully read or an error
int OcemE642X::receive_data(char*buf, int size,uint32_t timeout,int *timeo){
    int ret;
    int data_read=0;
    if(buf==NULL) return -1;
    DPRINT("wait for messages from slave %d\n",slave_id);
    ret=ocem_prot->poll(slave_id, buf, size,timeout,timeo);
    if(ret>0){
        DPRINT("received %d bytes (timeout %d) \"%s\"\n",ret,timeout,buf);
        data_read=updateInternalData(buf);
    } else if(ret == 0){
        DPRINT("nothing received, timeout %d (timeout arised %d)\n",timeout,*timeo);
        return 0;
    } else {
        DERR("error %d occurred (timeout %d) (timeout arised %d)\n",ret,timeout,*timeo);
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
    
    if((ret=update_status(&version,(char*)"VER",timeo_ms))<=0){
        DERR("data is not in sync\n");
        return ret;
    }
    
    sprintf(stringa,"Driver:OcemE642x, HW Release '%c' %.2d/%.2d/%.2d type:%s\n",version.get().rilascio,version.get().giorno,version.get().mese,version.get().anno, version.get().type);
    
    ver = stringa;
    return 0;
}

int OcemE642X::getHWVersion(std::string&version,uint32_t timeo_ms){
    return getHWVersion(version,timeo_ms);
}


int OcemE642X::setPolarity(int pol,uint32_t timeo_ms){
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
    *pol = polarity;
    GET_VALUE(polarity,timeo_ms,"POL");
    *pol = polarity;
    return 0;
}

int OcemE642X::setCurrentSP(float current,uint32_t timeo_ms){
    char stringa[256];
    
    if(current<min_current || current>max_current)
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    
    snprintf(stringa,sizeof(stringa),"SP %d",(int)(current/current_sensibility));
    
    sp_current = current/current_sensibility;
    
    CMD_WRITE(stringa,timeo_ms);
    return 0;
}


int OcemE642X::getCurrentSP(float* current,uint32_t timeo_ms){
    *current = sp_current*current_sensibility;
    return 0;
}

int  OcemE642X::getCurrentOutput(float* curr,uint32_t timeo_ms){
    *curr = current*current_sensibility;
    GET_VALUE(current,timeo_ms,"COR");
    *curr = current*current_sensibility;
    return 0;
}
int  OcemE642X::getVoltageOutput(float* volt,uint32_t timeo_ms){
    *volt = voltage*current_sensibility;
    GET_VALUE(voltage,timeo_ms,"TEN");
    *volt = voltage*current_sensibility;
    return 0;
}

int OcemE642X::startCurrentRamp(uint32_t timeo_ms){
    CMD_WRITE("STR",timeo_ms);
    return 0;
}



int OcemE642X::setCurrentRampSpeed(float asup,float asdown,uint32_t timeo_ms){
    int rsup,rsdown,ret;
    if(asup<OCEM_MIN_VOLTAGE || asup > OCEM_MAX_VOLTAGE)
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    
    if(asdown<OCEM_MIN_VOLTAGE || asdown > OCEM_MAX_VOLTAGE)
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    rsup=(asup/OCEM_MAX_VOLTAGE)*(1<<OCEM_CURRENT_RAMP_ADC);
    rsdown=(asdown/OCEM_MAX_VOLTAGE)*(1<<OCEM_CURRENT_RAMP_ADC);
    DPRINT("setting ramp rising speed to %f as (0x%x), falling speed to %f (0x%x)\n",asup,rsup,asdown,rsdown);
    if((ret=setChannel(CHANNEL_OUT,CHANNEL_RAMP_UP,0,rsup,timeo_ms))<0){
        DERR("error setting current RAMP UP \n");
        return ret;
    }
    if((ret=setChannel(CHANNEL_OUT,CHANNEL_RAMP_DOWN,0,rsdown,timeo_ms))<0){
        DERR("error setting current RAMP DOWN \n");
        return ret;
    }
    return 0;
}



int OcemE642X::resetAlarms(uint64_t alrm,uint32_t timeo_ms){
    CMD_WRITE("RES",timeo_ms);
    return 0;
}

int OcemE642X::getAlarms(uint64_t*alrm,uint32_t timeo_ms){
    *alrm = alarms;
    GET_VALUE(alarms,timeo_ms,"ALL");
    *alrm = alarms;
    return 0;
}


int OcemE642X::shutdown(uint32_t timeo_ms){
    regulator_state = REGULATOR_SHOULD_BE_OFF;
    // because we lose control once done this command
    DPRINT("SHUTDOWN command, we are going to lose control!!\n");
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
    GET_VALUE(regulator_state,timeo_ms,"SL");
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

int OcemE642X::setChannel(int inout,int number, int min,int max,uint32_t timeo_ms){
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
    CMD_WRITE(cmd,timeo_ms);
    return 0;
    
}
int OcemE642X::getChannel(int inout,int number, int* min,int* max,uint32_t timeout){
    
    uint64_t stamp=common::debug::getUsTime();
    if(!(min && max)){
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    //  send_receive((char*)"PRG S",buf,sizeof(buf));
    //update_status(timeout,10);
    if((inout==0) && (number < OCEM_INPUT_CHANNELS )) {
        GET_VALUE(ichannel[number],timeout,"PRG S");
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
        GET_VALUE(ochannel[number],timeout,"PRG S");
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
int OcemE642X::setThreashold(int channel,int value,uint32_t timeout){
    char cmd[256];
    if(channel>=4){
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    snprintf(cmd,sizeof(cmd),"TH I%d %d\n",channel,value);
    CMD_WRITE(cmd,timeout);
    return 0;
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

int OcemE642X::getAlarmDesc(uint64_t*desc){
    *desc = POWER_SUPPLY_EVENT_DOOR_OPEN;
    return 0;
}			
