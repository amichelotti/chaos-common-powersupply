//
//  OcemCore.cpp
//  PowerSupply
//
//  Created by andrea michelotti on 10/11/13.
//  Copyright (c) 2013 andrea michelotti. All rights reserved.
//

#include "OcemCore.h"
#include "common/debug/debug.h"
using namespace common::powersupply;

pthread_mutex_t OcemCore::unique_ocem_core_mutex=PTHREAD_MUTEX_INITIALIZER;
std::map<std::string,OcemProtocol_psh > OcemCore::unique_protocol;

void OcemCore::removeOcemProtocol(const char*dev){
    std::string mydev(dev);
    pthread_mutex_lock(&unique_ocem_core_mutex);
    std::map<std::string,OcemProtocol_psh >::iterator i=unique_protocol.find(mydev);
    unique_protocol.erase(i);
    pthread_mutex_unlock(&unique_ocem_core_mutex);
}
OcemProtocol_psh OcemCore::getOcemProtocol(const char *dev,int baudrate,int parity,int bits,int stop){
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


OcemCore::OcemCore(const char *_dev,int _slave_id,int _baudrate,int _parity,int _bits,int _stop): dev(_dev),baudrate(_baudrate),parity(_parity),bits(_bits),stop(_stop),slave_id(_slave_id)
{
    ocem_prot = getOcemProtocol(dev,baudrate,parity,bits,stop);
    state = UNKNOWN;
}

OcemCore::~OcemCore(){
    deinit();
}

int OcemCore::update_status(int timeout,int msxpoll){
  uint64_t tstart;
  uint64_t totPollTime=0;			
  char buf[2048];
  int retry=0;
  int ret;
  do{
    tstart= common::debug::getUsTime();
    ret = check_data( buf, sizeof(buf),timeout);
    totPollTime+= common::debug::getUsTime()-tstart;
    DPRINT("retry %d checking result %d, tot Poll time %.10llu us\n",retry++,ret,totPollTime);
    usleep(msxpoll*1000);
  } while((ret==common::serial::OcemProtocol::OCEM_NO_TRAFFIC)&& (totPollTime/1000)<timeout);
  return ret;
}


int OcemCore::init(){
    int ret=-1;
    char buf[2048];
    *buf = 0;
    state = UNKNOWN;
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
      int rett;
      // poll trial
      update_status(1000,100);
      sleep(1);
      if((rett=send_command((char*)"RMT",1000))<=0){
	DERR("RMT command failed %d \n",rett);
	return rett;
      }
      sleep(1);
      
      // poll trial
      DPRINT(" poll\n");
      if(check_data(buf,2048,1000)>0){
	DPRINT("Poll returned %s\n",buf);
      }
      sleep(1);
      DPRINT(" poll\n");
      if(check_data(buf,2048,1000)>0){
	DPRINT("Poll returned %s\n",buf);
      }

      if((rett=send_receive((char*)"VER",buf,2048,1000))<=0){
	DERR("VER command failed %d \n",rett);
	return rett;
      }
      DPRINT("protocol correctly set answer is:\"%s\"\n",buf);
      if((rett=send_receive((char*)"RMT",buf,2048,1000))<=0){
	DERR("RMT command failed %d \n",rett);
	return rett;
      }
      
      DPRINT("protocol correctly set answer is:\"%s\"\n",buf);
    }
    
    return ret;
}

int OcemCore::deinit(){
    ocem_prot.reset();
    removeOcemProtocol(dev);
    
    return 0;
}



// return the number of characters sucessfully written or an error
int OcemCore::send_command(char*cmd,int timeout){
    char command[1024];
    int tim=0,ret;
    if(cmd==NULL) return -1;
    DPRINT("sending command \"%s\" to slave %d\n",cmd,slave_id);
    snprintf(command,sizeof(command),"%s" OCEM_TERMINATOR,cmd);
    ret =ocem_prot->select(slave_id, command,timeout,&tim);
    if(ret>0){
        DPRINT("command sent %d bytes (timeout %d) sucessfully \n",ret,tim);
    } else if(ret == 0){
        DPRINT("nothing has been sent, timeout %d\n",tim);
    } else {
        DERR("error %d occurred (timeout %d)\n",ret,tim);
        
    }

    return ret;
}
// return the number of characters sucessfully read or an error
int OcemCore::send_receive(char*cmd,char*buf,int size,int timeos,int timeop){
  int ret,retry=0;
  uint64_t tstart;
  uint64_t totPollTime=0;
  DPRINT("sending command\n");
  ret=send_command(cmd,timeos);
 
  if(ret>0){
    DPRINT("command returned %d, checking...\n",ret);
    do{
      tstart= common::debug::getUsTime();
      ret = check_data( buf, size,timeop);
      totPollTime+= common::debug::getUsTime()-tstart;
      DPRINT("retry %d checking result %d, tot Poll time %.10llu us\n",retry++,ret,totPollTime);
      usleep(100000);
    } while((ret==common::serial::OcemProtocol::OCEM_NO_TRAFFIC)&& (totPollTime/1000)<timeop);
  } 
  return ret;
}
// return the number of characters sucessfully read or an error
int OcemCore::check_data(char*buf, int size,int timeout){
    int ret,tim=0;
    if(buf==NULL) return -1;
    DPRINT("wait for messages from slave %d\n",slave_id);
    ret=ocem_prot->poll(slave_id, buf, size,timeout,&tim);
    if(ret>0){
        DPRINT("received %d bytes (timeout %d) \"%s\"\n",ret,tim,buf);
    } else if(ret == 0){
        DPRINT("nothing received, timeout %d\n",tim);
    } else {
        DERR("error %d occurred (timeout %d)\n",ret,tim);

    }
    
    return ret;
}

std::string OcemCore::getSWVersion(){
    char version[1024];
    if(send_receive((char*)"VER",version,sizeof(version))>0){
        return std::string(version);
    }
    return std::string();
}

std::string OcemCore::getHWVersion(){
    return getHWVersion();
}


int OcemCore::setCurrentPolarity(int pol){
    return 0;
}

 int OcemCore::getCurrentPolarity(int* pol){
     return 0;
 }

 int OcemCore::setCurrentPoint(float current){
     return 0;
 }


 int OcemCore::getCurrentPoint(float* current){
     return 0;
 }

 int OcemCore::startCurrentRamp(){
     return 0;
 }


 int OcemCore::setVoltagePoint(float volt){
     return 0;
 }



 int OcemCore::getVoltagePoint(float* current){
     return 0;
 }


 int OcemCore::startVoltageRamp(){
     return 0;
 }

 int OcemCore::setCurrentRampSpeed(float as){
     return 0;
 }

 int OcemCore::setVoltageRampSpeed(float vs){
     return 0;
 }


 int OcemCore::setCurrentThreashold(float maxcurr,float mincurr){
     return 0;
 }

 int OcemCore::setVoltageThreashold(float maxvolt,float minvolt){
     return 0;
 }


 int OcemCore::getCurrentThreashold(float *maxcurr,float *mincurr){
     return 0;
 }


 int OcemCore::getVoltageThreashold(float *maxvolt,float *minvolt){
     return 0;
 }

 int OcemCore::resetAlarms(uint64_t alrm){
     return 0;
 }

 int OcemCore::getAlarms(uint64_t*alrm){
     return 0;
 }


 int OcemCore::shutdown(){
     return 0;
 }


 int OcemCore::standby(){
     return 0;
 }

