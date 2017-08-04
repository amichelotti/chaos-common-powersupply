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
	DPRINT("[%s,%d] CMD_WRITE apply command \"%s\" timeout %d",dev.c_str(),slave_id,_cmdw,_timeo);			\
	_ret =send_command((char*)_cmdw,_timeo,&_timeout);	\
	if(_timeout==1) {DERR("[%s,%d] timeout sending command %s",dev.c_str(),slave_id,_cmdw);return POWER_SUPPLY_TIMEOUT;};				\
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
			DPRINT("[%s,%d] NOT APPLY (too near %lu ago) command \"%s\" timeout %d",dev.c_str(),slave_id, last,# cmdw,_timeo);	\
			update_status(500,10);}					\
			else{DPRINT("[%s,%d] apply command \"%s\" last command %lu us ago timeout %d",dev.c_str(),slave_id, # cmdw,last,_timeo); cmdw=2; \
			if((_ret=update_status((common::debug::basic_timed*)&value,(char*)# cmdw,_timeo))<=0){ \
				DERR("[%s,%d] cmd \"%s\" data is not in sync, ret %d",dev.c_str(),slave_id,# cmdw,_ret);		\
				return _ret;							\
			}}								\
}



void* OcemE642X::update_thread(void* p){
	OcemE642X* pnt = (OcemE642X*)p;
	return (void*)pnt->updateSchedule();
}
void* OcemE642X::updateSchedule(){
	int ret;
	float inst_curr=0;
	char buf[2048];
	int s=0;
	DPRINT("[%s,%d] OCEM  STARTED 0x%p ",dev.c_str(),slave_id,(void*)pthread_self());
	run=1;
	while(run){
		//	DPRINT("[%s,%d] SLAVE SCHEDULE",dev.c_str(),slave_id);
		ret = receive_data( buf, sizeof(buf),1000,0);
		if(start_ramp && (start_ramp.mod_time()>OCEM_REFRESH_TIME)){
			float inst_delta=fabs(current_sp-inst_curr);
			send_command((char*)"COR",1000,0);
			if(inst_delta < delta_current_sp){
				DPRINT("[%s,%d] CHECK CURRENT RAMP (mod time %lld us ago) current sp %f, inst curr %f, final delta current:%f inst delta curr %f, COMMAND STARTED OK, retry command %d, retry check %d.",dev.c_str(),slave_id,start_ramp.mod_time(),current_sp,inst_curr,delta_current_sp,inst_delta,retry_current,try_check_current);
				start_ramp =0;

				delta_current_sp=0;
			} else {
				DPRINT("[%s,%d] CHECK CURRENT RAMP (mod time %lld us ago) current sp %f, inst curr %f, final delta current:%f,inst delta curr %f, retry command %d, retry check %d.",dev.c_str(),slave_id,start_ramp.mod_time(),current_sp,inst_curr,delta_current_sp,inst_delta,retry_current,try_check_current);
				start_ramp=1;
				if(try_check_current++<OCEM_TRY_CHECK_COMMAND){
					send_command((char*)"SL",1000,0);
				} else {
					if(retry_current++<OCEM_MAX_COMMAND_RETRY){
						DPRINT("[%s,%d] CHECK CURRENT RE-SUBMIT CURRENT '%f' remaning retry %d",dev.c_str(),slave_id,current_sp,OCEM_MAX_COMMAND_RETRY-retry_current);
						int tmpret=retry_current;
						setCurrentSP(current_sp,1000);
						usleep(200000);
						startCurrentRamp(1000);
						try_check_current=0;
						retry_current=tmpret;
					} else {
						DERR("## CHECK CURRENT cannot set current to %f",current_sp);
						start_ramp=0;

					}
				}
			}

		} else if(start_pol&& (start_pol.mod_time()>OCEM_REFRESH_TIME)){
			//POLARITY
			if(polarity==pol_sp){
				DPRINT("[%s,%d] CHECK POLARITY '%d' SWITCH COMMAND EXECUTED  ",dev.c_str(),slave_id,(int)polarity);
				start_pol=0;
			} else {
				DPRINT("[%s,%d] CHECK POLARITY '%d' SWITCH",dev.c_str(),slave_id,(int)polarity);
				try_check_pol++;
				start_pol=1;

				if(try_check_pol<OCEM_TRY_CHECK_COMMAND){
					send_command((char*)"SL",1000,0);
				} else {

					if(retry_pol++<OCEM_MAX_COMMAND_RETRY){
						int tmpret=retry_pol;

						DPRINT("[%s,%d] CHECK POLARITY RE-SUBMIT POLARITY '%d' remaning retry %d",dev.c_str(),slave_id,pol_sp,OCEM_MAX_COMMAND_RETRY-retry_pol);
						setPolarity(pol_sp,1000);
						retry_pol=tmpret;
					} else {
						DERR("## CHECK POLARITY cannot set polarity to %d",pol_sp);
						start_pol=0;
					}
				}

			}

		} else if(start_state&& (start_state.mod_time()>OCEM_REFRESH_TIME)){
			//STATE ON/STBY
			if(regulator_state==state_sp){
				DPRINT("[%s,%d] CHECK STATE '%d' SWITCH COMMAND EXECUTED  ",dev.c_str(),slave_id,(int)regulator_state);
				start_state=0;
			} else {
				DPRINT("[%s,%d] CHECK STATE '%d' SWITCH",dev.c_str(),slave_id,(int)regulator_state);
				try_check_state++;
				start_state=1;
				if(try_check_state<OCEM_TRY_CHECK_COMMAND){
					send_command((char*)"SL",1000,0);
				} else {
					if(retry_state++<OCEM_MAX_COMMAND_RETRY){
						int tmpret=retry_state;

						if(state_sp==REGULATOR_STANDBY){
							DPRINT("[%s,%d] CHECK STATE  RE-SUBMIT STATE 'STANDBY' remaning retry %d",dev.c_str(),slave_id,OCEM_MAX_COMMAND_RETRY-retry_state);

							standby(1000);
						} else if(state_sp==REGULATOR_ON){
							DPRINT("[%s,%d] CHECK STATE  RE-SUBMIT STATE 'ON' remaning retry %d",dev.c_str(),slave_id,OCEM_MAX_COMMAND_RETRY-retry_state);

							poweron(1000);
						}
						retry_state=tmpret;
					} else {
						DERR("## CHECK STATE cannot set state to %s",state_sp==REGULATOR_STANDBY?"STANDBY":(state_sp==REGULATOR_ON)?"ON":"UKNOWN");
						start_state=0;
					}
				}
			}
		} else {


			uint64_t t=common::debug::getUsTime();
			if(current.mod_time()>OCEM_REFRESH_TIME){
				if((ocem_prot->getWriteSize(slave_id)==0)&&(ocem_prot->getReadSize(slave_id)==0)){
					DPRINT("[%s,%d] REFRESH ANALOGIC, because current has been modified %lu ms ago",dev.c_str(),slave_id,current.mod_time()/1000);
					send_command((char*)"SA",1000,0);
				}
			}
			if(regulator_state.mod_time()>(OCEM_REFRESH_TIME)){
				if((ocem_prot->getWriteSize(slave_id)==0)&&(ocem_prot->getReadSize(slave_id)==0)){
					DPRINT("[%s,%d] REFRESH LOGIC  because state has been modified %lu ms",dev.c_str(),slave_id,regulator_state.mod_time()/1000);
					send_command((char*)"SL",1000,0);
				}
			}
		}
		getCurrentOutput(&inst_curr,1000);
	}

	DPRINT("[%d] EXITING SLAVE UPDATE THREAD",slave_id);
}

void OcemE642X::removeOcemProtocol(std::string& mydev){
	DPRINT(" removing protocol on \"%s\"",mydev.c_str());
	pthread_mutex_lock(&unique_ocem_core_mutex);
	ocem_prot.reset();
	std::map<std::string,OcemProtocol_psh >::iterator i=unique_protocol.find(mydev);
	if(i!=unique_protocol.end()){
		if(i->second.use_count()==1){
			DPRINT("found removing use count %ld",i->second.use_count());
			unique_protocol.erase(i);
		} else {
			DPRINT("still in use cannot remove it use count %ld",i->second.use_count());

		}
	}
	pthread_mutex_unlock(&unique_ocem_core_mutex);
}
/*OcemE642X::OcemProtocol_psh OcemE642X::getOcemProtocol(std::string& mydev,int baudrate,int parity,int bits,int stop){
	DPRINT("getting protocol for \"%s\" baudrate %d , parity %d bits %d stop %d",mydev.c_str(),baudrate,parity,bits,stop);
	pthread_mutex_lock(&unique_ocem_core_mutex);
	std::map<std::string,OcemProtocol_psh >::iterator i=unique_protocol.find(mydev);
	if(i!=unique_protocol.end() && i->second!=NULL){
		pthread_mutex_unlock(&unique_ocem_core_mutex);
		DPRINT("RETRIVING serial ocem protocol on \"%s\" @x%p",mydev.c_str(),(void*)i->second.get());
		return i->second;
	}
	//    unique_protocol[mydev] =OcemProtocol_psh(new common::serial::ocem::OcemProtocol(mydev.c_str(),POSIX_SERIAL_COMM_DEFAULT_MAX_BUFFER_WRITE_SIZE,baudrate,parity,bits,stop));
	unique_protocol[mydev] =OcemProtocol_psh(new common::serial::ocem::OcemProtocolScheduleCFQ(mydev.c_str(),POSIX_SERIAL_COMM_DEFAULT_MAX_BUFFER_WRITE_SIZE,baudrate,parity,bits,stop));
	DPRINT("creating NEW serial ocem protocol on \"%s\" @x%p",mydev.c_str(),(void*)unique_protocol[mydev].get());

	pthread_mutex_unlock(&unique_ocem_core_mutex);

	return unique_protocol[mydev];
}*/
 OcemE642X::OcemProtocol_psh OcemE642X::getOcemProtocol(const std::string& protname,common::misc::driver::AbstractChannel_psh channel){
	DPRINT("getting protocol for \"%s\" and channel \"%s\"",protname.c_str(),channel->getUid().c_str());
	pthread_mutex_lock(&unique_ocem_core_mutex);
	std::map<std::string,OcemProtocol_psh >::iterator i=unique_protocol.find(channel->getUid());
		if(i!=unique_protocol.end() && i->second.get()!=NULL){
			pthread_mutex_unlock(&unique_ocem_core_mutex);
			DPRINT("RETRIVING protocol \"%s\" channel \"%s\" @%p usage count  %ld",protname.c_str(),i->second->getChannel()->getUid().c_str(),(void*)i->second.get(),i->second.use_count());
			return i->second;
		}
		if(protname == "OcemProtocol"){
			unique_protocol[channel->getUid()] =OcemProtocol_psh(new common::serial::ocem::OcemProtocol(channel));

		} else if(protname == "OcemProtocolBuffered"){
			unique_protocol[channel->getUid()] =OcemProtocol_psh(new common::serial::ocem::OcemProtocolBuffered(channel));

		} else if(protname == "OcemProtocolScheduleCFQ"){
			//default is CFQ
			unique_protocol[channel->getUid()] =OcemProtocol_psh(new common::serial::ocem::OcemProtocolScheduleCFQ(channel));

		} else {
			ERR("## exception bad ocem protocol specification");
			throw std::logic_error("bad ocem protocol specification");

		}
		//    unique_protocol[mydev] =OcemProtocol_psh(new common::serial::ocem::OcemProtocol(mydev.c_str(),POSIX_SERIAL_COMM_DEFAULT_MAX_BUFFER_WRITE_SIZE,baudrate,parity,bits,stop));
		DPRINT("creating NEW serial ocem protocol \"%s\" @%p",protname.c_str(),(void*)unique_protocol[channel->getUid()].get());

		pthread_mutex_unlock(&unique_ocem_core_mutex);

		return unique_protocol[channel->getUid()];

}


int OcemE642X::update_status(common::debug::basic_timed*data ,char *cmd,uint32_t timeout){

	//char buf[2048];
	//uint64_t stamp=common::debug::getUsTime();
	if((common::debug::getUsTime()-data->mod_time())>(OCEM_REFRESH_TIME)){
		CMD_WRITE(cmd,timeout);
	}
	update_status(600,200);
	DPRINT("[%s,%d] data type \"%s\" last update at %10lu us ago",dev.c_str(),slave_id,data->get_name(),(common::debug::getUsTime()- data->mod_time()));

	return 1;


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
	INITIALIZE_TIMED(start_ramp, 0U);
	INITIALIZE_TIMED(start_pol, 0U);
	INITIALIZE_TIMED(start_state, 0U);

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
	current_sp=0;
	ramps_sp=0;
	pol_sp=0;
	state_sp=0;
	try_check_current=try_check_pol=try_check_state=0;
	retry_current=retry_pol=retry_state=0;

}



/*OcemE642X::OcemE642X(const char *_dev,int _slave_id,int _baudrate,int _parity,int _bits,int _stop): dev(_dev),baudrate(_baudrate),parity(_parity),bits(_bits),stop(_stop),slave_id(_slave_id)
{

	DPRINT("[%s,%d] CREATE @0x%p",_dev,_slave_id,this);
	ocem_prot = getOcemProtocol(dev,baudrate,parity,bits,stop);
	max_current=0;
	min_current=0;
	max_voltage=0;
	min_voltage=0;
	initialized=0;
	init_internal();

}*/

OcemE642X::OcemE642X(const std::string& protname,common::misc::driver::AbstractChannel_psh channel,int _slave_id,float maxcurr,float maxvoltage,OcemType type):protocol(protname),slave_id(_slave_id),ocem_type(type){

//OcemE642X::OcemE642X(const char *_dev,int _slave_id,float maxcurr,float maxvoltage): dev(_dev),baudrate(9600),parity(0),bits(8),stop(1),slave_id(_slave_id){
	DPRINT("[%s,%d] OcemE642X constructor 0x%p",channel->getUid().c_str(),_slave_id,this);
	initialized=0;
	ocem_prot = getOcemProtocol(protname,channel);
	dev=channel->getUid();
	if(maxcurr>0)
		forceMaxCurrent(maxcurr);
	if(maxvoltage>0)
		forceMaxVoltage(maxvoltage);
	init_internal();
}

OcemE642X::~OcemE642X(){

	deinit();
	DPRINT("[%s,%d] destroy",dev.c_str(),slave_id);
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

				DPRINT("[%s,%d] setting input channel %d at %.10lu us to (%d,%d)",dev.c_str(),slave_id,channelnum,  ichannel[channelnum].mod_time(), ichannel[channelnum].get().min, ichannel[channelnum].get().max);
				parsed++;
			} else if((chtype == 'O')&&(channelnum<OCEM_OUTPUT_CHANNELS)){
				ochannel[channelnum] = ocem_channel(channelmin,channelmax);

				DPRINT("[%s,%d] setting output channel %d at %.10lu us to (%d,%d)",dev.c_str(),slave_id,channelnum,  ochannel[channelnum].mod_time(), ochannel[channelnum].get().min, ochannel[channelnum].get().max);
				parsed++;
			} else {
				DERR("[%s,%d] error parsing PRG",dev.c_str(),slave_id);
			}
		} else if(sscanf(pnt,"POL %3s",datac)==1){
			if(!strncmp(datac,"POS",3)){
				polarity = POL_POS;
				DPRINT("[%s,%d] got Polarity POS",dev.c_str(),slave_id);
				parsed++;
			} else if(!strncmp(datac,"NEG",3)){
				polarity = POL_NEG;
				DPRINT("[%s,%d] got Polarity NEG",dev.c_str(),slave_id);
				parsed++;
			} else if(!strncmp(datac,"OPN",3)){
				polarity = POL_ZERO;
				DPRINT("[%s,%d] got Polarity OPEN",dev.c_str(),slave_id);
				parsed++;
			} else if(!strncmp(datac,"RUN",3)){
				polarity = POL_RUN;
				DPRINT("[%s,%d] got Polarity INSTABLE",dev.c_str(),slave_id);
				parsed++;
			} else {
				DERR("[%s,%d] error parsing POL",dev.c_str(),slave_id);
			}
		} else if(sscanf(pnt,"SEL %3s",datac)==1){
			if(!strncmp(datac,"PRE",3)){
				selector_state = SELECTOR_PRE;
				DPRINT("[%s,%d] got Selector PRE",dev.c_str(),slave_id);
				parsed++;
			} else if(!strncmp(datac,"LOC",3)){
				selector_state = SELECTOR_LOC;
				DPRINT("[%s,%d] got Selector LOC",dev.c_str(),slave_id);
				parsed++;
			} else if(!strncmp(datac,"ERR",3)){
				selector_state = SELECTOR_ERROR;
				DPRINT("[%s,%d] got Selector ERROR",dev.c_str(),slave_id);
				parsed++;
			} else {
				DERR("[%s,%d] error parsing SEL",dev.c_str(),slave_id);
			}
		} else if(sscanf(pnt,"STA %3s",datac)==1){
			if(!strncmp(datac,"ATT",2)){
				regulator_state=REGULATOR_ON;
				DPRINT("[%s,%d] got Regulator ON",dev.c_str(),slave_id);
				parsed++;
			} else if(!strncmp(datac,"STB",3)){
				regulator_state = REGULATOR_STANDBY;
				DPRINT("[%s,%d] got Regulator Standby",dev.c_str(),slave_id);
				parsed++;
			} else if(!strncmp(datac,"ERR",3)){
				DPRINT("[%s,%d] got Regulator Error",dev.c_str(),slave_id);
				regulator_state = REGULATOR_ERROR;
				parsed++;
			} else {
				DERR("[%s,%d] error parsing STA",dev.c_str(),slave_id);
			}
		} else if(sscanf(pnt,"COR %u",&datau)==1){
			current = datau;
			DPRINT("[%s,%d] got Corrente %d",dev.c_str(),slave_id,datau);
			parsed++;
		} else if(sscanf(pnt,"TEN %u",&datau)==1){
			voltage = datau;
			DPRINT("[%s,%d] got Tensione %d",dev.c_str(),slave_id,datau);
			parsed++;
		} else if(sscanf(pnt,"VER OCEM SPA TELEOPERATORE %c%2d%2d%2d%8s",&rilascio,&anno,&mese,&giorno,tipo)==5){
			DPRINT("[%s,%d] got version rilascio \"%c\" %.2d/%.2d/%.2d tipo:\"%s\"",dev.c_str(),slave_id,rilascio,giorno,mese,anno,tipo);
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
							DPRINT("[%s,%d] AC unbalance",dev.c_str(),slave_id);
							break;
						case PHASE_LOSS:
							alarms=common::powersupply::POWER_SUPPLY_PHASE_LOSS | alarms;
							DPRINT("[%s,%d] PHASE LOSS",dev.c_str(),slave_id);
							break;
						case AIR_FLOW:
							alarms=common::powersupply::POWER_SUPPLY_AIR_FLOW | alarms;
							DPRINT("[%s,%d] AIR FLOW",dev.c_str(),slave_id);
							break;
						case DOORS:
							alarms=common::powersupply::POWER_SUPPLY_EVENT_DOOR_OPEN | alarms;
							DPRINT("[%s,%d] DOOR OPEN",dev.c_str(),slave_id);
							break;

						case TRASFORMER_OVT:
							alarms=common::powersupply::POWER_SUPPLY_TRANSFORMER_OVT | alarms;
							DPRINT("[%s,%d] TRANSFORMER OVT",dev.c_str(),slave_id);
							break;
						case SNUBBER_FUSE:
							alarms=common::powersupply::POWER_SUPPLY_SNUBBER_FUSE | alarms;
							DPRINT("[%s,%d] SNUBBER_FUSE",dev.c_str(),slave_id);
							break;
						case SCR_FUSE:
							alarms=common::powersupply::POWER_SUPPLY_SCR_FUSE | alarms;
							DPRINT("[%s,%d] SCR_FUSE",dev.c_str(),slave_id);
							break;

						case SCR_OVT:
							alarms=common::powersupply::POWER_SUPPLY_SCR_OVT | alarms;
							DPRINT("[%s,%d] SCR_OVT",dev.c_str(),slave_id);
							break;
						case CHOKE_OVT:
							alarms=common::powersupply::POWER_SUPPLY_CHOKE_OVT | alarms;
							DPRINT("[%s,%d] CHOKE_OVT",dev.c_str(),slave_id);
							break;

						case PASS_FILTER:
							alarms=common::powersupply::POWER_SUPPLY_PASS_FILTER | alarms;
							DPRINT("[%s,%d] PASS_FILTER",dev.c_str(),slave_id);
							break;
						case DIODE_FAULT:
							alarms=common::powersupply::POWER_SUPPLY_DIODE_FAULT | alarms;
							DPRINT("[%s,%d] DIODE_FAULT",dev.c_str(),slave_id);
							break;
						case DIODE_OVT:
							alarms=common::powersupply::POWER_SUPPLY_DIODE_OVT | alarms;
							DPRINT("[%s,%d] DIODE_OVT",dev.c_str(),slave_id);
							break;

						case ACTIVE_FILTER_OVT:
							alarms=common::powersupply::POWER_SUPPLY_ACTIVE_FILTER_OVT | alarms;
							DPRINT("[%s,%d] ACTIVE_FILTER_OVT",dev.c_str(),slave_id);
							break;

						case ACTIVE_FILTER_FUSE:
							alarms=common::powersupply::POWER_SUPPLY_ACTIVE_FILTER_FUSE | alarms;
							DPRINT("[%s,%d] ACTIVE_FILTER_FUSE",dev.c_str(),slave_id);
							break;

						case DCCT_FAULT:
							alarms=common::powersupply::POWER_SUPPLY_DCCT_FAULT | alarms;
							DPRINT("[%s,%d] DCCT_FAULT",dev.c_str(),slave_id);
							break;

						case DCCT_OVT:
							alarms=common::powersupply::POWER_SUPPLY_DCCT_OVT | alarms;
							DPRINT("[%s,%d] DCCT_OVT",dev.c_str(),slave_id);
							break;

						case EARTH_FAULT:
							alarms=common::powersupply::POWER_SUPPLY_EARTH_FAULT | alarms;
							DPRINT("[%s,%d] EARTH_FAULT",dev.c_str(),slave_id);
							break;

							/*  case CUBICLE_OVT:
                            alarms=common::powersupply::POWER_SUPPLY_CUBICLE_OVT | alarms;
                            DPRINT("CUBICLE_OVT");
                            break;
							 */
						case SETPOINT_CARD_FAULT:
							alarms=common::powersupply::POWER_SUPPLY_SETPOINT_CARD_FAULT | alarms;
							DPRINT("[%s,%d] SETPOINT_CARD_FAULT",dev.c_str(),slave_id);
							break;

						case EXTERNAL_INTERLOCK:
						case CUBICLE_OVT:
							alarms=common::powersupply::POWER_SUPPLY_EXTERNAL_INTERLOCK | alarms;
							DPRINT("[%s,%d] EXTERNAL_INTERLOCK",dev.c_str(),slave_id);
							break;

						default:
							alarms=common::powersupply::POWER_SUPPLY_ALARM_UNDEF | alarms;
							DPRINT("[%s,%d] POWER_SUPPLY_ALARM_UNDEF",dev.c_str(),slave_id);
							break;
						}

						DPRINT("[%s,%d] setting alarm \"%d\", alarm mask 0x%llx",dev.c_str(),slave_id,al, (unsigned long long)alarms);
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
			DPRINT("[%s,%d] Checking result %d, tot Poll time %.10lu us",dev.c_str(),slave_id,ret,totPollTime);

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
	DPRINT("[%s,%d] OCEM CUSTOM INITIALIZATION",dev.c_str(),slave_id);
	do{
		DPRINT("[%s,%d] performing poll %d",dev.c_str(),slave_id,cnt);

		ret=ocem_prot->poll(slave_id, buf, sizeof(buf),1000,&timeo);
		DPRINT("[%s,%d] returned %d",dev.c_str(),slave_id,ret);

		if(ret>0){
			updateInternalData(buf);
			result++;
		} else if(ret==common::serial::ocem::OcemProtocol::OCEM_NO_TRAFFIC){
			DPRINT("[%s,%d] NO traffic",dev.c_str(),slave_id);

		}
		usleep(500000);
	} while(cnt--);

	DPRINT("[%s,%d] performing RMT",dev.c_str(),slave_id);
	ret=ocem_prot->select(slave_id, "RMT",1000,&timeo);
	DPRINT("[%s,%d] returned %d",dev.c_str(),slave_id,ret);

	sleep(3);
	cnt=0;
	do{
		DPRINT("[%s,%d] performing cleaning poll %d",dev.c_str(),slave_id,cnt);

		ret=ocem_prot->poll(slave_id, buf, sizeof(buf),1000,&timeo);
		DPRINT("[%s,%d] returned %d",dev.c_str(),slave_id,ret);
		//at the beggining some Ocem generate crc error
		if((ret>0) || (ret==common::serial::ocem::OcemProtocol::OCEM_POLL_ANSWER_CRC_FAILED)) {
			updateInternalData(buf);
			result++;
		}

		usleep(500000);
		cnt++;
		if((ret==common::serial::ocem::OcemProtocol::OCEM_NO_TRAFFIC)){
			DPRINT("[%s,%d] NO traffic",dev.c_str(),slave_id);
			result++;
			break;
		}
	} while (((ret>0)||(retry--)));
	DPRINT("[%s,%d] END OCEM CUSTOM INITIALIZATION sucessfully operations:%d",dev.c_str(),slave_id,result);

	if(result==0){
		DERR("[%d] no activity",slave_id);
		return -3;
	}
	ocem_prot->select(slave_id, "SL",1000,&timeo);

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

	DPRINT("[%s,%d] INITIALIZING",dev.c_str(),slave_id);
	current_sp=0;
	ramps_sp=0;
	pol_sp=0;
	state_sp=0;

	if(ocem_prot){
		ocem_prot->registerSlave(slave_id);
		ret = ocem_prot->init();

		DPRINT("[%s,%d] ocem protocol initialized ret=%d",dev.c_str(),slave_id,ret)
	} /*else {
		ocem_prot = getOcemProtocol(dev,baudrate,parity,bits,stop);
		if(ocem_prot){
			ocem_prot->registerSlave(slave_id);
			ret = ocem_prot->init();

			DPRINT("[%s,%d] ocem protocol created and initialized ret=%d",dev.c_str(),slave_id,ret)
		} else {
			ERR("[%s,%d] CANNOT ALLOCATE OCEM PROTOCOL",dev.c_str(),slave_id);
			return -12;
		}

	}*/
	if(ret!=0){
		ERR("[%s,%d] CANNOT INITIALIZE SERIAL PORT",dev.c_str(),slave_id);
		return ret;
	}
	ocem_prot->start();

	if(ocemInitialization()<0){
		return -1000;
	}


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
		DPRINT("[%s,%d] type 2-3-4 detected max current 100A",dev.c_str(),slave_id);
		updateParamsByModel(OCEM_MODEL234);
	} else if(strstr(version.get().type,"/1-5A-5B")){
		DPRINT("[%s,%d] type 2-5A-5B detected max current 700A",dev.c_str(),slave_id);
		updateParamsByModel(OCEM_MODEL5A5B);
	} else {
		DPRINT("[%s,%d] uknown type:\"%s\"",dev.c_str(),slave_id,version.get().type);
		updateParamsByModel(OCEM_UKNOWN);
	}

	if(adc_current_conversion==0 || max_current==0){
		ERR("[%s,%d] No max current set",dev.c_str(),slave_id);
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

	DPRINT("[%s,%d] DEINITIALIZING",dev.c_str(),slave_id);
	if(run){
		run=0;
		DPRINT("[%s,%d] STOPPING THREAD 0x%p",dev.c_str(),slave_id,(void*)rpid);
		pthread_join(rpid,(void**)&ret);
	}
	if(ocem_prot.get()){
		DPRINT("[%s,%d] REMOVING SLAVE  prot 0x%p",dev.c_str(),slave_id,ocem_prot.get());

		ocem_prot->unRegisterSlave(slave_id);
	}
	removeOcemProtocol(dev);
	initialized=0;
	return 0;
}



// return the number of characters sucessfully written or an error
int OcemE642X::send_command(const char*cmd,uint32_t timeout,int*tim){
	char command[1024];
	int ret;
	if(cmd==NULL)
		return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
	if(tim)
		*tim=0;
	DPRINT("[%s,%d] sending command \"%s\", timeout %d",dev.c_str(),slave_id,cmd,timeout);
	snprintf(command,sizeof(command),"%s" OCEM_TERMINATOR,cmd);
	ret =ocem_prot->select(slave_id, command,timeout,tim);
	if(ret>0){
		DPRINT("[%s,%d] command sent %d bytes to slave sucessfully",dev.c_str(),slave_id,ret);
	} else if(ret == 0){
		DPRINT("[%s,%d] nothing has been sent",dev.c_str(),slave_id);
	} else {
		ERR("[%s,%d]  error %d occurred slave ",dev.c_str(),slave_id,ret);

	}

	return ret;
}
// return the number of characters sucessfully read or an error
int OcemE642X::send_receive(char*cmd,char*buf,int size,uint32_t timeos,uint32_t timeop,int *timeo){
	int ret;
	uint64_t tstart;
	uint64_t totPollTime=0;
	DPRINT("[%s,%d] sending command timeout set to %u ms",dev.c_str(),slave_id,timeos);
	ret=send_command(cmd,timeos,timeo);

	if(ret>0){
		DPRINT("[%s,%d] command returned %d, check timeout set to %u ms",dev.c_str(),slave_id,ret,timeop);
		do{
			tstart= common::debug::getUsTime();
			ret = receive_data( buf, size,timeop,timeo);
			totPollTime+= common::debug::getUsTime()-tstart;
			DPRINT("[%s,%d] checking result %d, tot Poll time %.10lu us, polling tim %.10lu ms max poll time %u ms, timeout %d",dev.c_str(),slave_id,ret,totPollTime,(totPollTime/1000),timeop,*timeo);
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
		DPRINT("[%s,%d] received %d bytes from %d (timeout %d) \"%s\"",dev.c_str(),slave_id,ret,slave_id,timeout,buf);
		data_read=updateInternalData(buf);
	} else if(ret == 0){
		DPRINT("[%s,%d] nothing received from slave, timeout %d",dev.c_str(),slave_id,timeout);
		return 0;
	} else if(ret == common::serial::ocem::OcemProtocol::OCEM_NO_TRAFFIC){
		DPRINT("[%s,%d] NO DATA",dev.c_str(),slave_id);
		return 0;
	} else {
		ERR("[%s,%d] error %d occurred, (timeout %d)",dev.c_str(),slave_id,ret,timeout);
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
	DPRINT("[%s,%d] Set polarity %d",dev.c_str(),slave_id,pol);
	if(pol>0){
		CMD_WRITE_AND_CHECK(polarity,timeo_ms,"POS","POL",POL_POS);
	} else if(pol<0){
		CMD_WRITE_AND_CHECK(polarity,timeo_ms,"NEG","POL",POL_NEG);
	} else if(pol==0){
		CMD_WRITE_AND_CHECK(polarity,timeo_ms,"OPN","POL",POL_ZERO);
	}
	pol_sp=pol;
	start_pol=1;
	retry_pol=0;
	try_check_pol=0;
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
	if(retry_pol>=OCEM_MAX_COMMAND_RETRY){
		ERR("[%s,%d] max number of retries performing polarity change to %d",dev.c_str(),slave_id,pol_sp);
		retry_pol=0;
		return POWER_SUPPLY_COMMAND_ERROR;
	}
	timeo_ms=std::max(timeo_ms,OCEM_DEFAULT_TIMEOUT_MS);

	if(timeo_ms && (polarity.mod_time()> timeo_ms*1000)){
		ERR("[%s,%d] Timeout of %u ms getting polarity mod time:%ld",dev.c_str(),slave_id,timeo_ms,polarity.mod_time()/1000);

		return POWER_SUPPLY_TIMEOUT;
	}
	return 0;
}

int OcemE642X::setCurrentSP(float set_current_sp,uint32_t timeo_ms){
	char stringa[256];
	int val;
	if(set_current_sp<min_current || set_current_sp>max_current)
		return POWER_SUPPLY_BAD_INPUT_PARAMETERS;

	val =(int)round(set_current_sp/adc_current_conversion);
	snprintf(stringa,sizeof(stringa),"SP %.7d",val);
	DPRINT("[%s,%d] set current -> %.7d (0x%x) '%s'",dev.c_str(),slave_id,val,val,stringa);
	sp_current = set_current_sp/adc_current_conversion;

	CMD_WRITE(stringa,timeo_ms);
	current_sp=set_current_sp;
	delta_current_sp=fabs(current*adc_current_conversion-current_sp);

	return 0;
}


int OcemE642X::getCurrentSP(float* curr,uint32_t timeo_ms){
	*curr = sp_current*adc_current_conversion;
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
	if(retry_current>=OCEM_MAX_COMMAND_RETRY){
		ERR("[%s,%d] max number of retries performing set point current to %f",dev.c_str(),slave_id,current_sp);
		retry_current=0;
		return POWER_SUPPLY_COMMAND_ERROR;
	}
	timeo_ms=std::max(timeo_ms,OCEM_DEFAULT_TIMEOUT_MS);

	if(timeo_ms && (current.mod_time()> timeo_ms*1000)){
		ERR("[%s,%d] Timeout of %u ms, getting current mod time:%llu.",dev.c_str(),slave_id,timeo_ms,current.mod_time()/1000);

		return POWER_SUPPLY_TIMEOUT;
	}
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
	start_ramp=1;
	retry_current=0;
	try_check_current=0;
	return 0;
}



int OcemE642X::setCurrentRampSpeed(float asup,float asdown,uint32_t timeo_ms){
	int rsup,rsdown;
	char stringa[256];
	float sensibility=((max_current/10.0 -1.0)*1.0)/(1<<current_ramp_adc);
	if(asup<1.0 || asup > (max_current/10.0)){
		ERR("[%s,%d] bad input parameters asup %f",dev.c_str(),slave_id,asup);
		return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
	}
	if(asdown<1.0 || asdown > (max_current/10.0)){
		ERR("[%s,%d] bad input parameters asdown %f",dev.c_str(),slave_id,asdown);
		return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
	}
	rsup = asup/sensibility;

	rsdown = asdown/sensibility;
	sprintf(stringa,"VRUP %.7d",rsup);
	DPRINT("[%s,%d] setting ramp rising speed to %f as (0x%x), falling speed to %f (0x%x)",dev.c_str(),slave_id,asup,rsup,asdown,rsdown);
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
	timeo_ms=std::max(timeo_ms ,OCEM_DEFAULT_TIMEOUT_MS);

	if(timeo_ms && (alarms.mod_time()> timeo_ms*1000)){
		ERR("[%s,%d] Timeout of %u ms getting alarms mod time:%llu",dev.c_str(),slave_id,timeo_ms,alarms.mod_time()/1000);

		return POWER_SUPPLY_TIMEOUT;
	}
	return 0;
}


int OcemE642X::shutdown(uint32_t timeo_ms){
	regulator_state = REGULATOR_SHOULD_BE_OFF;
	// because we lose control once done this command
	DPRINT("[%s,%d] SHUTDOWN command, we are going to lose control!!",dev.c_str(),slave_id);
	CMD_WRITE("OFF",timeo_ms);
	return 0;
}
int OcemE642X::poweron(uint32_t timeo_ms){
	CMD_WRITE_AND_CHECK(regulator_state,timeo_ms,"ON","SEL",REGULATOR_ON);
	state_sp=REGULATOR_ON;
	start_state=1;
	try_check_state=0;
	retry_state=0;
	return 0;
}

int OcemE642X::standby(uint32_t timeo_ms){
	CMD_WRITE_AND_CHECK(regulator_state,timeo_ms,"STB","SEL",REGULATOR_STANDBY);
	state_sp=REGULATOR_STANDBY;
	start_state=1;
	retry_state=0;
	try_check_state=0;
	return 0;
}


int  OcemE642X::getState(int* state,std::string &desc,uint32_t timeo_ms){
	*state = 0;
	std::stringstream ss;
	//  GET_VALUE(regulator_state,timeo_ms,SL);

	if(regulator_state == REGULATOR_SHOULD_BE_OFF) {
		*state |=POWER_SUPPLY_STATE_OFF;
		ss<< "off ";
	}

	if(regulator_state == REGULATOR_STANDBY){
		*state |= POWER_SUPPLY_STATE_STANDBY;
		ss<< "standby ";
	}

	if(regulator_state == REGULATOR_ON){
		*state |= POWER_SUPPLY_STATE_ON;
		ss<< "on ";
	}
	if(regulator_state == REGULATOR_UKN){
		*state|=POWER_SUPPLY_STATE_UKN;
		ss<< "ukwnown ";
	}

	if(regulator_state == REGULATOR_ERROR){
		*state|=POWER_SUPPLY_STATE_ERROR;
		ss<< "error ";
	}

	if(selector_state == SELECTOR_LOC){
		*state|=POWER_SUPPLY_STATE_LOCAL;
		ss<< "local ";
	}
	if(alarms!=0){
		*state |= POWER_SUPPLY_STATE_ALARM;
		ss<< "Alarm ";
	}

	desc=ss.str();
	timeo_ms=std::max(timeo_ms ,OCEM_DEFAULT_TIMEOUT_MS);

	if(timeo_ms && (regulator_state.mod_time()> timeo_ms*1000)){
		ERR("[%s,%d] Timeout of %u ms getting state mod time:%llu",dev.c_str(),slave_id,timeo_ms,regulator_state.mod_time()/1000);
		//regulator_state = REGULATOR_UKN;

		return POWER_SUPPLY_TIMEOUT;
	}
	if(retry_state>=OCEM_MAX_COMMAND_RETRY){
		ERR("[%s,%d] max number of retries performing state change",dev.c_str(),slave_id);
		retry_state=0;
		return POWER_SUPPLY_COMMAND_ERROR;
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
			ERR("[%s,%d] data is not available yet",dev.c_str(),slave_id);
			return POWER_SUPPLY_READOUT_OUTDATED;
		}
	} else if((inout==1) && (number < OCEM_OUTPUT_CHANNELS )) {
		DPRINT("get channel OUTPUT number %d, timeo %d",number,timeout);
		*min = ochannel[number].get().min;
		*max = ochannel[number].get().max;
		if(ochannel[number].mod_time()> stamp){
			DPRINT("[%s,%d] got output channel %d update %.7d %.7d",dev.c_str(),slave_id,number,*min,*max);
			return 0;
		} else {
			ERR("[%s,%d] data is not available yet",dev.c_str(),slave_id);
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
	DPRINT("[%s,%d] set current sensibility to %f A",dev.c_str(),slave_id,sens);
	current_sensibility=sens;
	ret=setThreashold(0,sens,1000);
	if(ret<0)return ret;
	return 0;
}        
int OcemE642X::setVoltageSensibility(float sens){
	int ret;
	voltage_sensibility=sens;
	DPRINT("[%s,%d] set voltage sensibility to %f A",dev.c_str(),slave_id,sens);

	ret=setThreashold(1,sens,1000);
	if(ret<0)return ret;
	return 0;

}
