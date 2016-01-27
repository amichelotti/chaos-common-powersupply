#include "AL250PowerSupply.h"
#include <common/debug/core/debug.h>
#include <boost/thread/mutex.hpp>
#define IsInInterval(value,min,max)  ( ( value>=min ) && (value<=max) )  

#define CHECK_IF_ALLOWED \
 if (slave == 0){ \
        DERR("command not allowed on master");\
        return DEFAULT_NOT_ALLOWED;}

static boost::mutex io_mux;

using namespace common;
using namespace common::powersupply;

//definizione della mappa statica
std::map<const std::string,AL250::ChannelPhysicalMap,common::CompareStdStr> common::powersupply::AL250::mainUnitTable;


powersupply::AL250::AL250(const std::string Parameters, int sl) {
    this->ConnectionParameters=strdup(Parameters.c_str());
    this->Hardware=NULL; // new Hazemeyer::Corrector((char*)Parameters.c_str());
    this->slave=sl;
    this->CurrentSP=0;
    this->HwMaxCurrent=9.995;
    this->HwMinCurrent=-10;
    
}
AL250::~AL250() { 
 //   DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);
 //   DPRINT("ALEDEBUG after sem");
    this->deinit();
    if(this->ConnectionParameters){
        free(this->ConnectionParameters);
    }
    this->ConnectionParameters=NULL;
}
int AL250::getPolarity(int* pol, uint32_t timeo_ms) {
   int32_t  iData=0;
    double data;
    bool ret;
//    DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);
//    DPRINT("ALEDEBUG after sem");
    //this->Hardware->setModbusReadTimeout(timeo_ms*1000);

    //DPRINT("called getPolarity for slave %d\n",this->slave);
    if (this->slave == 0)
    {
        ret=this->Hardware->ReadBitRegister(Hazemeyer::Corrector::MAIN_AVERAGE_I,(int16_t*)&iData);
        if (ret)
        {
            data=this->Hardware->ConvertFromDigit(Hazemeyer::Corrector::CONV_MAIN_AMP,iData);
            
        }
    }
    else
    {
       ret=this->Hardware->ReadChannelCurrent(this->slave-1,&data);
    }
    if (ret)
    {
        *pol = (data > 0)? 1:-1;
        return 0;
    }
    DPRINT("error reading channel current (ret= %d) for slave %d",ret,this->slave);    
    return POWER_SUPPLY_RECEIVE_ERROR;
    
}
int AL250::getCurrentOutput(float* current, uint32_t timeo_ms) {
    bool ret;
    int32_t iData=0;
    double data;
 //   DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);
  //  DPRINT("ALEDEBUG after sem");


    //this->Hardware->setModbusReadTimeout(timeo_ms*1000);
    DPRINT("slave %d reading current",slave);

    if (this->slave == 0)
    {
        ret=this->Hardware->ReadBitRegister(Hazemeyer::Corrector::MAIN_AVERAGE_I,(int16_t*)&iData);
        if (ret)
        {
            *current=(float) this->Hardware->ConvertFromDigit(Hazemeyer::Corrector::CONV_MAIN_AMP,iData);
            DPRINT("slave %d, ent=0x%f",slave,*current);
            return 0;
        }
        else  {
            DERR("slave %d error reading current ",slave);
            return POWER_SUPPLY_RECEIVE_ERROR;
        }   
    }
    else
    {
        ret=this->Hardware->ReadChannelCurrent(this->slave-1,&data);
        if (ret)
        {
            *current= (float) data;
            DPRINT("slave %d, ent=0x%f",slave,*current);

            return 0;
        }
        else{
           DERR("slave %d error reading current",slave);

           return POWER_SUPPLY_RECEIVE_ERROR;
        }
    }
}
int AL250::getVoltageOutput(float* volt, uint32_t timeo_ms ) {
    bool ret;
    int32_t iData=0;
    double data;
//    DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);
 //   DPRINT("ALEDEBUG after sem");

    //this->Hardware->setModbusReadTimeout(timeo_ms*1000);

    DPRINT("getVoltageOutput called for slave %d\n",this->slave);
    if (this->slave == 0)
    {
        ret=this->Hardware->ReadBitRegister(Hazemeyer::Corrector::MAIN_AVERAGE_V,(int16_t*)&iData);
        if (ret)
        {
            *volt=(float) this->Hardware->ConvertFromDigit(Hazemeyer::Corrector::CONV_MAIN_VOLT,iData);
            return 0;
        }
        else
            return POWER_SUPPLY_RECEIVE_ERROR;
    }
    else
    {
        ret=this->Hardware->ReadChannelVoltage(this->slave-1,&data);
        if (ret)
        {
            *volt=(float) data;
               return 0;
        }
        else
            return POWER_SUPPLY_RECEIVE_ERROR;
    }

}
int AL250::getAlarms(uint64_t* alrm, uint32_t timeo_ms ) {
    int32_t  iData=0;
    int32_t  iMainData=0;
    uint64_t alCode=0;
    Hazemeyer::Corrector::ReadReg  Reg;
    Hazemeyer::Corrector::ReadReg  MainFaults=Hazemeyer::Corrector::GENERAL_FAULTS;
    bool ret, centralUnitFault=false;
 //   DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);
 //   DPRINT("ALEDEBUG after sem");

    //this->Hardware->setModbusReadTimeout(timeo_ms*1000);
    switch (this->slave)
    {
        default: 
            DERR("[%d] Power supply Command Error",slave);
            return POWER_SUPPLY_COMMAND_ERROR;
        case 0: break;
        case 1: Reg=Hazemeyer::Corrector::CH0_FAULTS; break;
        case 2: Reg=Hazemeyer::Corrector::CH1_FAULTS; break;
        case 3: Reg=Hazemeyer::Corrector::CH2_FAULTS; break;
        case 4: Reg=Hazemeyer::Corrector::CH3_FAULTS; break;
        case 5: Reg=Hazemeyer::Corrector::CH4_FAULTS; break;
        case 6: Reg=Hazemeyer::Corrector::CH5_FAULTS; break;
        case 7: Reg=Hazemeyer::Corrector::CH6_FAULTS; break;
        case 8: Reg=Hazemeyer::Corrector::CH7_FAULTS; break;
     
    }
    ret=this->Hardware->ReadBitRegister(MainFaults,(int16_t*)&iMainData);
    if (!ret) 
            return POWER_SUPPLY_RECEIVE_ERROR;
    if  (this->slave != 0)
    {
        
        if (iMainData != 0)  {
            alCode|=POWER_SUPPLY_MAINUNIT_FAIL;
        }
         ret=this->Hardware->ReadBitRegister(Reg,(int16_t*)&iData);
        if (!ret) {
             DERR("[%d] Power supply Receive Error",slave);

            return POWER_SUPPLY_RECEIVE_ERROR;
        }
        if (iData &  0x1) alCode|=POWER_SUPPLY_EVENT_OVER_TEMP;
        if (iData &  0x2) alCode|=POWER_SUPPLY_FUSE_FAULT;
        if (iData &  0x4) alCode|=POWER_SUPPLY_OVER_VOLTAGE;
        if (iData &  0x8) alCode|=POWER_SUPPLY_OVER_CURRENT;
        if (iData &  0x10) alCode|=POWER_SUPPLY_COMMUNICATION_FAILURE;
    } else {
        if (iMainData & 0x400) alCode|=POWER_SUPPLY_EVENT_DOOR_OPEN;
        if (iMainData & 0x800) alCode|=POWER_SUPPLY_EVENT_OVER_TEMP;
        if (iMainData & 0x1) alCode|=POWER_SUPPLY_EVENT_OVER_TEMP;
        if (iMainData & 0x2) alCode|=POWER_SUPPLY_EVENT_OVER_TEMP;
        if (iMainData & 0x4) alCode|=POWER_SUPPLY_EVENT_OVER_TEMP;
        if (iMainData & 0x8) alCode|=POWER_SUPPLY_FUSE_FAULT;
        if (iMainData & 0x10) alCode|=POWER_SUPPLY_EARTH_FAULT;
        
    }
    //fornire l'errore secondo lo standard
    *alrm=  alCode;
    return 0;
    
    
}
int AL250::resetAlarms(uint64_t alrm,uint32_t timeo_ms) {
    
    int ret;
 //   DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);
 //   DPRINT("ALEDEBUG after sem");


    //this->Hardware->setModbusWriteTimeout(timeo_ms*1000);
    if (this->slave == 0){
        ret=this->Hardware->ResetMainUnit();
    } else{
        ret=this->Hardware->SendChannelCommand(this->slave-1,Hazemeyer::Corrector::CHANNEL_RESET);
    }
    if(ret!=true){
        DERR("[%d] command error",slave);;
        return POWER_SUPPLY_COMMAND_ERROR;
    }
    return 0;
}
int AL250::shutdown(uint32_t timeo_ms ) {
    int ret;
 //   DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);

//    DPRINT("ALEDEBUG after sem");
    //this->Hardware->setModbusWriteTimeout(timeo_ms*1000);
    if (this->slave == 0){
        ret=this->Hardware->TurnOffMainUnit();
    } else{
        ret=this->Hardware->SendChannelCommand(this->slave-1,Hazemeyer::Corrector::CHANNEL_OFF);
    }    
     if(ret!=true){
        DERR("[%d] command error",slave);;
        return POWER_SUPPLY_COMMAND_ERROR;
    }
    return 0;
}
int AL250::poweron(uint32_t timeo_ms){
    int ret;
 //   DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);

  //  DPRINT("ALEDEBUG after sem");

    // this->Hardware->setModbusWriteTimeout(timeo_ms*1000);
    if (this->slave == 0){
        ret=this->Hardware->TurnOnMainUnit();
    } else {
        ret=this->Hardware->SendChannelCommand(this->slave-1,Hazemeyer::Corrector::CHANNEL_ON);
    } 
     if(ret!=true){
        DERR("[%d] command error",slave);;
        return POWER_SUPPLY_COMMAND_ERROR;
    }
    return 0;
}
int AL250::standby(uint32_t timeo_ms) {
    int ret;
 //   DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);

 //   DPRINT("ALEDEBUG after sem");
    //  this->Hardware->setModbusWriteTimeout(timeo_ms*1000);

    if (this->slave == 0){
        ret = Hardware->TurnStandbyMainUnit();
    } else {
        ret= Hardware->SendChannelCommand(this->slave-1,Hazemeyer::Corrector::CHANNEL_OFF);
        
    }
     if(ret!=true){
        DERR("[%d] command error",slave);
        return POWER_SUPPLY_COMMAND_ERROR;
    }
    return 0;
    
}
int AL250::getCurrentSP(float* current,uint32_t timeo_ms) {
    /*
      if (this->slave == 0)
        return DEFAULT_NOT_ALLOWED;
    */
    
    *current=this->CurrentSP;
    return 0;
}
int AL250::setCurrentSP(float current, uint32_t timeo_ms) {

    CHECK_IF_ALLOWED;
    
    if (!IsInInterval(current,this->minCurrent,this->maxCurrent)){
         DERR("[%d] current out of ranges [%f:%f]",slave,minCurrent,maxCurrent);

        return POWER_SUPPLY_BAD_OUT_OF_RANGE_PARAMETERS;
    }
    
    this->CurrentSP=current;
    return 0;
}
int AL250::getMaxMinCurrent(float* max, float* min) {
    CHECK_IF_ALLOWED;
    *max=this->maxCurrent;
    *min=this->minCurrent;
    return 0;
}
int AL250::startCurrentRamp(uint32_t timeo_ms) {
    bool ret;
    DPRINT( "slave %d starting ramp",slave);
    CHECK_IF_ALLOWED;
    boost::mutex::scoped_lock lock(io_mux);
    
    //this->Hardware->setModbusWriteTimeout(timeo_ms*1000);

    ret=this->Hardware->SetChannelCurrent(this->slave-1,this->CurrentSP);
    ret= (ret== true)? 0 : POWER_SUPPLY_COMMAND_ERROR;
    return ret;
}
int AL250::init(){
    AL250::ChannelPhysicalMap Elem;
    bool ret=false;
    size_t index=0;
    //DPRINT("ALEDEBUG before sem");
    boost::mutex::scoped_lock lock(io_mux);
   // DPRINT("ALEDEBUG after sem");

    DPRINT( "slave %d initializing",slave);

    std::string App(this->ConnectionParameters);
     std::pair<map<std::string,AL250::ChannelPhysicalMap>::iterator,bool> pRet;
    std::string SerialDev;
    index=App.find_first_of(',');
    if (index == string::npos)
    {
        this->initDone=false;
        DERR("error bad input parameters %s",App.c_str());
        return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    }
    SerialDev.insert(0,App,0,index);
   
      
    Elem.driverPointer=this->Hardware;
    std::pair<std::string,AL250::ChannelPhysicalMap> Pair(SerialDev,Elem);
    pRet=this->mainUnitTable.insert(Pair);
    if (pRet.second) //new element
    {
         
        this->Hardware = new Hazemeyer::Corrector( App.c_str());
        pRet.first->second.driverPointer=this->Hardware;
        DPRINT("connecting of a new Hazemeyer::Corrector:@ 0x%x",this->Hardware);
        ret=this->Hardware->Connect();
        if (!ret){
            DERR("offline");
            return POWER_SUPPLY_OFFLINE;
        }
    }
    else
    {
        DPRINT( "already existing, using: @0x%x",pRet.first->second.driverPointer);
        this->Hardware=pRet.first->second.driverPointer;
    }
    
    pRet.first->second.ConnectedChannels[this->slave]=true;
    this->printStaticTableContent();
 
  
    this->minCurrent=this->HwMinCurrent;
    this->maxCurrent=this->HwMaxCurrent;
   
    if (this->slave == 0){
        DPRINT("Turning on MainUnit");
        ret=this->Hardware->TurnOnMainUnit();
    } else{
        DPRINT("Turning on sending channel on slave %d",slave);

        ret=this->Hardware->SendChannelCommand(this->slave-1,Hazemeyer::Corrector::CHANNEL_ON);
    }
    
     if(ret!=true){
        DERR("[%d] command error",slave);
        initDone=false;
        return POWER_SUPPLY_COMMAND_ERROR;
    }
    
    
    
    initDone=true;
    return 0;
}
int AL250::deinit() {
    std::string key;
    std::vector<bool> *instances;
    std::string App(this->ConnectionParameters);
    //DPRINT("ALEDEBUG before sem");
   boost::mutex::scoped_lock lock(io_mux);
   // DPRINT("ALEDEBUG after sem");

    DPRINT( "slave %d deinitializinh",slave);

   // cout << "deinit slave " << this->slave << " " << this->ConnectionParameters <<endl;
    if (!this->initDone) 
        return 0;
    size_t index=App.find_first_of(',');
    if (index==std::string::npos) return POWER_SUPPLY_BAD_INPUT_PARAMETERS;
    key.insert(0,App,0,index);
    std::map<std::string,AL250::ChannelPhysicalMap>::iterator it;
    it=this->mainUnitTable.find(key); 
    if (it == this->mainUnitTable.end())
    {
        
        throw  "bad disallocation";
        return POWER_SUPPLY_EXCEPTION;
    }
    instances=&(it->second.ConnectedChannels);
    instances->at(this->slave)=false;
    unsigned short actives=0;
    for (int i=0; i < instances->size();i++)
    {
        if (instances->at(i)== true)
            actives++;
        
    }
    //deallochiamo se e solo se è l'ultimo.
    if (!actives)
    {
        
#ifndef ALEDEBUG
        this->Hardware->CloseConnection();
#endif
        this->Hardware->~Corrector();
        this->Hardware=NULL;
        this->mainUnitTable.erase(it);
        this->printStaticTableContent();
    }
    else
    {
        DERR("non posso deallocare");
        this->Hardware=NULL;
    }
    this->initDone=false;
    return 0;
 }
int AL250::forceMaxCurrent(float max) {
    float absMax= abs(max);
    if (this->slave==0)
        return DEFAULT_NOT_ALLOWED;
    
    if (max > this->HwMaxCurrent)
        return POWER_SUPPLY_BAD_OUT_OF_RANGE_PARAMETERS;
    else
    {
        this->maxCurrent=absMax;
        this->minCurrent=-absMax;
    }
    
    return 0;
}
int AL250::getCurrentSensibility(float* sens) {
    if (this->slave == 0)
        *sens=0.0025;
    else
        *sens=0.00122;
        
    return 0;
    
}
int AL250::getCurrentSensibilityOnSet(float* sens) {
    if (this->slave==0)
        return DEFAULT_NOT_ALLOWED;
    else
        *sens=0.00488;
}
int AL250::getAlarmDesc(uint64_t* alarm){
    return 0;
}

std::string AL250::getAlarmDescr(uint64_t alarm) {
    std::string Ret="";
    for (int i=0; i < 64; i++)
    {
        if ((alarm & (1 << i)) != 0)
        {
            switch(i)
            {
                case 0: Ret+="POWER SUPPLY DOOR OPEN\n"; break;
                case 1: Ret+="POWER SUPPLY OVER TEMP\n"; break;
                case 2: Ret+="POWER SUPPLY FUSE FAULT\n"; break;
                case 3: Ret+="POWER SUPPLY EARTH FAULT\n"; break;
                case 4: Ret+="POWER SUPPLY OVER VOLTAGE\n"; break;
                case 5: Ret+="POWER SUPPLY OVER CURRENT\n"; break;
                case 6: Ret+="POWER SUPPLY COMMUNICATION FAILURE\n"; break;
                case 7: Ret+="POWER SUPPLY FAILURE ON MAIN UNIT\n"; break;
                default: break;
                
            }
        }
        
    }
    
    return Ret;
}
int AL250::getVoltageSensibility(float* sens) {
    if (this->slave==0)
        *sens=0.311;
    else
        *sens=0.00183;
    return 0;
}

int AL250::getState(int* state, std::string& desc, uint32_t timeo_ms ) {
  //  DPRINT("ALEDEBUG before sem");
   
    boost::mutex::scoped_lock lock(io_mux);
  //  DPRINT("ALEDEBUG after sem");
    //this->Hardware->setModbusWriteTimeout(timeo_ms*1000);
    Hazemeyer::Corrector::ReadReg Reg;
    bool ret;
    int stCode=0;
    std::string Ret;
    int32_t data=0;
    *state=POWER_SUPPLY_STATE_UKN;
    DPRINT("slave %d getting state",slave);

    if (this->slave)
        Reg=Hazemeyer::Corrector::CHS_POWER;
    else
        Reg=Hazemeyer::Corrector::GENERAL_STATUS;
        
    ret = this->Hardware->ReadBitRegister(Reg,(int16_t*)&data);
    if (!ret) {
         DERR("slave %d reading state on reg 0x%x, ret=%d",slave,Reg,ret);
	 desc.assign("Communication Failure");
        return POWER_SUPPLY_RECEIVE_ERROR;
    }
    if (this->slave == 0)
    {
        if ((data & 1)==0) { 
	    desc.assign("Unknown State");
            stCode|=POWER_SUPPLY_STATE_UKN;
        }
        else
        {
            if (data & 2) {stCode|=POWER_SUPPLY_STATE_STANDBY;desc.assign("standby");}
            if (data & 4) {stCode|=POWER_SUPPLY_STATE_ON;desc.assign("on");}
            if ((data & 6)==0) {stCode|=POWER_SUPPLY_STATE_OFF;desc.assign("off");}
            if ((data & 8)) {stCode|=POWER_SUPPLY_STATE_LOCAL;desc+=" local";}
        }

    }
    else
    {
       
         if (data & (1 << (this->slave-1))) 
	 {
	    stCode|=POWER_SUPPLY_STATE_ON;
	    desc.assign("on");
	 }
         else
	 {
             stCode|=POWER_SUPPLY_STATE_STANDBY;
	     desc.assign("standby");
	 }
    }
    //adding ALARMS
    Reg=Hazemeyer::Corrector::GENERAL_FAULTS;
    ret = this->Hardware->ReadBitRegister(Reg,(int16_t*)&data);
    if (!ret) {
        DERR("slave %d reading faults on reg 0x%x",slave,Reg);

        return POWER_SUPPLY_RECEIVE_ERROR;   
    }
    if (data){
             stCode|=POWER_SUPPLY_STATE_ALARM;
    } else {
        if (this->slave)
        {
            Reg=Hazemeyer::Corrector::CHS_READY;
            ret = this->Hardware->ReadBitRegister(Reg,(int16_t*)&data);
            if (!ret) return POWER_SUPPLY_RECEIVE_ERROR;  
            
            if ((data & (1 << (this->slave-1)))==0) 
            {
                stCode|=POWER_SUPPLY_STATE_ALARM;
	        desc.assign("alarm");
	    }
        }
    }
    *state = stCode;
    return 0;
}
void AL250::printStaticTableContent() {
   std::map<const std::string,AL250::ChannelPhysicalMap>::iterator it;
   for (it=this->mainUnitTable.begin(); it!=this->mainUnitTable.end(); ++it)
  {
    std::cout << it->first << " => " << it->second.driverPointer;
    for (int c=0; c < it->second.ConnectedChannels.size();c++)
        cout << " " <<  it->second.ConnectedChannels[c];
    cout << endl;
  }
    
    
}
