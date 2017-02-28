//
//  test_serial.cpp
// 
//
//
//
//


#include "common/powersupply/powersupply.h"
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <common/debug/core/debug.h>
#include <boost/regex.hpp>
#include <string>
#ifdef CHAOS
#include <chaos/ui_toolkit/ChaosUIToolkit.h>
#endif
#define DEFAULT_TIMEOUT 10000
using boost::regex;

static char* convToUpper(char*str){
  char *b = str;
  char *tmp=str;
  if(str==NULL) return NULL;
  while((*str!=0)){
    *tmp=toupper(*str);
    tmp++,str++;
  }
  *tmp=0;
  return b;
}


static void printRawCommandHelp(){
    std::cout<<"\tSELECT <OCEMID> <CMD>      : perform a select"<<std::endl;
    std::cout<<"\tPOLL <OCEMID> : perform a poll and dump result"<<std::endl;
    std::cout<<"\tHELP               : this help"<<std::endl;
    std::cout<<"\tQUIT               : quit program"<<std::endl;
  
}
void raw_test(common::serial::ocem::OcemProtocol*oc){
  char stringa[1024];
  boost::regex cmd_match("^(\\w+) (\\d+)(\\s+(.+)|)");
  if(oc->init()!=0 ){
    printf("## cannot initialize protocol\n");
    return;
  }
  printRawCommandHelp();
  while(gets(stringa)){
      uint64_t tm;
      char *t=stringa;
      boost::smatch match;
      convToUpper(t);

      tm = common::debug::getUsTime();
      if(boost::regex_match(std::string(t),match,cmd_match,boost::match_perl)){
	int ret;
          std::string op=match[1];
	std::string ids=match[2];
	int id = atoi(ids.c_str());
	if(op == "SELECT"){
	  int timeout=0;
	  std::string cmd=match[4];
	  ret=oc->select(id,(char*)cmd.c_str(),5000,&timeout);
	  if(ret<0){
	    printf("## error sending ret:%d, timeout :%d\n",ret,timeout);
	  }
	} else if(op == "POLL"){
	  int timeout=0;
	  char buf[1024];
	  ret=oc->poll(id,buf,sizeof(buf),5000,&timeout);
	  if(ret<0){
	    printf("## error polling ret:%d, timeout %d\n",ret,timeout);
	  } else {
	    char outbuf[1024];
	    oc->decodeBuf(buf,outbuf,sizeof(outbuf));
	  }
	}
      } else if(!strcmp(t,"QUIT")){
	return;
      } else if(!strcmp(t,"HELP")){
	printRawCommandHelp();
	return;
      }
  }
}


static int check_for_char(){
  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(0,&fds);
  struct timeval tm;
  tm.tv_sec = 0;
  tm.tv_usec = 0;

  select(1,&fds,0,0,&tm);
  return FD_ISSET(0,&fds);
  
}
static void printCommandHelp(){
    std::cout<<" Available commands  :"<<std::endl;
    std::cout<<"\tPOL <1/0/-1>       : set polarity"<<std::endl;
    std::cout<<"\tSP <float>         : set SetPoint"<<std::endl;
    std::cout<<"\tSLOPE <up A/s> <down A/s>: set Slope rising and falling"<<std::endl;
    std::cout<<"\tRAMP               : start ramp"<<std::endl;
    std::cout<<"\tRSTALARMS          : reset alarms"<<std::endl;
    std::cout<<"\t------------------------------------------"<<std::endl;
    std::cout<<"\tSTANDBY            : force standby"<<std::endl;
    std::cout<<"\tON                 : force poweron"<<std::endl;
    std::cout<<"\tOFF                : force powerof"<<std::endl;
    std::cout<<"\t------------------------------------------"<<std::endl;
    std::cout<<"\tGETCURRENT         : get current"<<std::endl;
    std::cout<<"\tGETVOLTAGE         : get voltage"<<std::endl;
    std::cout<<"\tGETALARMS          : get alarms"<<std::endl;
    std::cout<<"\tGETSTATE           : get state"<<std::endl;
    std::cout<<"\tGETSP              : get SetPoint"<<std::endl;
    std::cout<<"\tGETVER             : get HW/SW version"<<std::endl;
    std::cout<<"\tGETPOL             : get polarity"<<std::endl;
    std::cout<<"\t-------------------------------------------"<<std::endl;
    std::cout<<"\tPOLL               : get Ocem answer"<<std::endl;
    std::cout<<"\tSELECT <command>   : select"<<std::endl;

    std::cout<<"\t-------------------------------------------"<<std::endl;
    std::cout<<"\tOPEN <ser> <slave> : open a new powersupply"<<std::endl;
    std::cout<<"\tCLOSE              : close powersupply"<<std::endl;
    std::cout<<"\tHELP               : this help"<<std::endl;
    std::cout<<"\tQUIT               : quit program"<<std::endl;

}
int main(int argc, char *argv[])
{

std::string ver;
    float maxcurrent=100;
    float maxvoltage=10;
    bool span=false;
    bool interactive=false;
    int slave_id;
    std::string dev;
 
#ifdef CHAOS
    
      chaos::ui::ChaosUIToolkit::getInstance()->getGlobalConfigurationInstance()->addOption("dev,d", po::value<std::string>(&dev), "The serial device /dev/ttyxx");
      chaos::ui::ChaosUIToolkit::getInstance()->getGlobalConfigurationInstance()->addOption("id", po::value<int>(&slave_id), "slave destination ID, ");
      
      chaos::ui::ChaosUIToolkit::getInstance()->getGlobalConfigurationInstance()->addOption("maxcurr", po::value<float>(&maxcurrent), "max current");
      chaos::ui::ChaosUIToolkit::getInstance()->getGlobalConfigurationInstance()->addOption("maxvolt", po::value<float>(&maxvoltage), "max voltage");
      chaos::ui::ChaosUIToolkit::getInstance()->getGlobalConfigurationInstance()->addOption("span", po::value<bool>(&span), "span to fin id");
     chaos::ui::ChaosUIToolkit::getInstance()->getGlobalConfigurationInstance()->addOption("interactive,i", po::value<bool>(&interactive), "interactive");
      chaos::ui::ChaosUIToolkit::getInstance()->init(argc, argv);

      
   


#else
    boost::program_options::options_description desc("options");
    
  desc.add_options()("help", "help");
  // put your additional options here
  desc.add_options()("dev", boost::program_options::value<std::string>(), "serial device where the ocem is attached");
  desc.add_options()("id", boost::program_options::value<int>(), "slave destination ID, ");
  desc.add_options()("maxcurr", boost::program_options::value<float>(), "max current ");
  desc.add_options()("maxvolt", boost::program_options::value<float>(), "max voltage ");


  desc.add_options()("interactive", "interactive test");
  desc.add_options()("span,s", "span devices find devices on the bus");
  desc.add_options()("raw,r", "raw access to ocem bus"); 
  if(vm.count("id")==0){
        std::cout<<"## you must specify an existing slave id [0:31]"<<desc<<std::endl;
     return -1;
  }
     if(vm.count("dev")==0){
        std::cout<<"## you must specify a valid device"<<desc<<std::endl;
     return -1;
    }
   slave_id = vm["id"].as<int>();
   dev = vm[dev].as<std::string>();
   if(vm.count("interactive"))
       interactive=true;
#endif
  //////
 


  if(span){
    std::cout<<"finding device on the bus"<<std::endl;
    int id=0;
    int found=0;
   
    for(;id<32;id++){
      common::powersupply::AbstractPowerSupply *ps= new common::powersupply::OcemE642X(dev.c_str(),id,maxcurrent,maxvoltage);
      if(ps==NULL){
	std::cout<<"## cannot initialize resources"<<std::endl;
	return -2;
	
      }
      std::cout<<"Trying address:"<<id<<std::endl;
      if(ps->init()==0){
    	  std::cout<<"found:"<<id<<std::endl;
    	  found ++;
      }
      delete ps;
    }
    return found;
  }
 /* if(vm.count("raw")){
    common::serial::OcemProtocol* oc= new common::serial::OcemProtocol(param.c_str());
    if(oc) 
      raw_test(oc);

    delete oc;
    return 0;
  }
*/
  

  
  printf("Connecting to slave %d, via \"%s\"... \n",slave_id,dev.c_str());

  common::powersupply::OcemE642X *ps= new common::powersupply::OcemE642X(dev.c_str(),slave_id,maxcurrent,maxvoltage);

  if(ps){
    std::cout<<"Initializing driver"<<std::endl;

    if(ps->init()!=0){
      printf("## cannot initialise power supply\n");
      return -1;
    }
  }
  printf("OK\n");
  if(interactive){
    if(  ps->getSWVersion(ver,DEFAULT_TIMEOUT)!=0){
      printf("## cannot get SW version \n");
      return -3;
    }
    printf("Version:%s\n",ver.c_str());
    while(1){
      float curr,volt,sp;
      int pol,stat;
      std::string state;
      int ch,ret;
      uint64_t ev;
      if((ret= ps->getCurrentOutput(&curr,DEFAULT_TIMEOUT))<0){
	printf("\n## error retrieving current, ret %d\n",ret);
      }
      if((ret=ps->getVoltageOutput(&volt,DEFAULT_TIMEOUT))<0){
	printf("\n## error retrieving voltage, ret %d\n",ret);
      }
      if((ret=ps->getCurrentSP(&sp,DEFAULT_TIMEOUT))<0){
	printf("\n## error retrieving current set point, ret %d\n",ret);
      }
      if((ret=ps->getPolarity(&pol,DEFAULT_TIMEOUT))<0){
	printf("\n## error retrieving polarity, ret %d\n",ret);
      }
      if((ret=ps->getState(&stat,state,DEFAULT_TIMEOUT))<0){
	printf("\n## error retrieving State, ret %d\n",ret);
      }
      if((ret=ps->getAlarms(&ev,DEFAULT_TIMEOUT))<0){
	printf("\n## error retrieving Alarms, ret %d\n",ret);
      } 
      printf("A:%3.4f,V:%2.2f,SetPoint('1'):%3.4f,Polarity('2'):%c,State('3'):(0x%x)\"%s\", alarms(reset '4'):x%llX\r",curr,volt,sp,(pol>0)?'+':(pol<0)?'-':'0',stat,state.c_str(),ev);
      fflush(stdout);
      if(check_for_char() && (ch=getchar())){
	if(ch == 'q'){
	  printf("quitting....\n");
	  break;
	}
	if(ch=='1'){
	  float max,min;
	  float cur;
	  ps->getMaxMinCurrent(&max,&min);
	  printf("\nSet current point [%3.4f:%3.4f]:",min,max);
	  scanf("%f",&cur);
	  printf("\n");
	  if( (ret=ps->setCurrentSP(cur,DEFAULT_TIMEOUT))<0){
	    printf("\n## error setting current ret %d\n",ret);
	  } else {
	    if( (ret=ps->startCurrentRamp(DEFAULT_TIMEOUT))<0){
	      printf("\n## error starting ramp%d\n",ret);
	    }
	  }
	}
      
	if(ch=='2'){
	  int polarity;
	  printf("\nSet Polarity [<0 negative,>0 positive, 0 open]:");
	  scanf("%d",&polarity);
	  printf("\n");
	  if( (ret=ps->setPolarity(polarity,DEFAULT_TIMEOUT))<0){
	    printf("\n## error setting polarity ret %d\n",ret);
	  } 
	}

	if(ch=='4'){
	  uint64_t alm=0;
	  printf("\nResetting events\n");

	  if( (ret=ps->resetAlarms(alm,DEFAULT_TIMEOUT))<0){
	    printf("\n## error resetting alarms ret %d\n",ret);
	  } 
	}
	    
	if(ch=='3'){
	  int polarity;
	  printf("\nSet State [1 Power ON, 2 StandBy , 3  Shutdown (the communincation could drop):");
	  scanf("%d",&polarity);
	  printf("\n");
	  if(polarity==1){
	    if( (ret=ps->poweron(DEFAULT_TIMEOUT))<0){
	      printf("\n## error setting power on ret %d\n",ret);
	    }
	  }
	
	  if(polarity==2){
	    if( (ret=ps->standby(DEFAULT_TIMEOUT))<0){
	      printf("\n## error setting standby ret %d\n",ret);
	    }
	  }
	
	  if(polarity==3){
	    if( (ret=ps->shutdown(DEFAULT_TIMEOUT))<0){
	      printf("\n## error setting OFF ret %d\n",ret);
	      
	    }
	  }
	
	}
      
      }
    }
  } else {
    char stringa[1024];
    char cmd[256],val[256],val1[256];
    printf("waiting commands (HELP for command list)\n");
    while(gets(stringa)){
      int ret;
      uint64_t tm;
      char *t=stringa;
      convToUpper(t);
      tm = common::debug::getUsTime();
      if(sscanf(stringa,"%s %s %s",cmd,val,val1)==3){
	if(!strcmp(cmd,"OPEN")){
	  if(ps!=NULL){
	    printf("## device already open \"CLOSE\" before\n");
	    continue;
	  } else {
	    ps= new common::powersupply::OcemE642X(val,atoi(val1));
	    if(ps==NULL){
	      printf("cannot allocate resources for \"%s:%s\"\n",val,val1);
	      continue;
	    } else {
	      printf("Connecting to slave %s, via \"%s\"... \n",val1,val);
	      if(ps->init()!=0){
		printf("## cannot initialise power supply on \"%s:%s\"\n",val,val1);
		delete ps;
		ps = NULL;
		continue;
	      }
	    }
	  }
	  
	} else if(!strcmp(cmd,"SLOPE")){
	  float up = atof(val);
	  float down=atof(val1);
	  printf("setting current SLOPE UP %4.4f DOWN %4.4f\n",up,down);
	  if( (ret=ps->setCurrentRampSpeed(up,down,DEFAULT_TIMEOUT))<0){
	    printf("## error setting SLOPE ret %d\n",ret);
	    continue;
	  } 
	} else {
	  printf("## syntax error, syntax is SLOPE <UP> <DOWN>\n"); 
	  continue;
	}
      
      } else if(sscanf(stringa,"%s %s",cmd,val)==2){
	int ival = atoi(val);
	float fval = atof(val);
        if(!strcmp(cmd,"SELECT")){
            int ret;
            ret=ps->send_command(val,1000,0);
            if(ret<0){
                printf("## error SELECT %s ret %d\n",val,ret);
                continue;
            } 
            
        } else if(!strcmp(cmd,"POL")){

	  printf("setting polarity to %d\n",ival);
	  if( (ret=ps->setPolarity(ival,DEFAULT_TIMEOUT))<0){
	    printf("## error setting polarity ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"SP")){
	  printf("setting current SP to %4.4f\n",fval);
	  if( (ret=ps->setCurrentSP(fval,DEFAULT_TIMEOUT))<0){
	    printf("## error setting SP ret %d\n",ret);
	    continue;
	  } 
	} else {
	  printf("## syntax error\n"); 
	  continue;
	}
      }  else if(sscanf(stringa,"%s",cmd)==1){
	if(!strcmp(cmd,"RAMP")){
	  printf("start ramp..\n");
	  if( (ret=ps->startCurrentRamp(DEFAULT_TIMEOUT))<0){
	    printf("## error starting ramp ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"STANDBY")){
	  printf("setting standby\n");
	  if( (ret=ps->standby(DEFAULT_TIMEOUT))<0){
	    printf("## error STANDBY ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"POLL")){
          char buffer[1024];
          int tim;
	  printf("executing poll\n");
	  if( (ret=ps->receive_data(buffer,sizeof(buffer),5000,&tim))<0){
	    printf("## error POLLING ret %d\n",ret);
	    continue;
	  }  else {
              printf("->\"%s\"\n",buffer);
          }
	} else if(!strcmp(cmd,"ON")){
	  printf("setting poweron\n");
	  if( (ret=ps->poweron(DEFAULT_TIMEOUT))<0){
	    printf("## error POWERON ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"OFF")){
	  printf("setting shutdown\n");
	  if( (ret=ps->shutdown(DEFAULT_TIMEOUT))<0){
	    printf("## error shutdown ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"RSTALARMS")){
	  printf("resetting alarms\n");
	  if( (ret=ps->resetAlarms(0,DEFAULT_TIMEOUT))<0){
	    printf("## error resetting allarms ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"GETCURRENT")){
	  float curr;
	  if( (ret=ps->getCurrentOutput(&curr,DEFAULT_TIMEOUT))<0){
	    printf("## error getting current ret %d\n",ret);
	    continue;
	  } else {
	    printf("* %4.4f\n",curr);
	  }
	} else if(!strcmp(cmd,"GETVOLTAGE")){
	  float curr;
	  if( (ret=ps->getVoltageOutput(&curr,DEFAULT_TIMEOUT))<0){
	    printf("## error getting voltage ret %d\n",ret);
	    continue;
	  } else {
	    printf("* %4.4f\n",curr);
	  }
	} else if(!strcmp(cmd,"GETALARMS")){
	  uint64_t curr;

	  if( (ret=ps->getAlarms(&curr,DEFAULT_TIMEOUT))<0){
	    printf("## error getting alarms ret %d\n",ret);
	    continue;
	  } else {
	    std::string ret=common::powersupply::AbstractPowerSupply::decodeEvent((common::powersupply::PowerSupplyEvents)curr);
	    printf("* %.16llx \"%s\"\n",curr,ret.c_str());
	  }
	} else if(!strcmp(cmd,"GETSTATE")){
	  int stat;
	  std::string desc;
	  if( (ret=ps->getState(&stat,desc,DEFAULT_TIMEOUT))<0){
	    printf("## error getting state ret %d\n",ret);
	    continue;
	  } else {
	    printf("* 0x%.8x (%s)\n",stat,desc.c_str());
	  }
	} else if(!strcmp(cmd,"GETPOL")){
	  int stat;

	  if( (ret=ps->getPolarity(&stat,DEFAULT_TIMEOUT))<0){
	    printf("## error getting polarity ret %d\n",ret);
	    continue;
	  } else {
	    printf("* \"%s\"\n",stat>0?"POS":stat<0?"NEG":"OPN");
	  }
	} else if(!strcmp(cmd,"GETVER")){
	  std::string desc;
	  if( (ret=ps->getHWVersion(desc,DEFAULT_TIMEOUT))<0){
	    printf("## error getting state ret %d\n",ret);
	    continue;
	  } else {
	    float cmax,cmin,vmax,vmin;
	    ps->getMaxMinCurrent(&cmax,&cmin);
	    ps->getMaxMinVoltage(&vmax,&vmin);
	    printf("* %s (max current %4.4f, max voltage %4.4f )\n",desc.c_str(),cmax,vmax);
	  }
	} else if(!strcmp(cmd,"GETSP")){
	  float curr;
	  if( (ret=ps->getCurrentSP(&curr,DEFAULT_TIMEOUT))<0){
	    printf("## error getting SP ret %d\n",ret);
	    continue;
	  } 
	  printf("* %4.4f\n",curr);
	} else if(!strcmp(cmd,"QUIT")){
	  printf("quitting\n");
	  break;
	} else if(!strcmp(cmd,"CLOSE")){
	  delete ps;
	  ps = NULL;

	} else if(!strcmp(cmd,"HELP")){
	  printCommandHelp();

	} else {
	  printf("## syntax error\n");
	  continue;
	}
      }
      printf("* OK [%.8llu ms]\n",(common::debug::getUsTime()-tm)/1000);
    }
  }
  delete ps;
  printf("* OK\n");
  return 0;
}

