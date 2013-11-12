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
#define DEFAULT_TIMEOUT 10000
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
    std::cout<<" Available commands:"<<std::endl;
    std::cout<<"\tPOL <1/0/-1> : set polarity"<<std::endl;
    std::cout<<"\tSP <float>   : set SetPoint"<<std::endl;
    std::cout<<"\tRAMP         : start ramp"<<std::endl;
    std::cout<<"\tRSTALARMS    : reset alarms"<<std::endl;
    std::cout<<"\t--------------------------"<<std::endl;
    std::cout<<"\tSTANDBY      : force standby"<<std::endl;
    std::cout<<"\tON           : force poweron"<<std::endl;
    std::cout<<"\tOFF          : force powerof"<<std::endl;
    std::cout<<"\t----------------------------"<<std::endl;
    std::cout<<"\tGETCURRENT   : get current"<<std::endl;
    std::cout<<"\tGETVOLTAGE   : get voltage"<<std::endl;
    std::cout<<"\tGETALARMS    : get alarms"<<std::endl;
    std::cout<<"\tGETSTAT      : get state"<<std::endl;
    std::cout<<"\tGETSP        : get SetPoint"<<std::endl;
    std::cout<<"\tGETVER       : get HW/SW version"<<std::endl;
    std::cout<<"\tGETPOL       : get polarity"<<std::endl;
    std::cout<<"\t----------------------------"<<std::endl;
    std::cout<<"\tHELP         : this help"<<std::endl;
    std::cout<<"\tQUIT         : quit program"<<std::endl;

}
int main(int argc, char *argv[])
{


  boost::program_options::options_description desc("options");
  std::string ver;
  desc.add_options()("help", "help");
  // put your additional options here
  desc.add_options()("dev", boost::program_options::value<std::string>(), "serial device where the ocem(s) are attached");
  desc.add_options()("id", boost::program_options::value<std::string>(), "slaves ids (comma separated)");
  desc.add_options()("interactive", "interactive test");
  //////
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc,argv, desc),vm);
  boost::program_options::notify(vm);
    
  if (vm.count("help")) {
    std::cout << desc << "\n";
    printCommandHelp();
    return 1;
  }
  if(vm.count("dev")==0){
    std::cout<<"## you must specify a valid serial device:"<<desc<<std::endl;
    return -1;
  }
  std::string param = vm["dev"].as<std::string>();


  if(vm.count("id")==0){
    std::cout<<"## you must specify an existing slave id [0:31]"<<desc<<std::endl;
    return -1;
  }
  int slave_id = vm["id"].as<int>();


  printf("Connecting to slave %d, via \"%s\"... \n",slave_id,param.c_str());

  common::powersupply::AbstractPowerSupply *ps= new common::powersupply::OcemE642X(param.c_str(),slave_id);

  if(ps){
    if(ps->init()!=0){
      printf("## cannot initialise power supply\n");
      return -1;
    }
  }
  printf("OK\n");
  if(vm.count("interactive")){
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
    char cmd[256],val[256];
    printf("waiting commands (HELP for command list)\n");
    while(gets(stringa)){
      int ret;
      
      if(sscanf(stringa,"%s %s",cmd,val)==2){
	int ival = atoi(val);
	float fval = atof(val);
	if(!strcmp(cmd,"POL")){

	  printf("setting polarity to %d :",ival);
	  if( (ret=ps->setPolarity(ival,DEFAULT_TIMEOUT))<0){
	    printf("## error setting polarity ret %d\n",ret);
	  } else {
	    printf("OK\n");
	  }
	} else if(!strcmp(cmd,"SP")){
	  printf("setting current SP to %f :",fval);
	  if( (ret=ps->setCurrentSP(fval,DEFAULT_TIMEOUT))<0){
	    printf("## error setting SP ret %d\n",ret);
	  } else {
	    printf("OK\n");
	  }
	} else {
	  printf("## syntax error\n"); 
	}
      } else if(sscanf(stringa,"%s",cmd)==1){
	if(!strcmp(cmd,"RAMP")){
	  printf("start ramp..");
	  if( (ret=ps->startCurrentRamp(DEFAULT_TIMEOUT))<0){
	    printf("## error starting ramp ret %d\n",ret);
	  } else {
	    printf("OK\n");
	  }
	} else if(!strcmp(cmd,"STANDBY")){
	  printf("setting stand by ");
	  if( (ret=ps->standby(DEFAULT_TIMEOUT))<0){
	    printf("## error STANDBY ret %d\n",ret);
	  } else {
	    printf("OK\n");
	  }
	} else if(!strcmp(cmd,"ON")){
	  printf("setting poweron ");
	  if( (ret=ps->poweron(DEFAULT_TIMEOUT))<0){
	    printf("## error POWERON ret %d\n",ret);
	  } else {
	    printf("OK\n");
	  }
	} else if(!strcmp(cmd,"OFF")){
	  printf("setting shutdown ");
	  if( (ret=ps->shutdown(DEFAULT_TIMEOUT))<0){
	    printf("## error shutdown ret %d\n",ret);
	  } else {
	    printf("OK\n");
	  }
	} else if(!strcmp(cmd,"RSTALARMS")){
	  printf("reset alarms ");
	  if( (ret=ps->resetAlarms(0,DEFAULT_TIMEOUT))<0){
	    printf("## error shutdown ret %d\n",ret);
	  } else {
	    printf("OK\n");
	  }
	} else if(!strcmp(cmd,"GETCURRENT")){
	  float curr;
	  printf("get current ");
	  if( (ret=ps->getCurrentOutput(&curr,DEFAULT_TIMEOUT))<0){
	    printf("## error getting current ret %d\n",ret);
	  } else {
	    printf("%f OK\n",curr);
	  }
	} else if(!strcmp(cmd,"GETVOLTAGE")){
	  float curr;
	  printf("get voltage ");
	  if( (ret=ps->getVoltageOutput(&curr,DEFAULT_TIMEOUT))<0){
	    printf("## error getting voltage ret %d\n",ret);
	  } else {
	    printf("%f OK\n",curr);
	  }
	} else if(!strcmp(cmd,"GETALARMS")){
	  uint64_t curr;
	  printf("get alarms ");
	  if( (ret=ps->getAlarms(&curr,DEFAULT_TIMEOUT))<0){
	    printf("## error getting alarms ret %d\n",ret);
	  } else {
	    printf("%16llx OK\n",curr);
	  }
	} else if(!strcmp(cmd,"GETSTATE")){
	  int stat;
	  std::string desc;
	  printf("get state ");
	  if( (ret=ps->getState(&stat,desc,DEFAULT_TIMEOUT))<0){
	    printf("## error getting state ret %d\n",ret);
	  } else {
	    printf("0x%x (%s) OK\n",stat,desc.c_str());
	  }
	} else if(!strcmp(cmd,"GETPOL")){
	  int stat;
	  printf("get polarity");
	  if( (ret=ps->getPolarity(&stat,DEFAULT_TIMEOUT))<0){
	    printf("## error getting polarity ret %d\n",ret);
	  } else {
	    printf("%s OK\n",stat>0?"POS":stat<0?"NEG":"OPN");
	  }
	} else if(!strcmp(cmd,"GETVER")){
	  std::string desc;
	  printf("get version ");
	  if( (ret=ps->getHWVersion(desc,DEFAULT_TIMEOUT))<0){
	    printf("## error getting state ret %d\n",ret);
	  } else {
	    float cmax,cmin,vmax,vmin;
	    ps->getMaxMinCurrent(&cmax,&cmin);
	    ps->getMaxMinVoltage(&vmax,&vmin);
	    printf("%s (max current %f, max voltage %f ) OK\n",desc.c_str(),cmax,vmax);
	  }
	} else if(!strcmp(cmd,"GETSP")){
	  float curr;
	  printf("get SP ");
	  if( (ret=ps->getCurrentSP(&curr,DEFAULT_TIMEOUT))<0){
	    printf("## error getting SP ret %d\n",ret);
	  } else {
	    printf("%f OK\n",curr);
	  }
	} else if(!strcmp(cmd,"QUIT")){
	  printf("quitting\n");
	  break;
	} else if(!strcmp(cmd,"HELP")){
	  printCommandHelp();

	} else {
	  printf("## syntax error\n"); 
	}
      }
    }
  }
  delete ps;
  return 0;
}

