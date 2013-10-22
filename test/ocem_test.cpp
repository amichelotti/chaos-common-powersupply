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
int main(int argc, char *argv[])
{


  boost::program_options::options_description desc("options");

  desc.add_options()("help", "help");
  // put your additional options here
  desc.add_options()("dev", boost::program_options::value<std::string>(), "serial device where the ocem is attached");
  desc.add_options()("id", boost::program_options::value<int>(), "slave destination ID");
  //////
  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc,argv, desc),vm);
  boost::program_options::notify(vm);
    
  if (vm.count("help")) {
    std::cout << desc << "\n";
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

  std::string ver = ps->getSWVersion();
  if(ver.empty()==true){
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
    if((ret= ps->getCurrentOutput(&curr))<0){
      //      printf("## error retrieving current, ret %d\n",ret);
    }
    if((ret=ps->getVoltageOutput(&volt))<0){
      //      printf("## error retrieving voltage, ret %d\n",ret);
    }
    if((ret=ps->getCurrentSP(&sp))<0){
      //      printf("## error retrieving current set point, ret %d\n",ret);
    }
    if((ret=ps->getPolarity(&pol))<0){
      //      printf("## error retrieving polarity, ret %d\n",ret);
    }
    if((ret=ps->getState(&stat,state))<0){
      //      printf("## error retrieving State, ret %d\n",ret);
    }
    if((ret=ps->getAlarms(&ev))<0){
      //      printf("## error retrieving Alarms, ret %d\n",ret);
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
	if( (ret=ps->setCurrentSP(cur))<0){
	  printf("## error setting current ret %d\n",ret);
      } else {
	  if( (ret=ps->startCurrentRamp())<0){
	    printf("## error starting ramp%d\n",ret);
	  }
	}
      }
      
      if(ch=='2'){
	int polarity;
	printf("\nSet Polarity [<0 negative,>0 positive, 0 open]:");
	scanf("%d",&polarity);
	printf("\n");
	if( (ret=ps->setPolarity(polarity))<0){
	  printf("## error setting polarity ret %d\n",ret);
	} 
      }

      if(ch=='4'){
	uint64_t alm=0;
	printf("\nResetting events\n");

	if( (ret=ps->resetAlarms(alm))<0){
	  printf("## error resetting alarms ret %d\n",ret);
	} 
      }
	    
      if(ch=='3'){
	int polarity;
	printf("\nSet State [1 Power ON, 2 StandBy , 3  Shutdown (the communincation could drop):");
	scanf("%d",&polarity);
	printf("\n");
	if(polarity==1){
	if( (ret=ps->poweron())<0){
	  printf("## error setting power on ret %d\n",ret);
	}
	}
	
	if(polarity==2){
	  if( (ret=ps->standby())<0){
	    printf("## error setting standby ret %d\n",ret);
	  }
	}
	
	if(polarity==3){
	  if( (ret=ps->shutdown())<0){
	    printf("## error setting OFF ret %d\n",ret);
	  }
	}
	
      }
      
    }
  }
  delete ps;
  return 0;
}

