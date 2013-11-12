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

int main(int argc, char *argv[])
{


  boost::program_options::options_description desc("options");
  std::string ver;
  std::vector<int>::iterator slvid;
  std::map<int,common::powersupply::AbstractPowerSupply *> alims;
  std::map<int,common::powersupply::AbstractPowerSupply *>::iterator ialims;
  std::vector<int> option_id;
  desc.add_options()("help", "help");
  // put your additional options here
  desc.add_options()("dev", boost::program_options::value<std::string>(), "serial device where the ocem(s) are attached");
  desc.add_options()("id", boost::program_options::value<std::vector<int> >(&option_id)->multitoken(), "slaves ids");
  desc.add_options()("interactive", "interactive test");
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


  for(slvid=option_id.begin();slvid!= option_id.end();slvid++){
    common::powersupply::AbstractPowerSupply *ps= new common::powersupply::OcemE642X(param.c_str(),*slvid);
    if(ps==NULL){
      printf("## cannot created resources for alim %d\n",*slvid);
      return -2;
    }
    printf("Connecting to slave %d, via \"%s\"... \n",*slvid,param.c_str());
    sleep(1);
    if(ps->init()!=0){
      printf("## cannot initialise power supply  %d\n",*slvid);
      return -1;
    }
   
    if(  ps->getSWVersion(ver,DEFAULT_TIMEOUT)!=0){
      printf("## cannot get SW version for alim %d\n",*slvid);
      return -3;
    }
    printf("Power supply %d, Version:%s\n",*slvid,ver.c_str());
    alims[*slvid] = ps;
  }
  /*
  while(1){
    for(ialims=alims.begin();ialims!=alims.end();ialims++){
      ialims->second
    }
    }*/
  return 0;
}

