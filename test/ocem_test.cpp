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

  common::powersupply::OcemCore *oc= new common::powersupply::OcemCore(param.c_str(),slave_id);

  if(oc){
    if(oc->init()!=0){
      printf("## cannot initialise ocem device\n");
      return -1;
    }
  }

  std::string ver = oc->getSWVersion();
  if(ver.empty()==true){
    printf("## cannot get SW version \n");
    return -3;
  }
  printf("GetVersion:\"%s\"\n",ver.c_str());
  delete oc;
  return 0;
}

