/* 
 * File:   test.cpp
 * Author: alex
 *
 * Created on 23 novembre 2015, 12.11
 */

#include <cstdlib>
#include "AL250PowerSupply.h"

using namespace std;
using namespace common;
using namespace common::powersupply;

/*
 * 
 */
int main(int argc, char** argv) {

    int ret;
    AL250 Unit("/dev/ttyr0f,2,9600,N,8,1",0);
  //  cout << Unit.getConnectionParameters() << endl;
    AL250 Unit2("/dev/ttyr0f,2,9600,N,8,1",1);
  //  cout << Unit2.getConnectionParameters() << endl;
    
    Unit.init();
    Unit2.init();
    
    cout << Unit.getConnectionParameters() << endl;
    cout << Unit2.getConnectionParameters() << endl;
    
    
    ret=true;
    return 0;
}

