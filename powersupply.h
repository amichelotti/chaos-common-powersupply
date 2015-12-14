/*	
 *	powersupply.h
 *	!CHAOS
 *	Created by automatically 
 *	
 *    	Copyright 2012 INFN, National Institute of Nuclear Physics
 *
 *    	Licensed under the Apache License, Version 2.0 (the "License");
 *    	you may not use this file except in compliance with the License.
 *    	You may obtain a copy of the License at
 *
 *    	http://www.apache.org/licenses/LICENSE-2.0
 *
 *    	Unless required by applicable law or agreed to in writing, software
 *    	distributed under the License is distributed on an "AS IS" BASIS,
 *    	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    	See the License for the specific language governing permissions and
 *    	limitations under the License.
 */
#ifndef __common_powersupply_h__
#define __common_powersupply_h__


// include your class/functions headers here
#include <stdint.h>
#include <string>
#include <common/powersupply/core/AbstractPowerSupply.h>
#include <common/powersupply/models/Ocem/OcemE642X.h>
#include <common/powersupply/models/Simulator/SimPSupply.h>

#define PSLAPP		LAPP_ << "[PowerSupply] "
#define PSDBG		LDBG_ << "[PowerSupply "<<__PRETTY_FUNCTION__<<" ]"
#define PSERR		LERR_ << "[PowerSupply "<<__PRETTY_FUNCTION__<<" ]"

namespace common {
  namespace powersupply {
};
};
#endif
