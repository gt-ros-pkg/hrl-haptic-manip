/* 
MEKA CONFIDENTIAL

Copyright 2011 
Meka Robotics LLC
All Rights Reserved.

NOTICE:  All information contained herein is, and remains
the property of Meka Robotics LLC. The intellectual and 
technical concepts contained herein are proprietary to 
Meka Robotics LLC and may be covered by U.S. and Foreign Patents,
patents in process, and are protected by trade secret or copyright law.
Dissemination of this information or reproduction of this material
is strictly forbidden unless prior written permission is obtained
from Meka Robotics LLC.
*/
 
#include <stdio.h>
#include <m3rt/base/component.h>
#include <m3skin/hardware/taxel_array.h>
#include <m3skin/hardware/taxel_array_ec.h>
#include <m3skin/hardware/taxel_array_virtual.h>

extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 

#define M3TAXEL_ARRAY_TYPE_NAME "m3taxel_array"
#define M3TAXEL_ARRAY_EC_TYPE_NAME "m3taxel_array_ec"
#define M3TAXEL_ARRAY_VIRTUAL_TYPE_NAME "m3taxel_array_virtual"

//Creators
m3rt::M3Component* create_m3taxel_array(){return new m3skin::M3TaxelArray;}
m3rt::M3Component* create_m3taxel_array_ec(){return new m3skin::M3TaxelArrayEc;}
m3rt::M3Component* create_m3taxel_array_virtual(){return new m3skin::M3TaxelArrayVirtual;}

//Deletors
void destroy_m3taxel_array(m3rt::M3Component* c) {delete c;}
void destroy_m3taxel_array_ec(m3rt::M3Component* c) {delete c;}
void destroy_m3taxel_array_virtual(m3rt::M3Component* c) {delete c;}

class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[M3TAXEL_ARRAY_EC_TYPE_NAME] = create_m3taxel_array_ec;
		m3rt::destroyer_factory[M3TAXEL_ARRAY_EC_TYPE_NAME] =  destroy_m3taxel_array_ec;

		m3rt::creator_factory[M3TAXEL_ARRAY_TYPE_NAME] = create_m3taxel_array;
		m3rt::destroyer_factory[M3TAXEL_ARRAY_TYPE_NAME] =  destroy_m3taxel_array;

		m3rt::creator_factory[M3TAXEL_ARRAY_VIRTUAL_TYPE_NAME] = create_m3taxel_array_virtual;
		m3rt::destroyer_factory[M3TAXEL_ARRAY_VIRTUAL_TYPE_NAME] =  destroy_m3taxel_array_virtual;
	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
