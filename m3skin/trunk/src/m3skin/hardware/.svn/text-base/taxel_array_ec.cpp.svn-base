/* 
 M3 -- Meka Robotics Robot Components
 Copyright (c) 2010 Meka Robotics
 Author: edsinger@mekabot.com (Aaron Edsinger)

 M3 is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 M3 is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License
 along with M3.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <m3skin/hardware/taxel_array_ec.h>

namespace m3skin {

using namespace m3rt;
using namespace std;


const int M3TaxelArrayEc::taxel_mapping[TAXEL_PER_BRANCH] = {
  152,153,155,164,165,167,176,177,179,188,189,191,
  151,154,156,163,166,168,175,178,180,187,190,192,
  148,145,150,160,157,162,172,169,174,184,181,186,
  149,146,147,161,158,159,173,170,171,185,182,183,
  104,105,107,116,117,119,128,129,131,140,141,143,
  103,106,108,115,118,120,127,130,132,139,142,144,
  100, 97,102,112,109,114,124,121,126,136,133,138,
  101, 98, 99,113,110,111,125,122,123,137,134,135,
   56, 57, 59, 68, 69, 71, 80, 81, 83, 92, 93, 95,
   55, 58, 60, 67, 70, 72, 79, 82, 84, 91, 94, 96,
   52, 49, 54, 64, 61, 66, 76, 73, 78, 88, 85, 90,
   53, 50, 51, 65, 62, 63, 77, 74, 75, 89, 86, 87,
    8,  9, 11, 20, 21, 23, 32, 33, 35, 44, 45, 47,
    7, 10, 12, 19, 22, 24, 31, 34, 36, 43, 46, 48,
    4,  1,  6, 16, 13, 18, 28, 25, 30, 40, 37, 42,
    5,  2,  3, 17, 14, 15, 29, 26, 27, 41, 38, 39
};

void M3TaxelArrayEc::Startup() {
      
	for (int i = 0; i < NUM_TAXEL; i++) {
		status.add_taxel_value(0);		
	}
	SetStateSafeOp();
}

int M3TaxelArrayEc::TaxelLocationLookup(int tax_idx)
{
    int idx = 0;
    for (int i = 0; i < TAXEL_PER_BRANCH; i++)
    {
	if (taxel_mapping[i]-1 == tax_idx)
	  idx = i;
    }
    return idx;
}

void M3TaxelArrayEc::SetStatusFromPdo(unsigned char * data) {
	if (IsPdoVersion(TAXEL_ARRAY_PDO_V0)) {
		// Cast the raw buffer into the status
		M3TaxelArrayPdoV0Status * ec = (M3TaxelArrayPdoV0Status *) data;

		status.set_timestamp(GetBaseStatus()->timestamp());//Not provided by DSP for this version		
		//Pass in EXT data
		int ext_sz = sizeof(M3TaxelArrayPdoV0StatusExt);
		int num_copy=MIN(M3TAX_PDO_V0_STATUS_EXT_SZ,ext_sz); //max num to copy 
		int start = ec->ext_start_idx; 
		int end = (start+num_copy-1)%ext_sz; 
		if (end<start) //handle roll-over
		{
		  int nc1=ext_sz-start; 
		  int nc2=num_copy-nc1; 
		  memcpy(((unsigned char *)&exs)+start,(unsigned char *) ec->ext,nc1);
		  memcpy((unsigned char *)&exs,(unsigned char *)ec->ext+nc1,nc2);
		}
		else
		  memcpy((unsigned char *)&exs+start,(unsigned char *) ec->ext,num_copy);
	      		
		int j = 0;
		status.set_debug(exs.debug);
		int branch_idx = 0;
		for (int i = 0; i < NUM_TAXEL; i++) {			
			//status.set_taxel_value(branch_idx*TAXEL_PER_BRANCH + (taxel_mapping[j]-1), exs.value[i]);
			status.set_taxel_value(branch_idx*TAXEL_PER_BRANCH + TaxelLocationLookup(j),  (unsigned int)((unsigned short)exs.value[i]));
			//status.set_taxel_value(i, (unsigned int)((unsigned short)exs.value[i]));
			//status.set_taxel_value(i, tmp_cnt++);			
			j++;
			if (j >= TAXEL_PER_BRANCH)
			{
			  branch_idx++;
			  j = 0;
			}
		}
	}
}



void M3TaxelArrayEc::SetPdoFromCommand(unsigned char * data) {
	if (IsPdoVersion(TAXEL_ARRAY_PDO_V0)) {
		M3TaxelArrayPdoV0Cmd * ec = (M3TaxelArrayPdoV0Cmd *) data;
		ec->config = param.config();
	}
}

bool M3TaxelArrayEc::ReadConfig(const char * filename) {
	if (!M3ComponentEc::ReadConfig(filename))
		return false;
//	YAML::Node doc;
//	GetYamlDoc(filename, doc);
//	int val;
//	doc["param"]["config"] >> val;
//	param.set_config(val);
	return true;
}

bool M3TaxelArrayEc::LinkDependentComponents()
{
	return true;
}

void M3TaxelArrayEc::Shutdown()
{

}

}

