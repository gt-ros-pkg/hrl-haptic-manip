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

#ifndef M3_TAXEL_ARRAY_VIRTUAL_H
#define M3_TAXEL_ARRAY_VIRTUAL_H

#include <m3skin/hardware/taxel_array.h>

namespace m3skin
{
	using namespace std;
	using namespace ros;

class M3TaxelArrayVirtual : public M3TaxelArray
{
	public:
		M3TaxelArrayVirtual() {}

	protected:
		void Startup();
		void Shutdown();
		void StepStatus();
		void StepCommand();
		bool LinkDependentComponents();
		int tmp_cnt;
};


}

#endif


