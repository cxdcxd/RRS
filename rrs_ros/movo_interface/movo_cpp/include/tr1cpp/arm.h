#ifndef TR1CPP__ARM_H
#define TR1CPP__ARM_H

#include <tr1cpp/joint.h>

namespace tr1cpp
{
	class Arm
	{
		private:

		public:
			Arm();
			~Arm();
			Joint joints[8];
	};
}

#endif
