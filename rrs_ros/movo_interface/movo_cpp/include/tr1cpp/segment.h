#ifndef TR1CPP__SEGMENT_H
#define TR1CPP__SEGMENT_H

#include <tr1cpp/joint.h>

namespace tr1cpp
{
	template <int T> class Segment
	{
		private:

		public:
			Segment() { };
			~Segment() { };
			int size() const { return T; }
			Joint joints[T];
	};
}

#endif
