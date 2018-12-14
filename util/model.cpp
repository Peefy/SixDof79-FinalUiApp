
#include "stdafx.h"
#include "model.h"

namespace util
{
	void BaseModel::Dispose()
	{
		this->IsDisposed = true;
	}
 
	double MaxValue(double * vals, int valnum)
	{
		double max = vals[0];
		for (int i = 0;i < valnum;++i)
		{
			if (vals[i] > max)
				max = vals[i];
		}
		return max;
	}

	double PutRadIn(double rad, double minrad, double maxrad)
	{
		while (rad <= minrad)
			rad += 2 * 3.1415926;
		while (rad >= maxrad)
			rad -= 2 * 3.1415926;
		return rad;
	}
}
