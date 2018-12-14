#ifndef __MODEL_H_
#define __MODEL_H_

#include <string>

using namespace std;

namespace util
{
	class IDispose
	{
	public:
		virtual void Dispose() const;
		bool IsDisposed;

	private:

	};

	class BaseModel : public IDispose
	{
	public:
		BaseModel();
		~BaseModel();
		virtual void Dispose();
		string Title;
		int Id;
	};
	double MaxValue(double * vals, int valnum);
	double PutRadIn(double rad, double minrad = 0, double maxrad = 2 * 3.1415926);
}

#endif

