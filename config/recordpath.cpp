
#include "stdafx.h"
#include "recordpath.h"

#include <fstream>

using namespace std;

namespace config {

	void RecordPath(const char * filename, double x, double y, double z, double roll, double yaw, double pitch)
	{
		ofstream fout(filename, ios::app);
		fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";
		fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";fout << "0 ";
		fout << x << " ";
		fout << y << " ";
		fout << z << " ";
		fout << roll << " ";
		fout << yaw << " ";
		fout << pitch << " ";
		fout << endl;
		fout.flush();
		fout.close();
	}

}
