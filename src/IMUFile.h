#ifndef _IMU_FILE_H
#define _IMU_FILE_H
#include <stdio.h>

struct IMUData
{
	unsigned int week;//系统时间
	double sec;
	float gyroscopes[3];
	float accelerometers[3];
	float magnetometers[3];
	unsigned int utcweek;//传感器时间
	double utcsec;
};

class IMUFileParse
{
public:
	IMUFileParse();
	~IMUFileParse();
	int OpenFile(const char *filename);

	int ReadData(IMUData &data);

	void CloseFile();
private:
    FILE *_fp;
};

IMUFileParse::IMUFileParse():_fp(0)
{
}

IMUFileParse::~IMUFileParse()
{
}
int IMUFileParse::OpenFile(const char *filename)
{
	if(_fp)
	{
		fclose(_fp);
		_fp = 0;
	}
	_fp = fopen(filename, "rb");
	if(!_fp)
		return -1;
	return 0;
}

int IMUFileParse::ReadData(IMUData &data)
{
	if(_fp)
	{
		if(feof(_fp))
			return -1;
		fread(&data, sizeof(data), 1, _fp);
		return 0;
	}
	return -1;
}

void IMUFileParse::CloseFile()
{
	if(_fp)
	{
		fclose(_fp);
		_fp = 0;
	}
}
#endif
