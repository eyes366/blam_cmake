#include "LoopClosureOpr.h"

int main(void)
{
	LoopClosureOpr LCOpr;
	LCOpr.ReadData("D:\\MyWork\\CloudPointMappingLocation\\trunk\\BlamSlam\\build\\InterMedia20180423113047_");
	LCOpr.StartLoopDetect();

	return 0;
}
