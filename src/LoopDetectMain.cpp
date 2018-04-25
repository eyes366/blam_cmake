#include "LoopClosureOpr.h"

int main(void)
{
	LoopClosureOpr LCOpr;
//	LCOpr.ReadData("D:\\MyWork\\CloudPointMappingLocation\\trunk\\BlamSlam\\build\\InterMedia20180423113047_");
LCOpr.ReadData("/media/xinzi/新加卷/MyWork/CloudPointMappingLocation/trunk/BlamSlam/build/InterMedia20180423113047_");
	LCOpr.StartLoopDetect();

	getchar();

	return 0;
}
