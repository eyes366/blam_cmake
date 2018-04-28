#include "ManualLoopDetector.h"

int main(void)
{
    ManualLoopDetector detector;
    detector.ReadData("/media/xinzi/新加卷/MyWork/CloudPointMappingLocation/trunk/BlamSlam/build/InterMedia20180423113047_");
    detector.Process();

	getchar();

	return 0;
}
