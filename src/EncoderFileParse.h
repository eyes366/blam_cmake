#ifndef ENCODERFILE_H
#define ENCODERFILE_H
#include <stdio.h>

struct EncoderData
{
    unsigned int week;
    double sec;
    int speed[2];
};

class EncoderFileParse
{
public:
    EncoderFileParse();
    ~EncoderFileParse();
    int openFile(const char *filepath);

    int readData(EncoderData &data);

    void closeFile();
private:
    FILE *fp;
};

#endif // ENCODERFILE_H
