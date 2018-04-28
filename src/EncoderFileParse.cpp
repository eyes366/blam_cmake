#include "EncoderFileParse.h"



EncoderFileParse::EncoderFileParse():fp(NULL)
{
}

EncoderFileParse::~EncoderFileParse()
{
    if(fp)
    {
        fclose(fp);
    }
}

int EncoderFileParse::openFile(const char *filename)
{
    if(fp)
        fclose(fp);

    fp = fopen(filename, "rb");
    if(!fp)
    {
        return -1;
    }
    return 0;
}


int EncoderFileParse::readData(EncoderData &data)
{
    if(fp)
    {
		if(feof(fp))
			return -1;
        fread(&data, sizeof(data), 1, fp);
        return 0;
    }
    return -1;
}

void EncoderFileParse::closeFile()
{
    if(fp)
    {
        fclose(fp);
        fp = 0;
    }
}
