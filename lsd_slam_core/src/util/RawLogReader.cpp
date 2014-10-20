/*
 * RawLogReader.cpp
 *
 *  Created on: 19 Nov 2012
 *      Author: thomas
 */

#include "RawLogReader.h"

RawLogReader::RawLogReader(Bytef *& decompressionBuffer,
                           IplImage *& deCompImage,
                           std::string file)
 : timestamp(0),
   decompressionBuffer(decompressionBuffer),
   deCompImage(deCompImage),
   file(file),
   width(Resolution::getInstance().width()),
   height(Resolution::getInstance().height()),
   numPixels(width * height)
{
    assert(boost::filesystem::exists(file.c_str()));

    fp = fopen(file.c_str(), "rb");

    currentFrame = 0;

    assert(fread(&numFrames, sizeof(int32_t), 1, fp));

    depthReadBuffer = new unsigned char[numPixels * 2];
    imageReadBuffer = new unsigned char[numPixels * 3];
}

RawLogReader::~RawLogReader()
{
    delete [] depthReadBuffer;
    delete [] imageReadBuffer;

    if(deCompImage != 0)
    {
        cvReleaseImage(&deCompImage);
    }

    fclose(fp);
}

void RawLogReader::getNext()
{
    assert(fread(&timestamp, sizeof(int64_t), 1, fp));

    assert(fread(&depthSize, sizeof(int32_t), 1, fp));
    assert(fread(&imageSize, sizeof(int32_t), 1, fp));

    assert(fread(depthReadBuffer, depthSize, 1, fp));

    if(imageSize > 0)
    {
        assert(fread(imageReadBuffer, imageSize, 1, fp));
    }

    if(depthSize == numPixels * 2)
    {
        memcpy(&decompressionBuffer[0], depthReadBuffer, numPixels * 2);
    }
    else
    {
        unsigned long decompLength = numPixels * 2;
        uncompress(&decompressionBuffer[0], (unsigned long *)&decompLength, (const Bytef *)depthReadBuffer, depthSize);
    }

    unsigned short * depthBuffer = (unsigned short *)&decompressionBuffer[0];
    unsigned short maxVal = 0;
    unsigned short minVal = std::numeric_limits<unsigned short>::max();

    #pragma omp parallel for reduction(max : maxVal) reduction(min : minVal)
    for(int i = 0; i < numPixels; i++)
    {
        if(depthBuffer[i] > maxVal)
        {
            maxVal = depthBuffer[i];
        }

        if(depthBuffer[i] < minVal && depthBuffer[i] != 0)
        {
            minVal = depthBuffer[i];
        }
    }

    this->maxVal = maxVal;
    this->minVal = minVal;

    if(deCompImage != 0)
    {
        cvReleaseImage(&deCompImage);
    }

    CvMat tempMat = cvMat(1, imageSize, CV_8UC1, (void *)imageReadBuffer);

    if(imageSize == numPixels * 3)
    {
        deCompImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

        memcpy(deCompImage->imageData, imageReadBuffer, numPixels * 3);
    }
    else if(imageSize > 0)
    {
        deCompImage = cvDecodeImage(&tempMat);
    }
    else
    {
        deCompImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
        memset(deCompImage->imageData, 0, numPixels * 3);
    }

    depth = (unsigned short *)decompressionBuffer;
    rgb = (unsigned char *)deCompImage->imageData;

    currentFrame++;
}

int RawLogReader::getNumFrames()
{
    return numFrames;
}

bool RawLogReader::hasMore()
{
    return currentFrame + 1 < numFrames;
}

const std::string RawLogReader::getFile()
{
    return file;
}
