/*
 * RawLogReader.h
 *
 *  Created on: 19 Nov 2012
 *      Author: thomas
 */

#ifndef RAWLOGREADER_H_
#define RAWLOGREADER_H_

#include "Resolution.h"
#include "Stopwatch.h"

#include <cassert>
#include <zlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>
#include <boost/filesystem.hpp>

class RawLogReader
{
    public:
        RawLogReader(Bytef *& decompressionBuffer,
                     IplImage *& deCompImage,
                     std::string file);

        virtual ~RawLogReader();

        void getNext();

        int getNumFrames();

        bool hasMore();

        const std::string getFile();

        unsigned short minVal, maxVal;
        int64_t timestamp;

        unsigned short * depth;
        unsigned char * rgb;

    private:
        Bytef *& decompressionBuffer;
        IplImage *& deCompImage;
        unsigned char * depthReadBuffer;
        unsigned char * imageReadBuffer;
        int32_t depthSize;
        int32_t imageSize;

        const std::string file;
        FILE * fp;
        int32_t numFrames;
        int currentFrame;
        int width;
        int height;
        int numPixels;
};

#endif /* RAWLOGREADER_H_ */
