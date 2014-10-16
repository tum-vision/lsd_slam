/*
 * Resolution.h
 *
 *  Created on: 18 Nov 2012
 *      Author: thomas
 */

#ifndef RESOLUTION_H_
#define RESOLUTION_H_

#include <cassert>

class Resolution
{
    public:
        static const Resolution & getInstance(int width = 0, int height = 0)
        {
            static const Resolution instance(width, height);
            return instance;
        }

        const int & width() const
        {
            return imgWidth;
        }

        const int & height() const
        {
            return imgHeight;
        }

        const int & cols() const
        {
            return imgWidth;
        }

        const int & rows() const
        {
            return imgHeight;
        }

        const int & numPixels() const
        {
            return imgNumPixels;
        }

    private:
        Resolution(int width, int height)
         : imgWidth(width),
           imgHeight(height),
           imgNumPixels(width * height)
        {
            assert(width > 0 && height > 0);
        }

        const int imgWidth;
        const int imgHeight;
        const int imgNumPixels;
};

#endif /* RESOLUTION_H_ */
