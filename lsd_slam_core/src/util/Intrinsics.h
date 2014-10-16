/*
 * Intrinsics.h
 *
 *  Created on: 18 Nov 2012
 *      Author: thomas
 */

#ifndef INTRINSICS_H_
#define INTRINSICS_H_

#include <cassert>

class Intrinsics
{
    public:
        static const Intrinsics & getInstance(float fx = 0, float fy = 0, float cx = 0, float cy = 0)
        {
            static const Intrinsics instance(fx, fy, cx, cy);
            return instance;
        }

        const float & fx() const
        {
            return fx_;
        }

        const float & fy() const
        {
            return fy_;
        }

        const float & cx() const
        {
            return cx_;
        }

        const float & cy() const
        {
            return cy_;
        }

    private:
        Intrinsics(float fx, float fy, float cx, float cy)
         : fx_(fx),
           fy_(fy),
           cx_(cx),
           cy_(cy)
        {
        }

        const float fx_, fy_, cx_, cy_;
};

#endif /* INTRINSICS_H_ */
