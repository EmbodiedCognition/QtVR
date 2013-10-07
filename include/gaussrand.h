/****************************************************************************
 *                                                                          *
 * QtVR--Physics-base inverse kinematics and inverse dynamics               *
 * Copyright (c) 2013 Joseph Cooper                                         *
 *                                                                          *
 * This software is provided 'as-is', without any express or implied        *
 * warranty. In no event will the authors be held liable for any damages    *
 * arising from the use of this software.                                   *
 *                                                                          *
 * Permission is granted to anyone to use this software for any purpose,    *
 * including commercial applications, and to alter it and redistribute it   *
 * freely, subject to the following restrictions:                           *
 *                                                                          *
 *  1. The origin of this software must not be misrepresented; you must not *
 *  claim that you wrote the original software. If you use this software    *
 *  in a product, an acknowledgment in the product documentation would be   *
 *  appreciated but is not required.                                        *
 *                                                                          *
 *  2. Altered source versions must be plainly marked as such, and must not *
 *  be misrepresented as being the original software.                       *
 *                                                                          *
 *  3. This notice may not be removed or altered from any source            *
 *  distribution.                                                           *
 ****************************************************************************/
#ifndef GAUSSRAND_H
#define GAUSSRAND_H

/* Boost headers are for sampling from
   a Gaussian distribution to perturb marker
   positions
 */
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>

/**
 * @brief The GaussRand class encapsulates Gaussian Random generator from Boost
 *
 * There's a macro expansion bug in Qt that makes it so you can't include
 * some Boost files in a file that will go through the MOC system.  So
 * I pulled the boost references into their own place.
 */
class GaussRand
{
public:
  GaussRand() :
    randN(rng,norm)
  {
  }

  double next() {return randN();}

  /* Random number generator objects for Gaussian samples. */
  boost::mt19937 rng;
  boost::normal_distribution<double> norm;
  boost::variate_generator<boost::mt19937&,boost::normal_distribution<double> > randN;
};

#endif // GAUSSRAND_H
