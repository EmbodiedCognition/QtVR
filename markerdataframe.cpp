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
#include "markerdataframe.h"

MarkerDataFrame::MarkerDataFrame(QObject *parent) :
    QObject(parent)
{
  for (int ii=0;ii<50;++ii) {
    data[ii][0] = 0;
    data[ii][1] = 0;
    data[ii][2] = 0;
    data[ii][3] = -1;
  }
}

void MarkerDataFrame::setPoint(int mark,const float* dat)
{
  data[mark][0]=double(dat[0]);
  data[mark][1]=double(dat[1]);
  data[mark][2]=double(dat[2]);
  data[mark][3]=double(dat[3]);
}

void MarkerDataFrame::setPoint(int mark,const double* dat)
{
  data[mark][0]=(dat[0]);
  data[mark][1]=(dat[1]);
  data[mark][2]=(dat[2]);
  data[mark][3]=(dat[3]);
}

void MarkerDataFrame::setPoint(int mark,double xx,double yy,double zz,double cc)
{
  data[mark][0]=xx;
  data[mark][1]=yy;
  data[mark][2]=zz;
  data[mark][3]=cc;
}
