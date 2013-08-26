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
#ifndef JOINTDATAFRAME_H
#define JOINTDATAFRAME_H

#include <QObject>


#include "CapBody.h"

class JointDataFrame : public QObject
{
    Q_OBJECT
public:
    explicit JointDataFrame(QObject *parent = 0);

  double getJointData(int jj,int aa) {return data[jj][aa];}
  void setJointData(int jj,int aa,double val) {data[jj][aa]=val;}

signals:

public slots:

protected:
  double data[CapBody::JOINT_COUNT][3];

};

#endif // JOINTDATAFRAME_H
