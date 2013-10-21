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
#include "quatcamera.h"

QuatCamera::QuatCamera()
{
  reset();
}


QVector3D QuatCamera::getPosition()
{
  return position;
}

QQuaternion QuatCamera::getQuaternion()
{
  return orient;
}


/**
  Put the forward transform into m[16]
  as a row-major matrix.
  */
void QuatCamera::getRMajorTrans(double* m)
{
  quatToRowMajorMat(orient,m);
  // The forward transform simply puts
  // the position into place.
  m[3] = position.x();
  m[7] = position.y();
  m[11]= position.z();
}

void QuatCamera::getCMajorTrans(double* m)
{
  // Using the conjugate produces
  // the transpose representation
  quatToRowMajorMat(orient.conjugate(),m);
  // The forward transform simply puts
  // the position into place.
  m[12] = position.x();
  m[13] = position.y();
  m[14] = position.z();
}

void QuatCamera::getRMajorInvTrans(double* m)
{
  quatToRowMajorMat(orient.conjugate(),m);
  // When we invert the transform,
  // it swaps the order of the
  // matrices; so the translation
  // component is swapped.
  QVector3D pos = orient.rotatedVector(position);
  m[3] = -pos.x();
  m[7] = -pos.y();
  m[11]= -pos.z();

}

void QuatCamera::getCMajorInvTrans(double* m)
{
  // To invert, we'd normally take the conjugate,
  // instead, we'll just use the transpose
  // of the matrix in memory.
  quatToRowMajorMat(orient,m);

  QVector3D pos = orient.conjugate().rotatedVector(position);
  m[12] = -pos.x();
  m[13] = -pos.y();
  m[14] = -pos.z();
}

void QuatCamera::quatToRowMajorMat(const QQuaternion& q, double* m)
{
  // Column 1 would be: q*x*q^-1
  // That's [w x y z]*[0 1 0 0]*[w -x -y -z]
  // using quaternion multiplication

  // [w x y z]*[0 1 0 0] = [-x w z -y]
  // [-x w z -y]*[w -x -y -z] =
  //   [ -xw+wx+zy-yz    [0
  //      xx+ww-zz-yy  =  ww+xx-yy-zz
  //      xy+wz+zw+yx     2(xy+wz)
  //      xz-wy+zx-yw ]   2(xz-wy) ]
  // To simplify, we use
  //   ww+xx+yy+zz = 1 therefore
  //   ww+xx-yy-zz = 1-(ww+xx+yy+zz)+ww+xx-yy-zz
  //     = 1-2(yy+zz)

  double w = q.scalar();
  double x = q.x();
  double y = q.y();
  double z = q.z();

  // First column
  m[ 0] = 1.0 - 2.0 * (y*y + z*z);
  m[ 4] = 2.0 * (x*y + z*w);
  m[ 8] = 2.0 * (x*z - y*w);
  m[12] = 0.0;

  // Second column
  m[ 1] = 2.0 * (x*y - z*w);
  m[ 5] = 1.0 - 2.0 * (x*x + z*z);
  m[ 9] = 2.0 * (z*y + x*w);
  m[13] = 0.0;

  // Third column
  m[ 2] = 2.0 * (x*z + y*w);
  m[ 6] = 2.0 * (y*z - x*w);
  m[10] = 1.0 - 2.0 * (x*x + y*y);
  m[14] = 0.0;

  // Fourth column
  m[ 3] = 0.0;
  m[ 7] = 0.0;
  m[11] = 0.0;
  m[15] = 1.0;
}

void QuatCamera::reset()
{
  position=QVector3D(0,0,0);
  orient=QQuaternion(1,0,0,0);
}

void QuatCamera::setPosition(const QVector3D& pos)
{
  position = pos;
}

void QuatCamera::translateRelative(const QVector3D& relDist)
{
  QVector3D absTrans = orient.rotatedVector(relDist);
  translateAbsolute(absTrans);
}

void QuatCamera::translateAbsolute(const QVector3D& absDist)
{
  position+=absDist;
}

void QuatCamera::rotateRelative(const QQuaternion& quat)
{
  //QQuaternion tmp= orient*quat;
  orient = orient*quat;
  orient.normalize();
}

void QuatCamera::rotateAbsolute(const QQuaternion& quat)
{
  orient = quat*orient;
  orient.normalize();
}


