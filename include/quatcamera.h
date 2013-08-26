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
#ifndef QUATCAMERA_H
#define QUATCAMERA_H


#include <QQuaternion>
#include <QVector3D>
#include <QMatrix4x4>
#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502
#endif

/**
   QT Quaternion based camera.  Actually,
   it's just storage of rigid body information
   which we can transform locally or globally.
   If we were to put velocity info into this
   we'd have a full rigid body.
  */
class QuatCamera
{
  QQuaternion orient;
  QVector3D position;

public:
    QuatCamera();

  QVector3D getPosition();
  QQuaternion getQuaternion();

  QMatrix4x4 getTransform();
  QMatrix4x4 getInvTransform();
  void getTransform(QMatrix4x4& mat);
  void getInvTransform(QMatrix4x4& mat);

  void getRMajorTrans(double* m);
  void getCMajorTrans(double* m);
  void getRMajorInvTrans(double* m);
  void getCMajorInvTrans(double* m);

  /// Store quaternion Q in matrix m[16]
  /// If m is a column matrix, this
  /// will store the inverse rotation.
  void quatToRowMajorMat(const QQuaternion& q,double* m);


  void reset();
  void setPosition(const QVector3D& pos);
  void translateRelative(const QVector3D& relDist);
  void translateAbsolute(const QVector3D& absDist);
  void rotateRelative(const QQuaternion& quat);
  void rotateAbsolute(const QQuaternion& quat);

};

#endif // QUATCAMERA_H
