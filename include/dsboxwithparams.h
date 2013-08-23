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

#ifndef DSBOXWITHPARAMS_H
#define DSBOXWITHPARAMS_H

#include <QDoubleSpinBox>

/**
  Double spin box with extra parameters so that
  when it changes it can emit a signal intended
  for a specific target.

  This object is a nasty hack.
  The right way would probably be to associate
  the widget directly with the object it changes.
  */
class DSBoxWithParams : public QDoubleSpinBox
{
    Q_OBJECT
public:
    explicit DSBoxWithParams(int type,int axis,QWidget *parent = 0);

signals:
  void valueChanged(int,int,double);

public slots:
  void newValue(double);

protected:
  int tt, ax;

};

#endif // DSBOXWITHPARAMS_H
