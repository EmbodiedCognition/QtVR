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

#ifndef JOINTWIDGET_H
#define JOINTWIDGET_H

#include <QWidget>

namespace Ui {
    class JointWidget;
}

class JointWidget : public QWidget
{
    Q_OBJECT

public:
    explicit JointWidget(QString label,int jj,int aa,QWidget *parent = 0);
    ~JointWidget();

signals:
  void valueChanged(int,int,double);
  void forceChanged(int,int,double);

public slots:
  void zeroOut();
  void reset();

  void setForceLimit(double limit);
  void intToAngleSpinBox(int);
  void doubleToAngleSlider(double);
  void setValue(int,int,double);

  void  forceBox(double);
public:
    Ui::JointWidget *ui;

    int jVal;
    int aVal;
};

#endif // JOINTWIDGET_H
