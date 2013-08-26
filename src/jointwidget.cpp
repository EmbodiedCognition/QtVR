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
#include "jointwidget.h"
#include "ui_jointwidget.h"



JointWidget::JointWidget(QString label,int jj,int aa,QWidget *parent) :
    QWidget(parent),
    ui(new Ui::JointWidget),
    jVal(jj),
    aVal(aa)
{
    ui->setupUi(this);
    ui->label->setText(label);
    connect(ui->fmaxSpinBox,SIGNAL(valueChanged(double)),this,SLOT(forceBox(double)));
}

JointWidget::~JointWidget()
{
    delete ui;
}


void JointWidget::zeroOut()
{
  ui->fmaxSpinBox->setValue(0);
}
void JointWidget::reset()
{
  ui->angleSpinBox->setValue(0);
  ui->fmaxSpinBox->setValue(9999);
}

void JointWidget::setForceLimit(double limit)
{
  ui->fmaxSpinBox->setValue(limit);
}

/**
  Scale the value to be in (-pi,pi)
  */
void JointWidget::intToAngleSpinBox(int val)
{
  int iRange = ui->angleSlider->maximum() - ui->angleSlider->minimum();
  double dRange = ui->angleSpinBox->maximum() - ui->angleSpinBox->minimum();
  double dVal = (val - ui->angleSlider->minimum())*(dRange/iRange) + ui->angleSpinBox->minimum();
  ui->angleSpinBox->setValue(dVal);
  emit(valueChanged(jVal,aVal,dVal));
}


void JointWidget::doubleToAngleSlider(double val)
{
  int iRange = ui->angleSlider->maximum() - ui->angleSlider->minimum();
  double dRange = ui->angleSpinBox->maximum() - ui->angleSpinBox->minimum();
  int iVal = (val - ui->angleSpinBox->minimum())*(iRange/dRange) + ui->angleSlider->minimum();
  ui->angleSlider->setValue(iVal);
}

void JointWidget::setValue(int aa,int bb,double val)
{
  (void)(aa);
  (void)(bb);
  doubleToAngleSlider(val);
}

void  JointWidget::forceBox(double val)
{
  emit forceChanged(jVal,aVal,val);
}
