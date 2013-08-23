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
#include "markerwidget.h"
#include "ui_markerwidget.h"

MarkerWidget::MarkerWidget(int id,QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MarkerWidget),
    m_id(id)
{
    ui->setupUi(this);
    ui->idButton->setText(QString::number(id));
    connect(ui->idButton,SIGNAL(toggled(bool)),this,SLOT(setConnect(bool)));
    connect(ui->xSpinBox,SIGNAL(valueChanged(double)),this,SLOT(posChanged()));
    connect(ui->ySpinBox,SIGNAL(valueChanged(double)),this,SLOT(posChanged()));
    connect(ui->zSpinBox,SIGNAL(valueChanged(double)),this,SLOT(posChanged()));
    connect(ui->bodyComboBox,SIGNAL(currentIndexChanged(int)),this,SLOT(bodyChanged()));
    connect(ui->grabButton,SIGNAL(clicked()),this,SLOT(grab()));
}

MarkerWidget::~MarkerWidget()
{
    delete ui;
}

void MarkerWidget::setMarkBody(int id)
{
  ui->bodyComboBox->setCurrentIndex(id);
}

void MarkerWidget::setMarkPos(double xx,double yy,double zz)
{
  ui->xSpinBox->setValue(xx);
  ui->ySpinBox->setValue(yy);
  ui->zSpinBox->setValue(zz);
}


void MarkerWidget::setConnect(bool join)
{
  ui->idButton->setChecked(join);
  emit markConnect(m_id,join);
}


void MarkerWidget::posChanged()
{
  emit markPosSet(m_id,
                  ui->xSpinBox->value(),
                  ui->ySpinBox->value(),
                  ui->zSpinBox->value());
}

void MarkerWidget::bodyChanged()
{
  emit markBodySet(m_id,ui->bodyComboBox->currentIndex()-1);
}


void MarkerWidget::grab()
{
  emit markGrab(m_id);
}
