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
#ifndef MARKERWIDGET_H
#define MARKERWIDGET_H

#include <QWidget>

namespace Ui {
    class MarkerWidget;
}

class MarkerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MarkerWidget(int id,QWidget *parent = 0);
    ~MarkerWidget();

signals:
  void markBodySet(int marker,int body);
  void markConnect(int marker,bool join);
  void markPosSet(int marker,double xx,double yy,double zz);
  void markGrab(int marker);

public slots:
  void setMarkBody(int id);
  void setMarkPos(double xx,double yy,double zz);
  void setConnect(bool join);

  void posChanged();
  void bodyChanged();

  void grab();

private:
    Ui::MarkerWidget *ui;
    int m_id;
};

#endif // MARKERWIDGET_H
