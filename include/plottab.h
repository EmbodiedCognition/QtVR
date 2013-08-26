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
#ifndef PLOTTAB_H
#define PLOTTAB_H

#include <QScrollArea>


class SequenceForm;
class QVBoxLayout;


/**
  Store and manage interface elements that control
  the data that are plotted.
  */
class PlotTab : public QScrollArea
{
    Q_OBJECT
public:
    explicit PlotTab(QWidget *parent = 0);

signals:

public slots:
  void addNewPlot();
  void newPlotSlot(QString seq);
  //void removePlot(QObject* plot);


protected:
  SequenceForm* seqForm;
  QVBoxLayout* layout;





};

#endif // PLOTTAB_H
