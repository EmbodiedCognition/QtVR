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
#ifndef MYGRAPHICSSCENE_H
#define MYGRAPHICSSCENE_H

#include <QGraphicsScene>
#include <QPainterPath>
#include <QGraphicsPathItem>
#include <QColor>

class Sequence;
/**
  A plot of data, with optional vertical axis
  Vertically scalable and translatable
  Can display a fixed window of data or
  an open ended one.
  */
class DataPlotItem : public QObject
{
  Q_OBJECT


  Sequence* sequence;
  int row;
  int col;
  int tBegin;
  int tEnd; // Open if negative

public:
    explicit DataPlotItem(QGraphicsScene* scene,QObject *parent = 0);
  // The plot itself

  QGraphicsPathItem* pathItem;

public slots:
  void setSequence(Sequence* seq);
  void setDataPoint(int row,int col);
  void setRange(int startFrame,int endFrame);
  void setColor(const QColor& color);

  // slot to update path if the sequence changes
  void update();

  // Parent object has a slot to delete this
  // if the sequence is off-loaded.

  void setYScale(double scale);
  void setYTrans(double trans);

};

/**
  We hold a list of plots that are each assigned
  to a single degree of freedom from a subrange
  of a specific sequence.

  We also draw an axis and labels for the plots.
  (Labels are tough if we're allowing scaling.)


  */
class MyGraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit MyGraphicsScene(QObject *parent = 0);

  void setAxis(int length,int tickSpace,int labelSpace);

signals:

public slots:
  /*
  void createPlot(Sequence* seq,
                  int row,int col, // Which degree of freedom
                  int rStart,int rEnd, // Range of time
                  double yScale,double yOffset);*/

public:
  int axSize;

};

#endif // MYGRAPHICSSCENE_H
