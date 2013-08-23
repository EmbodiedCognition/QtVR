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
#include "mygraphicsscene.h"

#include "sequence.h"

DataPlotItem::DataPlotItem(QGraphicsScene* scene,QObject *parent)
  : QObject(parent)
{
  pathItem = new QGraphicsPathItem(0,scene);
  sequence = 0;
  row = 0;
  col = 0;
  tBegin = 0;
  tEnd = -1;
}

void DataPlotItem::setSequence(Sequence* seq)
{
  sequence = seq;
  update();
}

void DataPlotItem::setDataPoint(int row,int col)
{
  this->row = row;
  this->col = col;
  update();
}

void DataPlotItem::setRange(int startFrame,int endFrame)
{
  if (startFrame<0) tBegin=0;
  else tBegin = startFrame;

  if (endFrame>=tBegin) tEnd=endFrame;
  else tEnd = -1; // Open ended
  update();
}

void DataPlotItem::setColor(const QColor& color)
{
  QPen tmpPen = pathItem->pen();
  tmpPen.setColor(color);
  pathItem->setPen(tmpPen);

}

void DataPlotItem::update()
{
  if (sequence) {
    int cEnd = tEnd;
    if (cEnd<0) cEnd = sequence->size();
    QPainterPath path(QPointF(tBegin,sequence->value(tBegin,row,col)));
    for (int ii=tBegin+1;ii<cEnd;++ii) {
      path.lineTo(ii,sequence->value(ii,row,col));
    }
    pathItem->setPath(path);
  }
}

void DataPlotItem::setYScale(double scale)
{
  QTransform transform(1,0,0,scale,0,0);
  pathItem->setTransform(transform);
}

void DataPlotItem::setYTrans(double trans)
{
  pathItem->setY(trans);
}

//////////////////////////////////////////////////


MyGraphicsScene::MyGraphicsScene(QObject *parent) :
    QGraphicsScene(parent)
{
  axSize=0;
}


void MyGraphicsScene::setAxis(int length,int tickSpace,int labelSpace)
{
  axSize=length;
  double smallTick = 5;
  double bigTick = 10;

  QPainterPath path(QPointF(0,0));
  path.lineTo(length,0);
  for (int ii=0;ii<=length;ii+=tickSpace) {
    path.moveTo(ii,0);
    path.lineTo(ii,smallTick);
  }
  for (int ii=0;ii<=length;ii+=labelSpace) {
    path.moveTo(ii,bigTick);
    path.lineTo(ii,0);
    //path.moveTo(ii,-bigTick);
    path.addText(ii,bigTick+smallTick+15,QFont(QString("Helvetica"),15),QString::number(ii));
  }
  path.addText(250,50,QFont(QString("Helvetica"),20),"Time");
  QGraphicsPathItem* pathItem = new QGraphicsPathItem(path,0,this);
  pathItem;

}


