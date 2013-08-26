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
#ifndef SEQUENCE_H
#define SEQUENCE_H

#include <QObject>

#include "CapBody.h"
#include "DataCValues.h"

#include <QString>

class  QPainterPath;

class DataFrame
{
protected:
  int m_frame;
public:
    DataFrame(int ff) { m_frame = ff; }

  int frame() { return m_frame; }
  virtual void setData(int row,int col,double val)=0;
  virtual double getData(int row,int col)=0;
  virtual int rows()=0;
  virtual int cols()=0;
};

class JointFrame : public DataFrame
{
protected:
  double data[CapBody::JOINT_COUNT][3];
public:
    JointFrame(int ff=0) : DataFrame(ff) { clear(); }

  virtual void setData(int row,int col,double val) { data[row][col]=val; }
  virtual double getData(int row,int col) { return data[row][col]; }
  static int rSize() { return CapBody::JOINT_COUNT; }
  virtual int rows() { return rSize(); }
  static int cSize() { return 3; }
  virtual int cols() { return cSize(); }

  void clear();
  void addFrame(const JointFrame* frame);
  double getTotal();
  void writeData(FILE* datFile);
};

class MarkerFrame : public DataFrame
{
protected:
  double data[LED_COUNT][4];
public:
    MarkerFrame(int ff=0) : DataFrame(ff) {}

  virtual void setData(int row,int col,double val) { data[row][col]=val; }
  virtual double getData(int row,int col) { return data[row][col]; }


  static int rSize() { return LED_COUNT; }
  virtual int rows() { return rSize(); }
  static int cSize() { return 4; }
  virtual int cols() { return cSize(); }


};

/**
  Three dimensional data:
  1st dimension -> time
  2nd dimension -> degrees of freedom
  3rd dimension -> axes of the degree of freedom

  */
class Sequence : public QObject
{
    Q_OBJECT
public:
    explicit Sequence(QObject *parent = 0);
    ~Sequence();

  int size();
  double value(int data_type,int row_index,int column_index);
  QPainterPath* getPath(int row_index,int column_index);
  void fillPath(QPainterPath* path,int t0,int t1,int row_index,int column_index);

  void createPaths(DataFrame* frame);
  void updatePaths(DataFrame* frame);
  void appendFrame(DataFrame* frame);
  DataFrame* getFrame(int frame);
  void toFile(QString filename);

signals:

public slots:
  void clear();

protected:
  QList<DataFrame*> data;
  QPainterPath* paths;
  int row_count;
  int column_count;

};

#endif // SEQUENCE_H
