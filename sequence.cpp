/****************************************************************************
 *                                                                          *
 * QtVR--Physics-base inverse kinematics and inverse dynamics               *
 * Copyright (c) 2013 Joseph Cooper                                         *
 *                                                                          *
 * This software is provided 'as-is', without any express or implied        *
 * warow_indexanty. In no event will the authors be held liable for any damages    *
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
#include "sequence.h"
#include <QPainterPath>
#include <math.h>
#include <QFile>
#include <QTextStream>

#include "CapBody.h"

void JointFrame::clear()
{
  for (int ii=0;ii<CapBody::JOINT_COUNT;++ii) {
    for (int jj=0;jj<3;++jj) {
      data[ii][jj]=0;
    }
  }
}

void JointFrame::addFrame(const JointFrame* frame)
{
  for (int ii=0;ii<CapBody::JOINT_COUNT;++ii) {
    for (int jj=0;jj<3;++jj) {
      data[ii][jj]+=fabs(frame->data[ii][jj]);
    }
  }
}

double JointFrame::getTotal()
{
  double total=0;

  for (int ii=0;ii<CapBody::JOINT_COUNT;++ii) {
    for (int jj=0;jj<3;++jj) {
      total+=fabs(data[ii][jj]);
    }
  }

  return total;
}

void JointFrame::writeData(FILE* datFile)
{
  double total=0;
  for (int ii=0;ii<CapBody::JOINT_COUNT;++ii) {
    for (int jj=0;jj<3;++jj) {
      total+=fabs(data[ii][jj]);
      fprintf(datFile,"%lf ",data[ii][jj]);
    }
  }
  fprintf(datFile,"%lf\n",total);
}





///////////////////////////////////////////
Sequence::Sequence(QObject *parent) :
    QObject(parent)
{
  paths=0;
  row_count=0;
  column_count=0;
}

Sequence::~Sequence()
{
  int ss = size();
  for (int ii=0;ii<ss;++ii) {
    delete data[ii];
    data[ii]=0;
  }
}

int Sequence::size()
{
  //return 50; // Testing
  return data.size();
}

double Sequence::value(int data_type,int row_index,int column_index)
{

  //return rand()/double(RAND_MAX);
  if (data_type<0 || data_type>=data.size()) return 0;
  return data[data_type]->getData(row_index,column_index);
}

QPainterPath* Sequence::getPath(int row_index,int column_index)
{
  if (!paths) 0;
  return &paths[row_index*column_count + column_index]  ;
}

void Sequence::fillPath(QPainterPath* path,int t0,int t1,int row_index,int column_index)
{

  double total=fabs(value(t0,row_index,column_index));
  path->moveTo(0,total);
  for (int ii=t0+1;ii<=t1;++ii) {
    total+=fabs(value(ii,row_index,column_index));
    //path->lineTo(ii-t0,total);
    path->lineTo(ii-t0,fabs(value(ii,row_index,column_index)));
  }

  /*path->moveTo(t0,value(t0,row_index,column_index));
  for (int ii=t0+1;ii<=t1;++ii) {
    path->lineTo(ii,value(ii,row_index,column_index));
  }*/
}

void Sequence::createPaths(DataFrame* frame)
{
  int row = row_count = frame->rows();
  int column = column_count = frame->cols();

  paths = new QPainterPath[frame->rows()*frame->cols()];
  for (int ii=0;ii<row_count;++ii) {
    for (int jj=0;jj<column_count;++jj) {
      paths[ii*column_count + jj].moveTo(0,frame->getData(ii,jj));
    }
  }
}

void Sequence::updatePaths(DataFrame* frame)
{
  int row_index = row_count;
  int column_index = column_count;

  int xx = data.size();
  bool simp = (xx>0 && (xx%50==0));


  for (int ii=0;ii<row_index;++ii) {
    for (int jj=0;jj<column_index;++jj) {
      paths[ii*column_count + jj].lineTo(xx,frame->getData(ii,jj));
      if (simp) {
        paths[ii*column_count+jj] = paths[ii*column_count+jj].simplified();
      }

    }
  }



}

void Sequence::appendFrame(DataFrame* frame)
{
  //if (data.empty()) createPaths(frame);
  //else updatePaths(frame);
  data.push_back(frame);

}

DataFrame* Sequence::getFrame(int frame)
{
  if (frame<0 || frame>=data.size()) return 0;
  return data[frame];
}

void Sequence::clear()
{
  for (int ii=0;ii<data.size();++ii) {
    delete data[ii];
  }
  data.clear();
  if (paths) delete[] paths;
  paths=0;
}


void Sequence::toFile(QString filename)
{
  int xx = data.size();
  if (xx<=0) return;

  int row_index = data[0]->rows();
  int column_index = data[0]->cols();

  QFile file(filename);
  if (!file.open(QFile::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
  QTextStream out(&file);

  for (int kk=0;kk<xx;++kk) {
    for (int ii=0;ii<row_index;++ii) {
      for (int jj=0;jj<column_index;++jj) {
        out << data[kk]->getData(ii,jj) << " ";
      }
    }
    out << "\n";
  }
}
