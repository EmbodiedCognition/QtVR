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
  rsize=0;
  csize=0;
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

double Sequence::value(int tt,int rr,int cc)
{

  //return rand()/double(RAND_MAX);
  if (tt<0 || tt>=data.size()) return 0;
  return data[tt]->getData(rr,cc);
}

QPainterPath* Sequence::getPath(int rr,int cc)
{
  if (!paths) 0;
  return &paths[rr*csize + cc]  ;
}

void Sequence::fillPath(QPainterPath* path,int t0,int t1,int rr,int cc)
{

  double total=fabs(value(t0,rr,cc));
  path->moveTo(0,total);
  for (int ii=t0+1;ii<=t1;++ii) {
    total+=fabs(value(ii,rr,cc));
    //path->lineTo(ii-t0,total);
    path->lineTo(ii-t0,fabs(value(ii,rr,cc)));
  }

  /*path->moveTo(t0,value(t0,rr,cc));
  for (int ii=t0+1;ii<=t1;++ii) {
    path->lineTo(ii,value(ii,rr,cc));
  }*/
}

void Sequence::createPaths(DataFrame* frame)
{
  int rr = rsize = frame->rows();
  int cc = csize = frame->cols();

  paths = new QPainterPath[frame->rows()*frame->cols()];
  for (int ii=0;ii<rr;++ii) {
    for (int jj=0;jj<cc;++jj) {
      paths[ii*csize + jj].moveTo(0,frame->getData(ii,jj));
    }
  }
}

void Sequence::updatePaths(DataFrame* frame)
{
  int rr = rsize;
  int cc = csize;

  int xx = data.size();
  bool simp = (xx>0 && (xx%50==0));


  for (int ii=0;ii<rr;++ii) {
    for (int jj=0;jj<cc;++jj) {
      paths[ii*csize + jj].lineTo(xx,frame->getData(ii,jj));
      if (simp) {
        paths[ii*csize+jj] = paths[ii*csize+jj].simplified();
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

  int rr = data[0]->rows();
  int cc = data[0]->cols();

  QFile file(filename);
  if (!file.open(QFile::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
  QTextStream out(&file);

  for (int kk=0;kk<xx;++kk) {
    for (int ii=0;ii<rr;++ii) {
      for (int jj=0;jj<cc;++jj) {
        out << data[kk]->getData(ii,jj) << " ";
      }
    }
    out << "\n";
  }
}
