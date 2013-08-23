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
#include "boarddata.h"

#include <cstdio>
#include <QGLWidget>

BoardData::BoardData(QObject *parent) :
    QObject(parent)
{

  data=NULL;
}

BoardData::~BoardData()
{
  if (data) delete[] data;
  data=NULL;
}

void BoardData::loadData(const char* filename)
{
  if (data) delete[] data;
  data = NULL;

  FILE* datFile = fopen(filename,"rb");

  fread((void*)&frames,sizeof(int),1,datFile);
  fread((void*)&boards,sizeof(int),1,datFile);

  data = new double[frames*boards*4];
  fread((void*)data,sizeof(double),boards*frames*4,datFile);
  fclose(datFile);
  cFrame=0;
}

void BoardData::setFrame(int frame)
{
  if (frame<0) frame=0;
  if (frame>=frames) frame=frames-1;
  cFrame = frame;
}

void BoardData::step()
{
  cFrame+=1;
  if (cFrame>=frames) {
    cFrame=0;
  }
}

void BoardData::render()
{
  double* pt = &(data[cFrame*boards*4]);
  glBegin(GL_LINES);
  for (int ii=0;ii<boards;++ii) {
    glVertex3d(pt[ii*4+0],pt[ii*4+1],pt[ii*4+2]);
    glVertex3d(pt[ii*4+0],pt[ii*4+1],pt[ii*4+2]+pt[ii*4+3]*.025);
  }
  glEnd();
}
