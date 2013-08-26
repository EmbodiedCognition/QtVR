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
#include "pokemarkerdata.h"

#include "CapBody.h"

PokeMarkerData::PokeMarkerData(dWorldID world,dSpaceID space,QObject *parent) :
    QObject(parent)
{
  this->world=world;
  this->space=space;

  loadData("pokeData.dat");

  //data.loadFile("data.c3d");

  markCnt = 50;
  nFrame = cFrame = &data[0];


  geom = new dGeomID[markCnt];
  body = new dBodyID[markCnt];
  joint = new dJointID[markCnt];
  tryLink = new bool[markCnt];

  // We re-create the joints with every step.
  jointGroup = dJointGroupCreate(0);
  for (int ii=0;ii<markCnt;++ii) {
    body[ii] = dBodyCreate(world);
    dBodySetKinematic(body[ii]);
    geom[ii] = dCreateSphere(space,.01);
    dGeomSetBody(geom[ii],body[ii]);
    tryLink[ii]=false;
    joint[ii] = dJointCreateBall(world,jointGroup);
  }


  framesPerStep=1;
  timePerStep = .015;
  currentFrame = 0;
  paused=true;
  singleStep=false;

}


int PokeMarkerData::size()
{
  return data.size();
}

/**
  Update the marker positions and velocities.
  If the data model is paused, then we still
  set the position of the markers so that they
  can be displayed.  However, the velocity is
  zero.  If the model is not paused, we set the
  marker location and set its velocity.  Then
  we advance the current marker frame forward.

  If the current marker frame is not visible,
  the marker goes to the origin (or below the
  plane).
  If either frame is not visible, we don't
  attach the joint.

  */
void PokeMarkerData::step()
{
  // Set the markers to hold the
  // current frame

  // Get the current frame
  // Get the next frame

  //data.readFrame(currentFrame,cFrame);
  //data.readFrame(currentFrame+framesPerStep,nFrame);

  if (currentFrame<size()) cFrame = &(data[currentFrame]);
  if (currentFrame+framesPerStep<size()) nFrame = &(data[currentFrame+framesPerStep]);

  dJointGroupEmpty(jointGroup);

  if (paused && !singleStep) {
    for (int ii=0;ii<markCnt;++ii) {
      float xx,yy,zz;

      xx = cFrame->markers[ii][0];
      yy = cFrame->markers[ii][1];
      zz = cFrame->markers[ii][2];
      if (cFrame->markers[ii][3]<0) zz=-1;
      dBodySetLinearVel(body[ii],0,0,0);
      dBodySetPosition(body[ii],xx,yy,zz);

    }

  } else {
    for (int ii=0;ii<markCnt;++ii) {
      float xx,yy,zz;
      xx = cFrame->markers[ii][0];
      yy = cFrame->markers[ii][1];
      zz = cFrame->markers[ii][2];
      if (cFrame->markers[ii][3]<0) zz=-1;

      float dx,dy,dz;
      dx = nFrame->markers[ii][0] - xx;
      dy = nFrame->markers[ii][1] - yy ;
      dz = nFrame->markers[ii][2] - zz;

      dBodySetLinearVel(body[ii],dx/timePerStep,dy/timePerStep,dz/timePerStep);
      dBodySetPosition(body[ii],xx,yy,zz);
    }
    //currentFrame+=framesPerStep;
    setFrame(currentFrame+framesPerStep);
    singleStep = false;

  }

  for (int ii=0;ii<MARKER_COUNT;++ii) {
    int bID = cBody->marker_to_body[ii].id;
    //if (bID!=CapBody::HEAD_BODY &&
//        bID!=CapBody::L_HEEL_BODY &&
//        bID!=CapBody::R_HEEL_BODY) continue;
    if (tryLink[ii] && (bID>=0) &&
        (cFrame->markers[ii][3]>0) &&
        (nFrame->markers[ii][3]>0))

    {
      joint[ii]=dJointCreateBall(world,jointGroup);

      dJointAttach(joint[ii],body[ii],cBody->body_segments[bID]);
      dJointSetBallAnchor1Rel(joint[ii],0,0,0);
      dJointSetBallAnchor2Rel(joint[ii],
                              cBody->marker_to_body[ii].position[0],
                              cBody->marker_to_body[ii].position[1],
                              cBody->marker_to_body[ii].position[2]);
      dJointSetBallParam(joint[ii],dParamCFM,.0001);
      dJointSetBallParam(joint[ii],dParamERP,.2);
    }

  }



}


void PokeMarkerData::setPaused(bool playing)
{
  this->paused = !playing;
}

void PokeMarkerData::setSingleStep()
{
  singleStep=true;
  paused = true;
}

void PokeMarkerData::setFrame(int frame)
{
  if (frame>=0 && frame<size() && currentFrame!=frame) {
    currentFrame=frame;
    emit frameChanged(frame);
  }
}

void PokeMarkerData::changeBodyConnect(int mark,int body)
{
  cBody->marker_to_body[mark].id=body;
}

void PokeMarkerData::changeBodyLink(int mark,bool link)
{
  tryLink[mark]=link;
}

void PokeMarkerData::changeLinkPos(int mark,double xx,double yy,double zz)
{
  cBody->marker_to_body[mark].position[0]=xx;
  cBody->marker_to_body[mark].position[1]=yy;
  cBody->marker_to_body[mark].position[2]=zz;
}


void PokeMarkerData::loadData(const char* filename)
{
  FILE* file = fopen(filename,"rb");
  unsigned int frames;
  fread(&frames,sizeof(unsigned int),1,file);
  data.resize(frames);
  fread(&(data[0]),sizeof(PokeDataFrame),frames,file);
  fclose(file);
  emit maxSize(frames-1);
}
