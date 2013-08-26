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
#include "livemarkerdata.h"

#include "CapBody.h"

LiveMarkerData::LiveMarkerData(dWorldID world,dSpaceID space,QObject *parent) :
    QObject(parent)
{
  this->world=world;
  this->space=space;

  // ***** Connect to phasespace

  markCnt = 50;
  cFrame = &(frames[0]);
  nFrame = &(frames[1]);

  geom = new dGeomID[markCnt];
  body = new dBodyID[markCnt];
  joint = new dJointID[markCnt];
  tryLink = new bool[markCnt];

  //space = dSimpleSpaceCreate(0);
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
  timePerStep = .016;
  currentFrame = 0;
  paused=true;
  singleStep=false;
}

int LiveMarkerData::size()
{
  return 1000;
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
void LiveMarkerData::step()
{
  // Set the markers to hold the
  // current frame

  // Get the current frame
  // Get the next frame

  MarkerDataFrame* tmp = cFrame;
  cFrame = nFrame;
  nFrame = tmp;

  // ***** phasespace.fillFrame(nFrame);



  dJointGroupEmpty(jointGroup);

  if (paused && !singleStep) {
    for (int ii=0;ii<markCnt;++ii) {
      float xx,yy,zz;
      xx = cFrame->data[ii][0]*.001;
      yy = cFrame->data[ii][1]*.001;
      zz = cFrame->data[ii][2]*.001;
      if (cFrame->data[ii][3]<0) zz=-1;
      dBodySetLinearVel(body[ii],0,0,0);
      dBodySetPosition(body[ii],xx,yy,zz);

    }

  } else {
    for (int ii=0;ii<markCnt;++ii) {
      float xx,yy,zz;
      xx = cFrame->data[ii][0]*.001;
      yy = cFrame->data[ii][1]*.001;
      zz = cFrame->data[ii][2]*.001;
      if (cFrame->data[ii][3]<0) zz=-1;

      float dx,dy,dz;
      dx = nFrame->data[ii][0]*.001 - xx;
      dy = nFrame->data[ii][1]*.001 - yy ;
      dz = nFrame->data[ii][2]*.001 - zz;

      dBodySetLinearVel(body[ii],dx/timePerStep,dy/timePerStep,dz/timePerStep);
      dBodySetPosition(body[ii],xx,yy,zz);
    }
    setFrame(currentFrame+framesPerStep);
    singleStep = false;
  }

  for (int ii=0;ii<MARKER_COUNT;++ii) {
    int bID = cBody->markerToBody[ii].id;
    //if (bID!=CapBody::HEAD_BODY &&
//        bID!=CapBody::L_HEEL_BODY &&
//        bID!=CapBody::R_HEEL_BODY) continue;
    if (tryLink[ii] && (bID>=0) &&
        (cFrame->data[ii][3]>0) &&
        (nFrame->data[ii][3]>0))

    {
      joint[ii]=dJointCreateBall(world,jointGroup);

      dJointAttach(joint[ii],body[ii],cBody->bodies[bID]);
      dJointSetBallAnchor1Rel(joint[ii],0,0,0);
      dJointSetBallAnchor2Rel(joint[ii],
                              cBody->markerToBody[ii].pos[0],
                              cBody->markerToBody[ii].pos[1],
                              cBody->markerToBody[ii].pos[2]);
      dJointSetBallParam(joint[ii],dParamCFM,.0001);
      dJointSetBallParam(joint[ii],dParamERP,.2);
    }

  }
}


void LiveMarkerData::setPaused(bool playing)
{
  this->paused = !playing;
}

void LiveMarkerData::setSingleStep()
{
  singleStep=true;
  paused = true;
}

void LiveMarkerData::setFrame(int frame)
{
  frame;
  //if (frame>=0 && frame<size() && currentFrame!=frame) {
  //  currentFrame=frame;
  //  emit frameChanged(frame);
  //}
}

void LiveMarkerData::changeBodyConnect(int mark,int body)
{
  cBody->markerToBody[mark].id=body;
}

void LiveMarkerData::changeBodyLink(int mark,bool link)
{
  tryLink[mark]=link;
}

void LiveMarkerData::changeLinkPos(int mark,double xx,double yy,double zz)
{
  cBody->markerToBody[mark].pos[0]=xx;
  cBody->markerToBody[mark].pos[1]=yy;
  cBody->markerToBody[mark].pos[2]=zz;
}





