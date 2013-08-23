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
#include "markerdata.h"

#include "CapBody.h"

MarkerData::MarkerData(dWorldID world,dSpaceID space,QObject *parent) :
    QObject(parent),
    randN(rng,norm)
{
  this->world=world;
  this->space=space;

  data.loadFile("data/data.c3d");

  markCnt = data.data.pointsPerFrame;
  cOld = cFrame = new C3dFloatFrame(markCnt);
  nOld = nFrame = new C3dFloatFrame(markCnt);

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



  framesPerStep=2;
  timePerStep = .015;
  currentFrame = 0;
  paused=true;
  singleStep=false;

  vMarkers=false;

}

int MarkerData::size()
{
  return data.data.frames;
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
void MarkerData::step()
{
  // Set the markers to hold the
  // current frame

  // Get the current frame
  // Get the next frame


  if (!vMarkers) {
    data.readFrame(currentFrame,cFrame);
    data.readFrame(currentFrame+framesPerStep,nFrame);
  } else {
    if (vPoint>=0 && vPoint<shadowData.size()) {
      cFrame = shadowData[vPoint];
      if (vPoint==shadowData.size()-1) {
        nFrame = shadowData[vPoint];
      } else {
        nFrame = shadowData[vPoint+1];
      }

    } else {
      cFrame = nFrame = shadowData.last();
    }
  }
  dJointGroupEmpty(jointGroup);

  if (paused && !singleStep) {
    for (int ii=0;ii<markCnt;++ii) {
      float xx,yy,zz;
      xx = cFrame->data[ii].point[0]*.001;
      yy = cFrame->data[ii].point[1]*.001;
      zz = cFrame->data[ii].point[2]*.001;
      if (cFrame->data[ii].point[3]<0) zz=-1;
      dBodySetLinearVel(body[ii],0,0,0);
      dBodySetPosition(body[ii],xx,yy,zz);

    }

  } else {
    for (int ii=0;ii<markCnt;++ii) {
      float xx,yy,zz;
      xx = cFrame->data[ii].point[0]*.001;
      yy = cFrame->data[ii].point[1]*.001;
      zz = cFrame->data[ii].point[2]*.001;
      if (cFrame->data[ii].point[3]<0) zz=-1;

      float dx,dy,dz;
      dx = nFrame->data[ii].point[0]*.001 - xx;
      dy = nFrame->data[ii].point[1]*.001 - yy ;
      dz = nFrame->data[ii].point[2]*.001 - zz;

      dBodySetLinearVel(body[ii],dx/timePerStep,dy/timePerStep,dz/timePerStep);
      dBodySetPosition(body[ii],xx,yy,zz);
    }
    //currentFrame+=framesPerStep;
    setFrame(currentFrame+framesPerStep);
    if (vMarkers) vPoint+=1;
    singleStep = false;

  }

  /*
    Ugly GUI interactions made it annoying to keep the
    number of markers sync'ed with the c3d file.  So
    the number of marker widgets is hard-coded
    in the .pro file as MARKER_COUNT.
    */
  for (int ii=0;ii<MARKER_COUNT;++ii) {
    int bID = cBody->markerToBody[ii].id;

    if (tryLink[ii] && (bID>=0) &&
        (cFrame->data[ii].point[3]>0) &&
        (nFrame->data[ii].point[3]>0))
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

  if (vMarkers){
    cFrame = cOld;
    nFrame = nOld;
  }


}


void MarkerData::setPaused(bool playing)
{
  this->paused = !playing;
}

void MarkerData::setSingleStep()
{
  singleStep=true;
  paused = true;
}

void MarkerData::setFrame(int frame)
{
  if (frame>=0 && frame<size() && currentFrame!=frame) {
    currentFrame=frame;
    emit frameChanged(frame);

  }

}

void MarkerData::changeBodyConnect(int mark,int body)
{
  cBody->markerToBody[mark].id=body;
}

void MarkerData::changeBodyLink(int mark,bool link)
{
  tryLink[mark]=link;
}

void MarkerData::changeLinkPos(int mark,double xx,double yy,double zz)
{
  cBody->markerToBody[mark].pos[0]=xx;
  cBody->markerToBody[mark].pos[1]=yy;
  cBody->markerToBody[mark].pos[2]=zz;
}


/**
  Grab the data from the model and stash it
  here.
  */
void MarkerData::captureVirtualMarkers()
{
  dVector4 pt;
  C3dFloatFrame* frame = new C3dFloatFrame(markCnt);
  C3dFloatFrame* shadowFrame = new C3dFloatFrame(markCnt);


  for (int ii=0;ii<MARKER_COUNT;++ii) {
    if (cBody->markerToBody[ii].id>=0) {
      // Find the global coordinate of the
      // relative position to which the
      // marker is mapped
      dBodyGetRelPointPos(cBody->bodies[cBody->markerToBody[ii].id],
                          cBody->markerToBody[ii].pos[0],
                          cBody->markerToBody[ii].pos[1],
                          cBody->markerToBody[ii].pos[2],
                          pt);
      pt[3]=1;
    } else {
      pt[0]=pt[1]=pt[2]=0;
      pt[3]=-1;
    }
    frame->data[ii].point[0]=pt[0]*1000;
    frame->data[ii].point[1]=pt[1]*1000;
    frame->data[ii].point[2]=pt[2]*1000;
    frame->data[ii].point[3]=pt[3];
  }
  vData.push_back(frame);
  memcpy(shadowFrame->data,frame->data,markCnt*sizeof(FPoint));
  shadowData.push_back(shadowFrame);

}

void MarkerData::clearVirtualMarkers()
{
  int fCount = vData.size();
  for (int ii=0;ii<fCount;++ii) {
    delete(vData[ii]);
    delete(shadowData[ii]);
  }
  vData.clear();
  shadowData.clear();
}

void MarkerData::useVirtualMarkers(bool use)
{
  vMarkers = use;
  vPoint = 0;
}

/**
  We can't just perturb the frames on the fly because
  then we end up with a different perturbation every
  time which makes the velocity computation fall apart.
  */
void MarkerData::perturbShadowFrame(double sigma)
{
  int frameCnt = shadowData.size();
  for (int ii=0;ii<frameCnt;++ii) {
    for (int jj=0;jj<MARKER_COUNT;++jj) {
      for (int kk=0;kk<3;++kk) {
        double epsilon = sigma*randN();
        double val = vData[ii]->data[jj].point[kk] + epsilon;
        shadowData[ii]->data[jj].point[kk] = val;

      }
    }
  }
}
