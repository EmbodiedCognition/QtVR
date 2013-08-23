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
#include "swingdata.h"


#include "CapBody.h"

SwingData::SwingData(dWorldID world,dSpaceID space,QObject *parent) :
    QObject(parent)
{
  this->world=world;
  this->space=space;

  loadData();
  markCnt=50;
  //data.loadFile("data.c3d");

  //markCnt = data.data.pointsPerFrame;
  //cFrame = new C3dFloatFrame(markCnt);
  //nFrame = new C3dFloatFrame(markCnt);

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
  timePerStep = .015;
  currentFrame = 0;
  paused=true;
  singleStep=false;


}

void SwingData::loadData()
{
  FILE* datFile = fopen("swingData.txt","r");

  SwingDataFrame df;
  while (!feof(datFile)) {
    fscanf(datFile," %u %d %d ",&(df.frame),&(df.trial),&(df.state));
    fscanf(datFile," %lf %lf %lf ",&(df.tPos[0]),&(df.tPos[1]),&(df.tPos[2]));
    fscanf(datFile," %lf %lf %lf ",&(df.sPos[0]),&(df.sPos[1]),&(df.sPos[2]));
    fscanf(datFile," %lf %lf %lf ",&(df.gPos[0]),&(df.gPos[1]),&(df.gPos[2]));
    fscanf(datFile," %lf %lf %lf ",&(df.bPos[0]),&(df.bPos[1]),&(df.bPos[2]));
    fscanf(datFile," %lf %lf %lf ",&(df.rPos[0]),&(df.rPos[1]),&(df.rPos[2]));
    fscanf(datFile," %lf %lf %lf %lf ",
           &(df.rQuat[0]),&(df.rQuat[1]),
           &(df.rQuat[2]),&(df.rQuat[3]));
    fscanf(datFile," %lf %lf %lf ",&(df.hPos[0]),&(df.hPos[1]),&(df.hPos[2]));
    fscanf(datFile," %lf %lf %lf %lf ",
           &(df.hQuat[0]),&(df.hQuat[1]),
           &(df.hQuat[2]),&(df.hQuat[3]));
    for (int ii=0;ii<50;++ii) {
      fscanf(datFile," %lf %lf %lf %lf ",
             &(df.mark[ii][0]),&(df.mark[ii][1]),
             &(df.mark[ii][2]),&(df.mark[ii][3]));
    }
    data.push_back(df);
  }

  fclose(datFile);

}

int SwingData::size()
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
void SwingData::step()
{
  // Set the markers to hold the
  // current frame

  // Get the current frame
  // Get the next frame

  //data.readFrame(currentFrame,cFrame);
  //data.readFrame(currentFrame+framesPerStep,nFrame);

  int cc = currentFrame;
  int nn = cc + framesPerStep;
  if (cc>=(int)data.size()) cc=data.size()-1;
  if (nn>=(int)data.size()) nn=data.size()-1;
  if (cc<0) cc=0;
  if (nn<0) nn=0;


  cFrame = &(data[cc]);
  nFrame = &(data[nn]);

  dJointGroupEmpty(jointGroup);

  if (paused && !singleStep) {
    for (int ii=0;ii<markCnt;++ii) {
      float xx,yy,zz;
      xx = cFrame->mark[ii][0];
      yy = cFrame->mark[ii][1];
      zz = cFrame->mark[ii][2];
      if (cFrame->mark[ii][3]<0) zz=-1;
      dBodySetLinearVel(body[ii],0,0,0);
      dBodySetPosition(body[ii],xx,yy,zz);

    }

  } else {
    for (int ii=0;ii<markCnt;++ii) {
      float xx,yy,zz;
      xx = cFrame->mark[ii][0];
      yy = cFrame->mark[ii][1];
      zz = cFrame->mark[ii][2];
      if (cFrame->mark[ii][3]<0) zz=-1;

      float dx,dy,dz;
      dx = nFrame->mark[ii][0] - xx;
      dy = nFrame->mark[ii][1] - yy ;
      dz = nFrame->mark[ii][2] - zz;

      dBodySetLinearVel(body[ii],dx/timePerStep,dy/timePerStep,dz/timePerStep);
      dBodySetPosition(body[ii],xx,yy,zz);
    }
    //currentFrame+=framesPerStep;
    setFrame(currentFrame+framesPerStep);
    singleStep = false;

  }

  for (int ii=0;ii<MARKER_COUNT;++ii) {
    int bID = cBody->markerToBody[ii].id;
    //if (bID!=CapBody::HEAD_BODY &&
//        bID!=CapBody::L_HEEL_BODY &&
//        bID!=CapBody::R_HEEL_BODY) continue;
    if (tryLink[ii] && (bID>=0) &&
        (cFrame->mark[ii][3]>0) &&
        (nFrame->mark[ii][3]>0))

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


void SwingData::setPaused(bool playing)
{
  this->paused = !playing;
}

void SwingData::setSingleStep()
{
  singleStep=true;
  paused = true;
}

void SwingData::setFrame(int frame)
{
  if (frame>=0 && frame<size() && currentFrame!=frame) {
    currentFrame=frame;
    emit frameChanged(frame);
  }
}

void SwingData::changeBodyConnect(int mark,int body)
{
  cBody->markerToBody[mark].id=body;
}

void SwingData::changeBodyLink(int mark,bool link)
{
  tryLink[mark]=link;
}

void SwingData::changeLinkPos(int mm,double xx,double yy,double zz)
{
  cBody->markerToBody[mm].pos[0]=xx;
  cBody->markerToBody[mm].pos[1]=yy;
  cBody->markerToBody[mm].pos[2]=zz;
}


/**
   Write out the current frame, but replace the marker data
   with the body position.
  */
void SwingData::writeRelPos(FILE* file)
{
  SwingDataFrame* df = nFrame;
  dVector3 pt={0};





  fprintf(file," %u %d %d ",(df->frame),(df->trial),(df->state));
  fprintf(file," %lf %lf %lf ",(df->tPos[0]),(df->tPos[1]),(df->tPos[2]));
  fprintf(file," %lf %lf %lf ",(df->sPos[0]),(df->sPos[1]),(df->sPos[2]));
  fprintf(file," %lf %lf %lf ",(df->gPos[0]),(df->gPos[1]),(df->gPos[2]));
  fprintf(file," %lf %lf %lf ",(df->bPos[0]),(df->bPos[1]),(df->bPos[2]));
  fprintf(file," %lf %lf %lf ",(df->rPos[0]),(df->rPos[1]),(df->rPos[2]));
  fprintf(file," %lf %lf %lf %lf ",
         (df->rQuat[0]),(df->rQuat[1]),
         (df->rQuat[2]),(df->rQuat[3]));
  fprintf(file," %lf %lf %lf ",(df->hPos[0]),(df->hPos[1]),(df->hPos[2]));
  fprintf(file," %lf %lf %lf %lf ",
         (df->hQuat[0]),(df->hQuat[1]),
         (df->hQuat[2]),(df->hQuat[3]));
  for (int ii=0;ii<50;++ii) {
    if (cBody->markerToBody[ii].id>=0) {
      // Find the global coordinate of the
      // relative position to which the
      // marker is mapped
      dBodyGetRelPointPos(cBody->bodies[cBody->markerToBody[ii].id],
                          cBody->markerToBody[ii].pos[0],
                          cBody->markerToBody[ii].pos[1],
                          cBody->markerToBody[ii].pos[2],
                          pt);
      dReal tmp = 1;
      fprintf(file," %lf %lf %lf %lf ",
           pt[0],pt[1],pt[2],tmp);
    } else {
      // Write out original data if this marker
      // isn't mapped to a body.
      fprintf(file," %lf %lf %lf %lf ",
           (df->mark[ii][0]),(df->mark[ii][1]),
           (df->mark[ii][2]),(df->mark[ii][3]));
    }
  }
  fprintf(file,"\n");
}




