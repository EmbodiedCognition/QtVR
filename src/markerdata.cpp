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
#include "gaussrand.h"

MarkerData::MarkerData(dWorldID world,dSpaceID space,QObject *parent) :
    QObject(parent)
{
  this->world=world;
  this->space=space;
  data.loadFile("data/data.c3d");
  std::cout << "[markerdata] Marker file loaded...";
  marker_count = data.data.pointsPerFrame;
  std::cout << "  ...this file contains " << marker_count << " markers" << std::endl;
  old_c_frame = c_frame = new C3dFloatFrame(marker_count);
  old_n_frame = n_frame = new C3dFloatFrame(marker_count);

  geom = new dGeomID[marker_count];
  body = new dBodyID[marker_count];
  joint = new dJointID[marker_count];
  try_link = new bool[marker_count];

  //space = dSimpleSpaceCreate(0);
  joint_group = dJointGroupCreate(0);
  for (int ii=0;ii<marker_count;++ii) {
    body[ii] = dBodyCreate(world);
    dBodySetKinematic(body[ii]);
    geom[ii] = dCreateSphere(space,.01);
    dGeomSetBody(geom[ii],body[ii]);
    try_link[ii]=false;
    joint[ii] = dJointCreateBall(world,joint_group);
  }



  frames_per_step=2;
  time_per_step = .015;
  current_frame = 0;
  paused=true;
  single_step=false;


  use_virtual_markers=false;

  grand = new GaussRand;

}

MarkerData::~MarkerData()
{
  delete grand;
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


  if (!use_virtual_markers) {
    data.readFrame(current_frame,c_frame);
    data.readFrame(current_frame+frames_per_step,n_frame);
  } else {
    if (virtual_point>=0 && virtual_point<shadow_data.size()) {
      c_frame = shadow_data[virtual_point];
      if (virtual_point==shadow_data.size()-1) {
        n_frame = shadow_data[virtual_point];
      } else {
        n_frame = shadow_data[virtual_point+1];
      }

    } else {
      c_frame = n_frame = shadow_data.last();
    }
  }
  dJointGroupEmpty(joint_group);

  if (paused && !single_step) {
    for (int ii=0;ii<marker_count;++ii) {
      float xx,yy,zz;
      xx = c_frame->data[ii].point[0]*.001;
      yy = c_frame->data[ii].point[1]*.001;
      zz = c_frame->data[ii].point[2]*.001;
      if (c_frame->data[ii].point[3]<0) zz=-1;
      dBodySetLinearVel(body[ii],0,0,0);
      dBodySetPosition(body[ii],xx,yy,zz);

    }

  } else {
    for (int ii=0;ii<marker_count;++ii) {
      float xx,yy,zz;
      xx = c_frame->data[ii].point[0]*.001;
      yy = c_frame->data[ii].point[1]*.001;
      zz = c_frame->data[ii].point[2]*.001;
      if (c_frame->data[ii].point[3]<0) zz=-1;

      float dx,dy,dz;
      dx = n_frame->data[ii].point[0]*.001 - xx;
      dy = n_frame->data[ii].point[1]*.001 - yy ;
      dz = n_frame->data[ii].point[2]*.001 - zz;

      dBodySetLinearVel(body[ii],dx/time_per_step,dy/time_per_step,dz/time_per_step);
      dBodySetPosition(body[ii],xx,yy,zz);
    }
    //currentFrame+=framesPerStep;
    setFrame(current_frame+frames_per_step);
    if (use_virtual_markers) virtual_point+=1;
    single_step = false;

  }




  for (int ii=0;ii<marker_count;++ii) {
    int bID = body_pointer->marker_to_body[ii].id;

    if (try_link[ii] && (bID>=0) &&
        (c_frame->data[ii].point[3]>0) &&
        (n_frame->data[ii].point[3]>0))
    {
      joint[ii]=dJointCreateBall(world,joint_group);

      dJointAttach(joint[ii],body[ii],body_pointer->body_segments[bID]);
      dJointSetBallAnchor1Rel(joint[ii],0,0,0);
      dJointSetBallAnchor2Rel(joint[ii],
                              body_pointer->marker_to_body[ii].position[0],
                              body_pointer->marker_to_body[ii].position[1],
                              body_pointer->marker_to_body[ii].position[2]);
      dJointSetBallParam(joint[ii],dParamCFM,.0001);
      dJointSetBallParam(joint[ii],dParamERP,.2);
    }

  }

  if (use_virtual_markers){
    c_frame = old_c_frame;
    n_frame = old_n_frame;
  }


}


void MarkerData::setPaused(bool playing)
{
  this->paused = !playing;
}

void MarkerData::setSingleStep()
{
  single_step=true;
  paused = true;
}

void MarkerData::setFrame(int frame)
{
  if (frame>=0 && frame<size() && current_frame!=frame) {
    current_frame=frame;
    emit frameChanged(frame);

  }

}

void MarkerData::changeBodyConnect(int mark,int body)
{
  body_pointer->marker_to_body[mark].id=body;
}

void MarkerData::changeBodyLink(int mark,bool link)
{
  try_link[mark]=link;
}

void MarkerData::changeLinkPos(int mark,double xx,double yy,double zz)
{
  body_pointer->marker_to_body[mark].position[0]=xx;
  body_pointer->marker_to_body[mark].position[1]=yy;
  body_pointer->marker_to_body[mark].position[2]=zz;
}


/**
  Grab the data from the model and stash it
  here.
  */
void MarkerData::captureVirtualMarkers()
{
  dVector4 pt;
  C3dFloatFrame* frame = new C3dFloatFrame(marker_count);
  C3dFloatFrame* shadowFrame = new C3dFloatFrame(marker_count);


  for (int ii=0;ii<marker_count;++ii) {
    if (body_pointer->marker_to_body[ii].id>=0) {
      // Find the global coordinate of the
      // relative position to which the
      // marker is mapped
      dBodyGetRelPointPos(body_pointer->body_segments[body_pointer->marker_to_body[ii].id],
                          body_pointer->marker_to_body[ii].position[0],
                          body_pointer->marker_to_body[ii].position[1],
                          body_pointer->marker_to_body[ii].position[2],
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
  virtual_data.push_back(frame);
  memcpy(shadowFrame->data,frame->data,marker_count*sizeof(FPoint));
  shadow_data.push_back(shadowFrame);

}

void MarkerData::clearVirtualMarkers()
{
  int fCount = virtual_data.size();
  for (int ii=0;ii<fCount;++ii) {
    delete(virtual_data[ii]);
    delete(shadow_data[ii]);
  }
  virtual_data.clear();
  shadow_data.clear();
}

void MarkerData::useVirtualMarkers(bool use)
{
  use_virtual_markers = use;
  virtual_point = 0;
}

/**
  We can't just perturb the frames on the fly because
  then we end up with a different perturbation every
  time which makes the velocity computation fall apart.
  */
void MarkerData::perturbShadowFrame(double sigma)
{
  int frameCnt = shadow_data.size();
  for (int ii=0;ii<frameCnt;++ii) {
    for (int jj=0;jj<marker_count;++jj) {
      for (int kk=0;kk<3;++kk) {
        double epsilon = sigma*grand->next();
        double val = virtual_data[ii]->data[jj].point[kk] + epsilon;
        shadow_data[ii]->data[jj].point[kk] = val;

      }
    }
  }
}
