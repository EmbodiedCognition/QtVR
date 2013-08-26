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
#include "simworld.h"
#include "CapBody.h"

#include "sequence.h"

SimWorld::SimWorld(QObject *parent) :
    QObject(parent)
{
  dInitODE();
  world = dWorldCreate();
  space = dHashSpaceCreate(0);
  contactGroup = dJointGroupCreate(0);

  // Setup gravity
  dWorldSetGravity (world,0,0,-9.8);

  // Setup constraint satisfaction parameters.
  dWorldSetERP (world, 0.2); // How hard the world pushes to fix unsatisfied constraints
  dWorldSetCFM(world, 0.000001);  // Constraint force mixing
  dWorldSetContactSurfaceLayer(world,.005); // How deeply an object can penetrate (in meters)
  dWorldSetMaxAngularSpeed(world,15); // A hard limit on how fast anything can spin
  //dWorldSetLinearDamping(world,.02);  // Linear Friction
  //dWorldSetAngularDamping(world,.02); // Angular friction
  stepsize=1/60.0;

  // Create the ground plane
  groundPlane = dCreatePlane (space,0,0,1,0.0);


  body = new CapBody(this);
  body->createBody(world,space);

  markSpace = dSimpleSpaceCreate(0);
  targetSpace = dSimpleSpaceCreate(0);

  //
  //markData = new SwingData(world,markSpace,this);
  //markData = new LiveMarkerData(world,markSpace,this);

#if defined( BOARD_DATA )
  markData = new BoardMarkerData(world,markSpace,this);
#elif defined( POKE_DATA )
  markData = new PokeMarkerData(world,markSpace,this);
#else
  markData = new MarkerData(world,markSpace,this);
#endif
  markData->cBody=body;

#if defined( BOARD_DATA )
  board0 = dCreateBox(space,.5,.5,.05);
  board1 = dCreateBox(space,.5,.5,.05);

  dGeomSetPosition(board0,0,.5,0.025);
  dGeomSetPosition(board1,0,-.5,0.025);
#endif

  paused=true;
  singleStep=false;

  angleSequence = new Sequence(this);
  torqueSequence = new Sequence(this);

  selfCollide=false;
  followState=0;

  groundSquish = .00001;
  groundFriction = 1;

  mainFile = fopen("totals.txt","w");

}

SimWorld::~SimWorld()
{
  fclose(mainFile);
  //fclose(markFile);

}

#define MAX_CONTACTS 4
void SimWorld::collideGeoms(void* data,dGeomID o1,dGeomID o2)
{
  dContact contact[MAX_CONTACTS];
  SimWorld* obj = static_cast<SimWorld*>(data);

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);

  if (b1 == NULL) {
    b1 = b2;
    b2 = NULL;
    dGeomID o3 = o1;
    o1 = o2;
    o2 = o3;
  }

  // Don't use body-body collisions for now.
  if (!obj->selfCollide && (b2!=NULL)) return;

  // exit without doing anything if the two bodies are connected by a joint
  if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact)) return;


  // If all bodies are kinematic, we don't need to do anything here since
  // adding constraints has no effect.
  if ( ((b1==NULL)||dBodyIsKinematic(b1)) && ((b2==NULL)||dBodyIsKinematic(b2))) return;

  // Exclude collisions between the upper arm and torso.
  if (((o1==obj->body->geoms[CapBody::UP_TORSO_BODY])||
       (o2==obj->body->geoms[CapBody::UP_TORSO_BODY])) &&
      ((o1==obj->body->geoms[CapBody::RUP_ARM_BODY])||
       (o2==obj->body->geoms[CapBody::RUP_ARM_BODY])||
       (o1==obj->body->geoms[CapBody::LUP_ARM_BODY])||
       (o2==obj->body->geoms[CapBody::LUP_ARM_BODY]))) return;




  contact[0].surface.mode = dContactSoftCFM | ((b1&&b2)?0:dContactApprox1); //Surfaces are just a little squishy
  contact[0].surface.soft_cfm = (b1&&b2)?0.001:obj->groundSquish; //
  contact[0].surface.bounce_vel = 0.01;
  contact[0].surface.bounce = 0.25;
  contact[0].surface.mu = (b1&&b2)?1:obj->groundFriction; // Strong friction with ground

  for (int ii=1;ii<MAX_CONTACTS;++ii) {
    contact[ii].surface.mode = contact[0].surface.mode;
    contact[ii].surface.soft_cfm = contact[0].surface.soft_cfm;
    contact[ii].surface.bounce_vel = contact[0].surface.bounce_vel;
    contact[ii].surface.bounce = contact[0].surface.bounce;
    contact[ii].surface.mu = contact[0].surface.mu;
  }

  int numC = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,sizeof(dContact) );

  for (int ii=0;ii<numC;++ii) {
    dJointID cc = dJointCreateContact (obj->world,obj->contactGroup,&contact[ii]);
    dJointAttach(cc,b1,b2);
    if ((b2==NULL)&&(obj->activeGroundContacts<MAX_GROUND_FEEDBACK)) {
      dJointSetFeedback(cc,&(obj->groundFeedback[obj->activeGroundContacts]));
      dOPE(obj->contactPoint[obj->activeGroundContacts],=,contact[ii].geom.pos);

      if ((b1==obj->body->bodies[CapBody::L_HEEL_BODY])
          ||(b1==obj->body->bodies[CapBody::L_TARSAL_BODY]))
          //||(b1==obj->body->bodies[CapBody::L_TOE_BODY]))
      {
        obj->leftFootFeedback.push_back(obj->activeGroundContacts);
      } else if ((b1==obj->body->bodies[CapBody::R_HEEL_BODY])
                 ||(b1==obj->body->bodies[CapBody::R_TARSAL_BODY]))
        //||(b1==obj->body->bodies[CapBody::R_TOE_BODY]))
      {
        obj->rightFootFeedback.push_back(obj->activeGroundContacts);
      }
      obj->activeGroundContacts+=1;
    }
  }
}

void SimWorld::updateSeqFrame()
{
  if (followState==1 || followState==3) {
    body->setTargetFrame(angleSequence->getFrame(seqFrame));
    seqFrame+=1;
  }
  if (followState==2) {

    //body->applyForceFrame(torqueSeq->getFrame(seqFrame));
    applyTorquesToBody(torqueSequence->getFrame(seqFrame));
    seqFrame+=1;

  }
}

void SimWorld::applyTorquesToBody(DataFrame* jf)
{
//  body->zeroAccumulators();
  body->applyForceFrame(jf);

  // What torques are we applying here?
  fprintf(mainFile,"%d ",markData->currentFrame);
//  for (int ii=0;ii<body->BODY_COUNT;++ii) {
//    const dReal* tau = dBodyGetTorque(body->bodies[ii]);
//    fprintf(mainFile,"%.20le %.20le %.20le ",tau[0],tau[1],tau[2]);
//  }
//  fprintf(mainFile,"  ");
  for (int jj=0;jj<body->JOINT_COUNT;++jj) {
    for (int kk=0;kk<3;kk++) {
      fprintf(mainFile,"%.20le ",jf->getData(jj,kk));
    }
  }
  fprintf(mainFile,"\n");
}


#if defined( BOARD_DATA )
void SimWorld::updateBoards()
{
  //Find avg pos
  double pos0[3]={0};
  double pos1[3]={0};

  for (int ii=0;ii<4;++ii) {
    pos0[0]+=markData->cFrame->boards[0][ii][2]*.00025;
    pos0[1]+=markData->cFrame->boards[0][ii][0]*.00025;
    pos0[2]+=markData->cFrame->boards[0][ii][1]*.00025;
    pos1[0]+=markData->cFrame->boards[1][ii][2]*.00025;
    pos1[1]+=markData->cFrame->boards[1][ii][0]*.00025;
    pos1[2]+=markData->cFrame->boards[1][ii][1]*.00025;
  }
  dGeomSetPosition(board0,pos0[0],pos0[1],pos0[2]*.5);
  dGeomSetPosition(board1,pos1[0],pos1[1],pos1[2]*.5);
  dGeomBoxSetLengths(board0,.25,.45,pos0[2]);
  dGeomBoxSetLengths(board1,.25,.45,pos1[2]);

}
#endif

void SimWorld::setStepSize(double ss)
{
  if (stepsize!=ss) {
    stepsize=ss;
    //Optional emit
    body->setStepSize(ss);
  }
}


/*
  Before each step, we need to capture the
  orientation of all the body parts.

  Afterward, when we get the joint feedback,
  we need to record it in the local frame
  of reference
  */

void SimWorld::step()
{


  markData->setStepTime(stepsize);
  markData->step();

#if defined( BOARD_DATA )
  updateBoards();
#endif
  if (singleStep || !paused) {

    dSpaceCollide (space,this,&collideGeoms);

    //********
    // We need to save/set the state _before_ we apply the
    // control signal.  Setting the body state
    // changes things ever so slightly by re-normalizing
    // the quaternions.

    // Record the current position and orientation of the bodies
    body->saveTempState();
    //This sets the body state to what it currently is
    // but it also ensures that the bodies are all in the
    // same order every time we go through.
    body->restoreTempState();

    // Set the target joint angles/stiffness
    // or apply forward dynamics torques
    updateSeqFrame();
    body->updateControl();

    // Get ready to capture ground forces on the feet.
    activeGroundContacts=0;
    leftFootFeedback.clear();
    rightFootFeedback.clear();
    //////////

    dWorldStep (world,stepsize);
    //////////
    body->findMassProperties();

    // Write out filtered marker positions
    //if (writeMark) {
    //  markData->writeRelPos(markFile);
    //}


    singleStep=false;
    JointFrame* jf;
    if (followState==0) {
      jf = new JointFrame;
      body->fillJointFrame(jf);
      angleSequence->appendFrame(jf);
    }
    if (followState==1) {
      jf = new JointFrame;
      body->fillForceFrame(jf);
      torqueSequence->appendFrame(jf);

#if defined( POKE_DATA )
      fprintf(mainFile,"%lf %lf %d %d %d ",
              markData->cFrame->time,
              markData->cFrame->trialTime,
              markData->cFrame->condition,
              markData->cFrame->trialState,
              markData->cFrame->trial);
#endif
      //jf->writeData(mainFile);
    }
    if (followState==3) {
      // Grab the torques
      jf = new JointFrame;
      body->fillForceFrame(jf);
      torqueSequence->appendFrame(jf);
      // Rewind
      body->restoreTempState();
      // Zero-control
      body->zeroControl();
      // Apply torques
      applyTorquesToBody(jf);

      // Step again (don't need to collide again...)
      // We keep the old collision joints
      dWorldStep (world,stepsize);
      body->findMassProperties();

      body->restoreControl();
    }

    dJointGroupEmpty (contactGroup);

    /////
    // If we're in replay, when conditions
    // indicate a new trial has begun,
    // create a new output file for the trial
    // write each joint torque out
    // write the cumulative torques out
    // write the total torque out
    // write the cumulative total torque out
    // At the end of the trial,
    //   write out the cumulative and total
    //   cumulative torques
    /////

    /*
    if (startingTrial()) {
      char filename[256];
      sprintf(filename,"trial%04d.txt",markData->cFrame->trial);
      trialFile = fopen(filename,"w");
      totalFrame->clear();
      fprintf(mainFile,"%d %d %lf %lf %lf ",
              markData->cFrame->trial,
              markData->cFrame->condition,
              markData->cFrame->target[0],
              markData->cFrame->target[1],
              markData->cFrame->target[2]);
    } else if (inMiddleTrial()) {
      totalFrame->addFrame(jf);
      totalFrame->writeData(trialFile);
    } else if (endingTrial()) {
      totalFrame->writeData(mainFile);
      fclose(trialFile);
      trialFile = 0;
    }*/


    /*if (!followMark) {
      fprintf(mainFile," %lf %lf %lf %lf ",
              markData->cFrame->boards[0][4][2]*.001,
              markData->cFrame->boards[0][4][0]*.001,
              markData->cFrame->boards[0][4][1]*.001,
              markData->cFrame->boards[0][4][3]*9.8);
      fprintf(mainFile," %lf %lf %lf %lf ",
              markData->cFrame->boards[1][4][2]*.001,
              markData->cFrame->boards[1][4][0]*.001,
              markData->cFrame->boards[1][4][1]*.001,
              markData->cFrame->boards[1][4][3]*9.8);
    }*/




    lineList.clear();

    GLine line;

    // Raw ground feedback lines
    line.col[0]=1;
    line.col[1]=1;
    line.col[2]=0;
    for (int ii=0;ii<activeGroundContacts;++ii) {
      dOPE(line.p1,=,contactPoint[ii]);
      line.p2[0] = contactPoint[ii][0]+groundFeedback[ii].f1[0]*.001;
      line.p2[1] = contactPoint[ii][1]+groundFeedback[ii].f1[1]*.001;
      line.p2[2] = contactPoint[ii][2]+groundFeedback[ii].f1[2]*.001;
      //lineList.push_back(line);
    }

    // Composite foot feedback lines
    double mag;
    int nn;
    dReal* pt;
    dReal  mg;
    line.col[0]=0;
    line.col[1]=1;
    line.col[2]=0;
    dOPEC(line.p1,=,0);
    dOPEC(line.p2,=,0);
    mag=0;
    nn=leftFootFeedback.size();
    for (int ii=0;ii<nn;++ii) {
      pt = contactPoint[leftFootFeedback[ii]];
      mg = groundFeedback[leftFootFeedback[ii]].f1[2];
      line.p1[0]+=pt[0]*mg;
      line.p1[1]+=pt[1]*mg;
      line.p1[2]+=pt[2]*mg;
      mag+=mg;
    }
    if (nn>0) {
      dOPEC(line.p1,/=,mag);
      dOPE(line.p2,=,line.p1);
      // 80kg (~800N) should be a 1.6m line
      line.p2[2]+=mag*.002;
      lineList.push_back(line);
    }


#if defined(BOARD_DATA)
    if (followMark==1) { fprintf(mainFile," %lf %lf %lf %lf ",
            line.p1[0],line.p1[1],line.p1[2],
            mag);
    }
#endif

    dOPEC(line.p1,=,0);
    dOPEC(line.p2,=,0);
    mag=0;
    nn=rightFootFeedback.size();
    for (int ii=0;ii<nn;++ii) {
      pt = contactPoint[rightFootFeedback[ii]];
      mg = groundFeedback[rightFootFeedback[ii]].f1[2];
      line.p1[0]+=pt[0]*mg;
      line.p1[1]+=pt[1]*mg;
      line.p1[2]+=pt[2]*mg;
      mag+=mg;
    }
    if (nn>0) {
      dOPEC(line.p1,/=,mag);
      dOPE(line.p2,=,line.p1);
      line.p2[2]+=mag*.002;
      lineList.push_back(line);
    }
#if defined(BOARD_DATA)
    if (followMark==1) { fprintf(mainFile," %lf %lf %lf %lf ",
           line.p1[0],line.p1[1],line.p1[2],
            mag);
    }
#endif


#if defined(BOARD_DATA)
    // Board feedback lines
    line.col[0]=1;
    line.col[1]=0;
    line.col[2]=0;

    line.p1[0] = markData->cFrame->boards[0][4][2]*0.001;
    line.p1[1] = markData->cFrame->boards[0][4][0]*0.001;
    line.p1[2] = markData->cFrame->boards[0][4][1]*0.001;
    line.p2[0] = line.p1[0];
    line.p2[1] = line.p1[1];
    line.p2[2] = line.p1[2] + markData->cFrame->boards[0][4][3]/40.0;
    lineList.push_back(line);

    line.p1[0] = markData->cFrame->boards[1][4][2]*0.001;
    line.p1[1] = markData->cFrame->boards[1][4][0]*0.001;
    line.p1[2] = markData->cFrame->boards[1][4][1]*0.001;
    line.p2[0] = line.p1[0];
    line.p2[1] = line.p1[1];
    line.p2[2] = line.p1[2] + markData->cFrame->boards[1][4][3]/40.0;
    lineList.push_back(line);


    if (followMark==1) {
      fprintf(mainFile," %lf %lf %lf %lf ",
              markData->cFrame->boards[0][4][2]*.001,
              markData->cFrame->boards[0][4][0]*.001,
              markData->cFrame->boards[0][4][1]*.001,
              markData->cFrame->boards[0][4][3]*9.8);
      fprintf(mainFile," %lf %lf %lf %lf\n",
              markData->cFrame->boards[1][4][2]*.001,
              markData->cFrame->boards[1][4][0]*.001,
              markData->cFrame->boards[1][4][1]*.001,
              markData->cFrame->boards[1][4][3]*9.8);
    }

#endif

    // Torques
    line.col[0]=.5;
    line.col[1]=0;
    line.col[2]=1;
    CapBody* bod = this->body;

    const dReal* pos = dBodyGetPosition(bod->bodies[CapBody::WAIST_BODY]);
    const dReal* torq = bod->feedback[CapBody::ROOT_ANGMOTOR_JOINT].t1;
    dOPE(line.p1,=,pos);
    line.p2[0]=pos[0]+torq[0]*.001;
    line.p2[1]=pos[1]+torq[1]*.001;
    line.p2[2]=pos[2]+torq[2]*.001;
    //lineList.push_back(line);

    // Center of mass projection
    pos = bod->com;
    line.col[0]=0;
    line.col[1]=0;
    line.col[2]=0;
    dOPE(line.p1,=,pos);
    dOPE(line.p2,=,pos);
    line.p2[2]=0;
    //lineList.push_back(line);

    // Linear velocity
    line.col[0]=1;
    line.col[1]=0;
    line.col[2]=0;
    line.p2[0]=pos[0]+bod->cLin[0]*.1;
    line.p2[1]=pos[1]+bod->cLin[1]*.1;
    line.p2[2]=pos[2]+bod->cLin[2]*.1;
    //lineList.push_back(line);

    // Angular velocity
    line.col[0]=0;
    line.col[1]=0;
    line.col[2]=1;
    line.p2[0]=pos[0]+bod->cAng[0]*.1;
    line.p2[1]=pos[1]+bod->cAng[1]*.1;
    line.p2[2]=pos[2]+bod->cAng[2]*.1;
    //lineList.push_back(line);

    //if (!followMark) {
    //  jf->writeData(mainFile);
    //}
  }



}


void SimWorld::pauseSim(bool pp)
{
  paused=pp;
}

void SimWorld::setSingleStep()
{
  singleStep=true;
  paused=true;

}

void SimWorld::setGroundFriction(double friction)
{
  groundFriction = friction;
}

void SimWorld::setTerrainSoftness(double terrain)
{
  groundSquish = terrain;
}

void SimWorld::setTerrainZ(double zVal)
{
  dGeomPlaneSetParams(groundPlane,0,0,1,zVal);
}


void SimWorld::follow(int type)
{
  followState = type;
  switch (type) {
  case 0:
    // Follow markers
    emit useMarkers(true);

    break;
  case 1:
    // Follow angles
    emit useMarkers(false);
    seqFrame = 0;

    break;
  case 2:
    // Follow torques
    emit useMarkers(false);
    seqFrame = 0;
    break;
  case 3:
    // Instead of blindly following torques generated as a sequence
    // We find and then apply the torques.
    emit useMarkers(false);
    seqFrame = 0;

    break;
  default:
    // Uh oh.
    break;
  }
}


void SimWorld::setSelfCollide(bool hitSelf)
{
  selfCollide=hitSelf;
}

void SimWorld::anglesToFile(QString filename)
{
  angleSequence->toFile(filename);
}


/**
  Write out all of the virtual marker positions
  and the joint angles.
  */
void SimWorld::writeVMarkerFrame(int cc,int tri,FILE* file)
{
  fprintf(file,"%d %d ",cc,tri);
  for (int ii=0;ii<MARKER_COUNT;++ii) {
    dVector4 pt;
    if (body->markerToBody[ii].id>=0) {
      // Find the global coordinate of the
      // relative position to which the
      // marker is mapped
      dBodyGetRelPointPos(body->bodies[body->markerToBody[ii].id],
                          body->markerToBody[ii].pos[0],
                          body->markerToBody[ii].pos[1],
                          body->markerToBody[ii].pos[2],
                          pt);
      pt[3]=1;
    } else {
      pt[0]=pt[1]=pt[2]=0;
      pt[3]=-1;
    }
    for (int kk=0;kk<4;++kk) {
      fprintf(file,"%lf ",pt[kk]);
    }
  }
  for (int jj=0;jj<body->JOINT_COUNT;++jj) {
    for (int kk=0;kk<3;++kk) {
      fprintf(file,"%lf ",body->jointValue(jj,kk));
    }
  }
  fprintf(file,"\n");
}

/**
  Write out virtual marker positions, joint angles
  and the torques used to achieve them.
  */
void SimWorld::writeAngleFrame(int cc,int tri,FILE* file)
{
  fprintf(file,"%d %d ",cc,tri);
  for (int ii=0;ii<MARKER_COUNT;++ii) {
    dVector4 pt;
    if (body->markerToBody[ii].id>=0) {
      // Find the global coordinate of the
      // relative position to which the
      // marker is mapped
      dBodyGetRelPointPos(body->bodies[body->markerToBody[ii].id],
                          body->markerToBody[ii].pos[0],
                          body->markerToBody[ii].pos[1],
                          body->markerToBody[ii].pos[2],
                          pt);
      pt[3]=1;
    } else {
      pt[0]=pt[1]=pt[2]=0;
      pt[3]=-1;
    }
    for (int kk=0;kk<4;++kk) {
      fprintf(file,"%lf ",pt[kk]);
    }
  }
  for (int jj=0;jj<body->JOINT_COUNT;++jj) {
    for (int kk=0;kk<3;++kk) {
      fprintf(file,"%lf ",body->jointValue(jj,kk));
    }
  }
  for (int jj=0;jj<body->JOINT_COUNT;++jj) {
    for (int kk=0;kk<3;++kk) {
      fprintf(file,"%lf ",body->forceValue(jj,kk));
    }
  }
  fprintf(file,"\n");
}

