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
#include "CapBody.h"

#include <QFile>
#include <QTextStream>

#include "sequence.h"
#include "dsboxwithparams.h"

#define FMAX 250
#define INTERNALCFM 0
#define CUP(x) (fabs(x)<1.0e-5?(0):(x))


dReal CapBody::density = 1000.0; // Density of water.  Close enough.
//dReal CapBody::density = 800.0; // Water density puts me at 250 lbs

CapBody::CapBody(QObject *parent)
  : QObject(parent)
{
  for (int ii=0;ii<JOINT_COUNT;++ii) {
    joint_target[ii][0]=0;
    joint_target[ii][1]=0;
    joint_target[ii][2]=0;

    joint_offset[ii][0]=0;
    joint_offset[ii][1]=0;
    joint_offset[ii][2]=0;

    joints[ii] = 0;
    limits[ii] = 0;
    motors[ii] = 0;
  }
  stepsize = .015;

  keep_rel_anchors=true;
  global_forces = false;
}

CapBody::~CapBody(void)
{
}

void CapBody::fillJointFrame(JointFrame* jf)
{
  for (int ii=0;ii<JOINT_COUNT;++ii) {
    for (int ax=0;ax<3;++ax) {
      jf->setData(ii,ax,jointValue(ii,ax));
    }
  }
}

void CapBody::fillForceFrame(JointFrame* jf)
{
  for (int ii=0;ii<JOINT_COUNT;++ii) {
    for (int ax=0;ax<3;++ax) {
      double val = forceValue(ii,ax);
      jf->setData(ii,ax,val);

    }
  }
}

void CapBody::setTargetFrame(DataFrame* jf)
{
  if (!jf) return;
  for (int ii=0;ii<JOINT_COUNT;++ii) {
    for (int ax=0;ax<3;++ax) {
      setJointTarget(ii,ax,jf->getData(ii,ax));
      //emit targetChange(ii,ax,jf->getData(ii,ax));
    }
  }
}

void CapBody::applyForceFrame(DataFrame* jf)
{
  if (!jf) return;
  for (int ii=0;ii<JOINT_COUNT;++ii) {
    for (int ax=0;ax<3;++ax) {
      double val = jf->getData(ii,ax);

      applyJointForce(ii,ax,val);
    }
  }
}

void CapBody::createBody(dWorldID world,dSpaceID space)
{
  this->world=world;
  this->space=space;

  // We rotate some capsules when placing them
  // So we build quaternions representing that
  // rotation.
  dQuaternion turnRight;
  dQuaternion turnLeft;
  dQFromAxisAndAngle(turnRight,0,1,0,M_PI/2);
  dQFromAxisAndAngle(turnLeft,0,1,0,-M_PI/2);

  // We build all of the body parts first
  // with the desired dimensions (should
  // probaly be loaded from file)
  createSphere(HEAD_BODY,    HEAD_GEOM,    .13 );
  createBox(   NECK_BODY,    NECK_GEOM,    .08,.08,.08);
  createCaps(  UP_TORSO_BODY,UP_TORSO_GEOM,.14,.20);
  dBodySetQuaternion(body_segments[UP_TORSO_BODY],turnLeft);
  
  createCaps(  R_COLLAR_BODY,R_COLLAR_GEOM,.06,.09);
  createCaps(  RUP_ARM_BODY, RUP_ARM_GEOM, .05,.27);
  createCaps(  RLO_ARM_BODY, RLO_ARM_GEOM, .04 ,.17);
  createSphere(R_HAND_BODY,  R_HAND_GEOM,  .045);
  dBodySetQuaternion(body_segments[R_COLLAR_BODY],turnLeft);
  
  createCaps(  L_COLLAR_BODY,L_COLLAR_GEOM,.06,.09);
  createCaps(  LUP_ARM_BODY, LUP_ARM_GEOM, .05,.27);
  createCaps(  LLO_ARM_BODY, LLO_ARM_GEOM, .04 ,.17);
  createSphere(L_HAND_BODY,  L_HAND_GEOM,  .045);
  dBodySetQuaternion(body_segments[L_COLLAR_BODY],turnRight);

  createBox(   LO_TORSO_BODY,LO_TORSO_GEOM,.35,.15,.20);
  createCaps(  WAIST_BODY,   WAIST_GEOM,   .07,.20);
  dBodySetQuaternion(body_segments[WAIST_BODY],turnLeft);
  
  createCaps(  RUP_LEG_BODY, RUP_LEG_GEOM, .10,.30);
  createCaps(  RLO_LEG_BODY, RLO_LEG_GEOM, .08,.28);
  createSphere(R_HEEL_BODY,  R_HEEL_GEOM,  .07);
  createCaps(  R_TARSAL_BODY,   R_TARSAL_GEOM,   .03,.04);
  dBodySetQuaternion(body_segments[R_TARSAL_BODY],turnLeft);
  
  createCaps(  LUP_LEG_BODY, LUP_LEG_GEOM, .10,.30);
  createCaps(  LLO_LEG_BODY, LLO_LEG_GEOM, .08,.28);
  createSphere(L_HEEL_BODY,  L_HEEL_GEOM,  .07);
  createCaps(  L_TARSAL_BODY,   L_TARSAL_GEOM,   .03,.04);
  dBodySetQuaternion(body_segments[L_TARSAL_BODY],turnRight);

  // Now the body parts are built, we move the head
  // and then begin creating joints and attaching
  // them from the head down
  dBodySetPosition(body_segments[HEAD_BODY],0,0,2);

  createBall( CHIN_JOINT,HEAD_BODY,NECK_BODY,
    0,-.2,-.85,
    0, 0, .95,
    1,0,0,
    0,1,0,
    -M_PI/3,M_PI/4,
    -M_PI/4,M_PI/4,
    -M_PI/5,M_PI/5);

  createBall( THROAT_JOINT,NECK_BODY,UP_TORSO_BODY,
    0,0,-.95,
    .9,-.2, 0,
    1,0,0,
    0,1,0,
    -M_PI/4,M_PI/9,
    -M_PI/4,M_PI/4,
    -M_PI/9,M_PI/9);

  createBall( R_COLLAR_JOINT,UP_TORSO_BODY,R_COLLAR_BODY,
    .75,0,-.35,
    0,0,1,
    1,0,0,
    0,1,0,
    -M_PI/6,M_PI/6,
    -M_PI/4,M_PI/4,
    -M_PI/6,M_PI/4);

  createBall( R_SHOULDER_JOINT,R_COLLAR_BODY,RUP_ARM_BODY,
    0,0,-.9,
    0,0,.85,
    1,0,0,
    0,1,0,
    -5*M_PI/9,5*M_PI/9,
    -M_PI/3,M_PI/3,
    -M_PI/4,2*M_PI/3);

  createUni( R_ELBOW_JOINT,RUP_ARM_BODY,RLO_ARM_BODY,
    0,0,-.95,
    0,0,.95,
    1,0,0,
    0,0,1,
    -4*M_PI/5,0.01,
    -M_PI/2,M_PI/2);
  createUni( R_WRIST_JOINT,RLO_ARM_BODY,R_HAND_BODY,
    0,0,-.95,
    0,0,1,
    1,0,0,
    0,1,0,
    -M_PI/5,M_PI/5,
    -M_PI/2,M_PI/2);

  createBall( L_COLLAR_JOINT,UP_TORSO_BODY,L_COLLAR_BODY,
    .75,0,.35,
    0,0,1,
    1,0,0,
    0,1,0,
    -M_PI/6,M_PI/6,
    -M_PI/4,M_PI/4,
    -M_PI/4,M_PI/6);

  createBall( L_SHOULDER_JOINT,L_COLLAR_BODY,LUP_ARM_BODY,
    0,0,-.9,
    0,0,.85,
    1,0,0,
    0,1,0,
    -5*M_PI/9,5*M_PI/9,
    -M_PI/3,M_PI/3,
    -2*M_PI/3,M_PI/4);

  createUni( L_ELBOW_JOINT,LUP_ARM_BODY,LLO_ARM_BODY,
    0,0,-.95,
    0,0,.95,
    1,0,0,
    0,0,1,
    -4*M_PI/5,0.01,
    -M_PI/2,M_PI/2);
  createUni( L_WRIST_JOINT,LLO_ARM_BODY,L_HAND_BODY,
    0,0,-.95,
    0,0,1,
    1,0,0,
    0,1,0,
    -M_PI/5,M_PI/5,
    -M_PI/2,M_PI/2);

  createBall( SPINE_JOINT,UP_TORSO_BODY,LO_TORSO_BODY,
    -.95,-.1,0,
    0,0,.90,
    1,0,0,
    0,1,0,
    - M_PI/4, M_PI/6,
    - M_PI/4, M_PI/4,
    - M_PI/6, M_PI/6);

  createBall( WAIST_JOINT,LO_TORSO_BODY,WAIST_BODY,
    0,0,-.9,
    0.5,0,0,
    1,0,0,
    0,1,0,
    - M_PI/4, M_PI/6,
    - M_PI/3, M_PI/3,
    - M_PI/6, M_PI/6);

  createBall( R_HIP_JOINT,WAIST_BODY,RUP_LEG_BODY,
    0,0,-.6,
    0,0,.95,
    1,0,0,
    0,1,0,
    -2*M_PI/3,M_PI/3,
    -M_PI/3,M_PI/3,
    -M_PI/6,2*M_PI/3);

  createUni( R_KNEE_JOINT,RUP_LEG_BODY,RLO_LEG_BODY,
    0,0,-.95,
    0,0,.95,
    1,0,0,
    0,0,1,
    -0.01,4*M_PI/5,
    -M_PI/5,M_PI/5);

  createUni( R_ANKLE_JOINT,RLO_LEG_BODY,R_HEEL_BODY,
            0,0,-.95,
            0,0,1,
            1,0,0,
            0,1,0,
            -M_PI/3,M_PI/3,
            -M_PI/3,M_PI/3);

  createHinge(R_FOOT_JOINT,R_HEEL_BODY,R_TARSAL_BODY,
    0,1.5,-1,
    -1,0,0,
    0,1,0,
    -M_PI/16,M_PI/16);

//  createHinge(R_TOE_JOINT,R_TARSAL_BODY,R_TOE_BODY,
//    0,0,0,
//    0,-2,0,
//    1,0,0,
//    -M_PI/8,M_PI/8);

  createBall( L_HIP_JOINT,WAIST_BODY,LUP_LEG_BODY,
    0,0,.6,
    0,0,.95,
    1,0,0,
    0,1,0,
    -2*M_PI/3,M_PI/3,
    -M_PI/3,M_PI/3,
    -2*M_PI/3,M_PI/6);

  createUni( L_KNEE_JOINT,LUP_LEG_BODY,LLO_LEG_BODY,
    0,0,-.95,
    0,0,.95,
    1,0,0,
    0,0,1,
    -0.01,4*M_PI/5,
    -M_PI/5,M_PI/5);

  createUni( L_ANKLE_JOINT,LLO_LEG_BODY,L_HEEL_BODY,
            0,0,-.95,
            0,0,1,
            1,0,0,
            0,1,0,
            -M_PI/3,M_PI/3,
            -M_PI/3,M_PI/3);

  createHinge(L_FOOT_JOINT,L_HEEL_BODY,L_TARSAL_BODY,
    0,1.5,-1,
    1,0,0,
    0,1,0,
    -M_PI/16,M_PI/16);

//  createHinge(L_TOE_JOINT,L_TARSAL_BODY,L_TOE_BODY,
//    0,0,0,
//    0,-2,0,
//    1,0,0,
//    -M_PI/8,M_PI/8);

  BodyType rootBody = HEAD_BODY;


  motors[ROOT_LINMOTOR_JOINT] =
  joints[ROOT_LINMOTOR_JOINT] = dJointCreateLMotor(world,0);
  dJointAttach(joints[ROOT_LINMOTOR_JOINT],body_segments[rootBody],0);
  dJointSetLMotorNumAxes(joints[ROOT_LINMOTOR_JOINT],3);
  dJointSetLMotorAxis(joints[ROOT_LINMOTOR_JOINT],0,0,1,0,0);
  dJointSetLMotorAxis(joints[ROOT_LINMOTOR_JOINT],1,0,0,1,0);
  dJointSetLMotorAxis(joints[ROOT_LINMOTOR_JOINT],2,0,0,0,1);
  dJointSetLMotorParam(joints[ROOT_LINMOTOR_JOINT],dParamVel1,0);
  dJointSetLMotorParam(joints[ROOT_LINMOTOR_JOINT],dParamVel2,0);
  dJointSetLMotorParam(joints[ROOT_LINMOTOR_JOINT],dParamVel3,0);
  dJointSetLMotorParam(joints[ROOT_LINMOTOR_JOINT],dParamCFM1,1e-10);
  dJointSetLMotorParam(joints[ROOT_LINMOTOR_JOINT],dParamCFM2,1e-10);
  dJointSetLMotorParam(joints[ROOT_LINMOTOR_JOINT],dParamCFM3,1e-10);
  dJointSetLMotorParam(joints[ROOT_LINMOTOR_JOINT],dParamFMax1,FMAX);
  dJointSetLMotorParam(joints[ROOT_LINMOTOR_JOINT],dParamFMax2,FMAX);
  dJointSetLMotorParam(joints[ROOT_LINMOTOR_JOINT],dParamFMax3,FMAX);
  joint_info[ROOT_LINMOTOR_JOINT].link[0].id=rootBody;
  joint_info[ROOT_LINMOTOR_JOINT].link[1].id=-1;

  limits[ROOT_ANGMOTOR_JOINT] =
  joints[ROOT_ANGMOTOR_JOINT] = dJointCreateAMotor(world,0);
  dJointAttach(joints[ROOT_ANGMOTOR_JOINT],body_segments[rootBody],0);
  dJointSetAMotorNumAxes(joints[ROOT_ANGMOTOR_JOINT],3);
  dJointSetAMotorMode(joints[ROOT_ANGMOTOR_JOINT],dAMotorEuler);
  dJointSetAMotorAxis(joints[ROOT_ANGMOTOR_JOINT],0,1,1,0,0);
  dJointSetAMotorAxis(joints[ROOT_ANGMOTOR_JOINT],2,2,0,0,1);
  dJointSetAMotorParam(joints[ROOT_ANGMOTOR_JOINT],dParamLoStop1,-4*M_PI/9);
  dJointSetAMotorParam(joints[ROOT_ANGMOTOR_JOINT],dParamHiStop1, 4*M_PI/9);


  motors[ROOT_ANGMOTOR_JOINT] = dJointCreateAMotor(world,0);
  dJointAttach(motors[ROOT_ANGMOTOR_JOINT],body_segments[rootBody],0);
  dJointSetAMotorNumAxes(motors[ROOT_ANGMOTOR_JOINT],3);
  dJointSetAMotorMode(motors[ROOT_ANGMOTOR_JOINT],dAMotorEuler);
  dJointSetAMotorAxis(motors[ROOT_ANGMOTOR_JOINT],0,1,1,0,0);
  dJointSetAMotorAxis(motors[ROOT_ANGMOTOR_JOINT],2,2,0,0,1);
  dJointSetAMotorParam(motors[ROOT_ANGMOTOR_JOINT],dParamVel1,0);
  dJointSetAMotorParam(motors[ROOT_ANGMOTOR_JOINT],dParamVel2,0);
  dJointSetAMotorParam(motors[ROOT_ANGMOTOR_JOINT],dParamVel3,0);
  dJointSetAMotorParam(joints[ROOT_ANGMOTOR_JOINT],dParamCFM1,1e-10);
  dJointSetAMotorParam(joints[ROOT_ANGMOTOR_JOINT],dParamCFM2,1e-10);
  dJointSetAMotorParam(joints[ROOT_ANGMOTOR_JOINT],dParamCFM3,1e-10);
  dJointSetAMotorParam(motors[ROOT_ANGMOTOR_JOINT],dParamFMax1,FMAX);
  dJointSetAMotorParam(motors[ROOT_ANGMOTOR_JOINT],dParamFMax2,FMAX);
  dJointSetAMotorParam(motors[ROOT_ANGMOTOR_JOINT],dParamFMax3,FMAX);




  joint_info[ROOT_ANGMOTOR_JOINT].link[0].id=rootBody;
  joint_info[ROOT_ANGMOTOR_JOINT].link[1].id=-1;

  for (int ii=0;ii<JOINT_COUNT;++ii) {
    dJointSetFeedback(motors[ii],&(feedback[ii]));
  }

  for (int ii=0;ii<BODY_COUNT;++ii) {
    getBody(ii);
    dBodySetData(body_segments[ii],(void*)ii);
  }
}

void CapBody::createSphere(BodyType id,GeomType g,dReal radius)
{
  dMass m;
  
  body_segments[id] = dBodyCreate(world);
  geometries[g] = dCreateSphere(space,radius);
  dMassSetSphere(&m,density,radius);
  dBodySetMass(body_segments[id],&m);
  dGeomSetBody(geometries[g],body_segments[id]);
}

void CapBody::createCaps(BodyType id,GeomType g,dReal radius,dReal zLen)
{
  dMass m;
  
  body_segments[id] = dBodyCreate(world);
  geometries[g] = dCreateCapsule(space,radius,zLen);
  dMassSetCapsule(&m,density,3,radius,zLen);
  dBodySetMass(body_segments[id],&m);
  dGeomSetBody(geometries[g],body_segments[id]);
}

void CapBody::createBox(BodyType id,GeomType g,dReal xLen,dReal yLen,dReal zLen)
{
  dMass m;
  
  body_segments[id] = dBodyCreate(world);
  geometries[g] = dCreateBox(space,xLen,yLen,zLen);
  dMassSetBox(&m,density,xLen,yLen,zLen);
  dBodySetMass(body_segments[id],&m);
  dGeomSetBody(geometries[g],body_segments[id]);
}

/**
  Create a ball-socket joint.
  Attach it to id1 and id2
  With relative anchors (px,py,pz)
  We move the second body so that
  it's in the right relative location.
  */
void CapBody::createBall(JointType jj,BodyType id1,BodyType id2,
                         dReal px1,dReal py1,dReal pz1,
                         dReal px2,dReal py2,dReal pz2,
                         dReal ax1,dReal ay1,dReal az1,
                         dReal ax3,dReal ay3,dReal az3,
                         dReal lo1,dReal hi1,
                         dReal lo2,dReal hi2,
                         dReal lo3,dReal hi3)
{
  dVector3 dim1;
  dVector3 dim2;
  dVector3 pp1;
  dVector3 pp2;

  // Create the ball-and-socket joint
  joints[jj] = dJointCreateBall(world,0);
  dJointAttach(joints[jj],body_segments[id1],body_segments[id2]);
  dJointSetBallParam(joints[jj],dParamCFM,INTERNALCFM);

  bodyDims(id1,dim1);
  bodyDims(id2,dim2);
  const dReal* pos2 = dBodyGetPosition(body_segments[id2]);
  dBodyGetRelPointPos(body_segments[id1],px1*dim1[0]/2,py1*dim1[1]/2,pz1*dim1[2]/2,pp1);
  dBodyGetRelPointPos(body_segments[id2],px2*dim2[0]/2,py2*dim2[1]/2,pz2*dim2[2]/2,pp2);
  dBodySetPosition(body_segments[id2],
    pp1[0]-pp2[0]+pos2[0],
    pp1[1]-pp2[1]+pos2[1],
    pp1[2]-pp2[2]+pos2[2]);
  dJointSetBallAnchor(joints[jj],pp1[0],pp1[1],pp1[2]);

  // Create the AMotor in Euler mode for applying limits
  limits[jj] = dJointCreateAMotor(world,0);
  dJointAttach(limits[jj],body_segments[id1],body_segments[id2]);
  dJointSetAMotorMode( limits[jj], dAMotorEuler);
  dJointSetAMotorAxis(limits[jj],0,1,ax1,ay1,az1);
  dJointSetAMotorAxis(limits[jj],2,2,ax3,ay3,az3);
  dJointSetAMotorParam(limits[jj],dParamLoStop1,lo1);
  dJointSetAMotorParam(limits[jj],dParamHiStop1,hi1);
  dJointSetAMotorParam(limits[jj],dParamLoStop2,lo2);
  dJointSetAMotorParam(limits[jj],dParamHiStop2,hi2);
  dJointSetAMotorParam(limits[jj],dParamLoStop3,lo3);
  dJointSetAMotorParam(limits[jj],dParamHiStop3,hi3);

  // Create the AMotors for achieving target joint angles
  motors[jj] = dJointCreateAMotor(world,0);
  dJointAttach(motors[jj],body_segments[id1],body_segments[id2]);
  dJointSetAMotorMode( motors[jj], dAMotorEuler);
  dJointSetAMotorAxis(motors[jj],0,1,ax1,ay1,az1);
  dJointSetAMotorAxis(motors[jj],2,2,ax3,ay3,az3);
  dJointSetAMotorParam(motors[jj],dParamCFM1,1e-10);
  dJointSetAMotorParam(motors[jj],dParamCFM2,1e-10);
  dJointSetAMotorParam(motors[jj],dParamCFM3,1e-10);
  dJointSetAMotorParam(motors[jj],dParamFMax1,FMAX);
  dJointSetAMotorParam(motors[jj],dParamFMax2,FMAX);
  dJointSetAMotorParam(motors[jj],dParamFMax3,FMAX);

  joint_info[jj].link[0].id=id1;
  joint_info[jj].link[1].id=id2;

}

/*
void CapBody::createMotor(JointType jj,BodyType id1,BodyType id2,
                 dReal ax1,dReal ay1,dReal az1,
                 dReal ax3,dReal ay3,dReal az3,
                 dReal lo1,dReal hi1,
                 dReal lo2,dReal hi2,
                 dReal lo3,dReal hi3)
{
  dJointID kk = dJointCreateAMotor(world,0);
  dJointAttach(kk,bodies[id1],bodies[id2]);
  dJointSetAMotorMode( kk, dAMotorEuler);

  //1,0,0
  //0,1,0
  dJointSetAMotorAxis(kk,0,1,ax1,ay1,az1);
  dJointSetAMotorAxis(kk,2,2,ax3,ay3,az3);

  // We need to stay away from (-)pi/2
  // But this limit apparently only applies
  // to axis 1...

  dJointSetAMotorParam(kk,dParamLoStop1,lo1);
  dJointSetAMotorParam(kk,dParamHiStop1,hi1);
  dJointSetAMotorParam(kk,dParamLoStop2,lo2);
  dJointSetAMotorParam(kk,dParamHiStop2,hi2);
  dJointSetAMotorParam(kk,dParamLoStop3,lo3);
  dJointSetAMotorParam(kk,dParamHiStop3,hi3);


  dJointSetAMotorParam(kk,dParamFMax1,250);
  dJointSetAMotorParam(kk,dParamFMax2,250);
  dJointSetAMotorParam(kk,dParamFMax3,250);

  joints[jj]=kk;
}*/

/**
  All axes are relative
  h1 is constrained to be orthogonal to h2
  r1 must be orthogonal to h1
  When r1 is parallel to h2, angle2 = 0
*/
void CapBody::createUni(JointType jj,BodyType id1,BodyType id2,
                  dReal px1,dReal py1,dReal pz1,
                  dReal px2,dReal py2,dReal pz2,
                  dReal hx1,dReal hy1,dReal hz1,
                  dReal hx2,dReal hy2,dReal hz2,
                  dReal lo1,dReal hi1,
                  dReal lo2,dReal hi2)
{
  joints[jj] = dJointCreateUniversal(world,0);
  dJointAttach(joints[jj],body_segments[id1],body_segments[id2]);
  dJointSetUniversalParam(joints[jj],dParamCFM,INTERNALCFM);

  // ***** Re-orient second body to match target
  // Re-position second body
  // Set the hinges and anchor
  dVector3 dim1;
  dVector3 dim2;

  bodyDims(id1,dim1);
  bodyDims(id2,dim2);

  dVector3 pp1;
  dVector3 pp2;
  const dReal* pos2 = dBodyGetPosition(body_segments[id2]);

  dBodyGetRelPointPos(body_segments[id1],px1*dim1[0]/2,py1*dim1[1]/2,pz1*dim1[2]/2,pp1);
  dBodyGetRelPointPos(body_segments[id2],px2*dim2[0]/2,py2*dim2[1]/2,pz2*dim2[2]/2,pp2);

  dBodySetPosition(body_segments[id2],
    pp1[0]-pp2[0]+pos2[0],
    pp1[1]-pp2[1]+pos2[1],
    pp1[2]-pp2[2]+pos2[2]);

  dJointSetUniversalAnchor(joints[jj],pp1[0],pp1[1],pp1[2]);
  dJointSetUniversalAxes(joints[jj],hx1,hy1,hz1,hx2,hy2,hz2);

  dJointSetUniversalParam(joints[jj],dParamLoStop1,lo1);
  dJointSetUniversalParam(joints[jj],dParamHiStop1,hi1);
  //dJointSetUniversalParam(joints[jj],dParamFMax1,250);
  dJointSetUniversalParam(joints[jj],dParamLoStop2,lo2);
  dJointSetUniversalParam(joints[jj],dParamHiStop2,hi2);
  //dJointSetUniversalParam(joints[jj],dParamFMax2,250);

  // Create a separate motor so that we can
  // monitor the forces used exclusive of the
  // constraints.
  motors[jj] = dJointCreateAMotor(world,0);
  dJointAttach(motors[jj],body_segments[id1],body_segments[id2]);
  dJointSetAMotorMode(motors[jj], dAMotorUser);
  dJointSetAMotorNumAxes(motors[jj],2);
  dJointSetAMotorAxis(motors[jj],0,1,hx1,hy1,hz1);
  dJointSetAMotorAxis(motors[jj],1,2,hx2,hy2,hz2);

  // Stops are left in the joint
  // This has the advantage of actually allowing us
  // to push against a stop without using
  // the fudge param.
  //dJointSetAMotorParam(motors[jj],dParamLoStop1,lo1);
  //dJointSetAMotorParam(motors[jj],dParamHiStop1,hi1);
  //dJointSetAMotorParam(motors[jj],dParamLoStop2,lo2);
  //dJointSetAMotorParam(motors[jj],dParamHiStop2,hi2);
  dJointSetAMotorParam(motors[jj],dParamCFM1,1e-10);
  dJointSetAMotorParam(motors[jj],dParamCFM2,1e-10);
  dJointSetAMotorParam(motors[jj],dParamFMax1,FMAX);
  dJointSetAMotorParam(motors[jj],dParamFMax2,FMAX);
  
  joint_info[jj].link[0].id=id1;
  joint_info[jj].link[1].id=id2;
}

void CapBody::createHinge(JointType jj,BodyType id1,BodyType id2,
                 dReal px1,dReal py1,dReal pz1,
                 dReal px2,dReal py2,dReal pz2,
                 dReal hx1,dReal hy1,dReal hz1,
                 dReal lo1,dReal hi1)
{
  joints[jj] = dJointCreateHinge(world,0);
  dJointAttach(joints[jj],body_segments[id1],body_segments[id2]);
  dJointSetHingeParam(joints[jj],dParamCFM,INTERNALCFM);

  dVector3 dim1;
  dVector3 dim2;

  bodyDims(id1,dim1);
  bodyDims(id2,dim2);

  dVector3 pp1;
  dVector3 pp2;
  const dReal* pos2 = dBodyGetPosition(body_segments[id2]);

  dBodyGetRelPointPos(body_segments[id1],px1*dim1[0]/2,py1*dim1[1]/2,pz1*dim1[2]/2,pp1);
  dBodyGetRelPointPos(body_segments[id2],px2*dim2[0]/2,py2*dim2[1]/2,pz2*dim2[2]/2,pp2);

  dBodySetPosition(body_segments[id2],
    pp1[0]-pp2[0]+pos2[0],
    pp1[1]-pp2[1]+pos2[1],
    pp1[2]-pp2[2]+pos2[2]);

  dJointSetHingeAnchor(joints[jj],pp1[0],pp1[1],pp1[2]);
  dJointSetHingeAxis(joints[jj],hx1,hy1,hz1);
  dJointSetHingeParam(joints[jj],dParamLoStop1,lo1);
  dJointSetHingeParam(joints[jj],dParamHiStop1,hi1);
  //dJointSetHingeParam(joints[jj],dParamFMax1,250);

  motors[jj] = dJointCreateAMotor(world,0);
  dJointAttach(motors[jj],body_segments[id1],body_segments[id2]);
  dJointSetAMotorMode(motors[jj], dAMotorUser);
  dJointSetAMotorNumAxes(motors[jj],1);
  // The motor is set with respect to the
  // first body, but that's what hinge-motor does too.
  // We just hope that there's not too much error.
  dJointSetAMotorAxis(motors[jj],0,1,hx1,hy1,hz1);
  dJointSetAMotorParam(motors[jj],dParamFMax1,FMAX);
  dJointSetAMotorParam(motors[jj],dParamCFM1,1e-10);

  joint_info[jj].link[0].id=id1;
  joint_info[jj].link[1].id=id2;
}


void CapBody::bodyDims(BodyType id,dVector3 dims)
{
  dGeomID gg = dBodyGetFirstGeom( body_segments[id] );
  int gType = dGeomGetClass(gg);
  switch(gType) {
    case dSphereClass: 
		{
			dReal diam = 2*dGeomSphereGetRadius( gg );
      dims[0]=diam;
      dims[1]=diam;
      dims[2]=diam;
    } break;
    case dCapsuleClass:
    {
			dReal radius, length;
			dGeomCapsuleGetParams( gg ,&radius,&length );
      dims[0]=2*radius;
      dims[1]=2*radius;
      dims[2]=2*radius+length;
    } break;
		case dBoxClass:
		{
  		dGeomBoxGetLengths( gg,dims );
		} break;
    default:
    {
      dims[0]=0;
      dims[1]=0;
      dims[2]=0;
    } break;
  }
}



void CapBody::loadMarkToBodyMap(QString filename)
{
  int markCnt = (int)marker_to_body.size();
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
  QTextStream in(&file);
  for (int ii=0;((!in.atEnd())&&(ii<markCnt));++ii) {
    in.skipWhiteSpace();
    in >> marker_to_body[ii].id;
    in.skipWhiteSpace();
  }
}

void CapBody::loadMarkRelPosMap(QString filename)
{
  int markCnt = (int)marker_to_body.size();
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
  QTextStream in(&file);
  for (int ii=0;((!in.atEnd())&&(ii<markCnt));++ii) {
    for (int jj=0;jj<3;++jj) {
      in.skipWhiteSpace();
      in >> marker_to_body[ii].position[jj];
      in.skipWhiteSpace();
    }
  }
}

void CapBody::loadBodyProperties(QString filename)
{
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
  QTextStream in(&file);
  dVector3 dim;
  for (int ii=0;((!in.atEnd())&&(ii<BODY_COUNT));++ii) {
    in.skipWhiteSpace(); in >> part_info[ii].classtype;
    in.skipWhiteSpace(); in >> dim[0];
    in.skipWhiteSpace(); in >> dim[1];
    in.skipWhiteSpace(); in >> dim[2];
    in.skipWhiteSpace(); in >> part_info[ii].qq[0];
    in.skipWhiteSpace(); in >> part_info[ii].qq[1];
    in.skipWhiteSpace(); in >> part_info[ii].qq[2];
    in.skipWhiteSpace(); in >> part_info[ii].qq[3];
    in.skipWhiteSpace();

    setBodyDim(ii,0,dim[0]);
    setBodyDim(ii,1,dim[1]);
    setBodyDim(ii,2,dim[2]);

  }
}

void CapBody::loadJointAnchors(QString filename)
{
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
  QTextStream in(&file);
  int body;
  BodyLink link;

  for (int ii=0;ii<BODY_COUNT;++ii) {
    part_info[ii].links.clear();
  }

  for (int ii=0;!in.atEnd();++ii) {
    in.skipWhiteSpace(); in >> body;
    in.skipWhiteSpace(); in >> link.id;
    in.skipWhiteSpace(); in >> link.position[0];
    in.skipWhiteSpace(); in >> link.position[1];
    in.skipWhiteSpace(); in >> link.position[2];
    in.skipWhiteSpace();
    part_info[body].links.push_back(link);
  }
}

void CapBody::saveMarkToBodyMap(QString filename)
{
  QFile file(filename);
  if (!file.open(QFile::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
  QTextStream out(&file);
  int markCnt = (int) marker_to_body.size();
  for (int ii=0;ii<markCnt;++ii) {
    out << marker_to_body[ii].id << "\n";
  }
}

void CapBody::saveMarkRelPosMap(QString filename,bool bodyID)
{
  QFile file(filename);
  if (!file.open(QFile::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
  QTextStream out(&file);
  int markCnt = (int) marker_to_body.size();
  for (int ii=0;ii<markCnt;++ii) {
    if (bodyID) {
      out << marker_to_body[ii].id << " ";
    }
    out << marker_to_body[ii].position[0] << " "
        << marker_to_body[ii].position[1] << " "
        << marker_to_body[ii].position[2] << "\n";
  }
}

void CapBody::saveBodyProperties(QString filename)
{
  QFile file(filename);
  if (!file.open(QFile::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
  QTextStream out(&file);
  for (int ii=0;ii<BODY_COUNT;++ii) {
    out << part_info[ii].classtype << " "
        << part_info[ii].dimensions[0] << " "
        << part_info[ii].dimensions[1] << " "
        << part_info[ii].dimensions[2] << " "
        << part_info[ii].qq[0] << " "
        << part_info[ii].qq[1] << " "
        << part_info[ii].qq[2] << " "
        << part_info[ii].qq[3] << "\n";
  }
}

void CapBody::saveFullBodyProperties(QString filename)
{
  QFile file(filename);
  if (!file.open(QFile::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
  QTextStream out(&file);
  for (int ii=0;ii<BODY_COUNT;++ii) {

    dVector3 dim;
    int gType = dGeomGetClass(dBodyGetFirstGeom(body_segments[ii]));
    bodyDims(BodyType(ii),dim);
    const dReal* pos = dBodyGetPosition(body_segments[ii]);
    const dReal* qq = dBodyGetQuaternion(body_segments[ii]);
    const dReal* lv = dBodyGetLinearVel(body_segments[ii]);
    const dReal* av = dBodyGetAngularVel(body_segments[ii]);

    out << gType << " "
        << dim[0] << " " << dim[1] << " " << dim[2] << " "
        << CUP(pos[0]) << " " << CUP(pos[1]) << " " << CUP(pos[2]) << " "
        << CUP(qq[0]) << " " << CUP(qq[1]) << " "
        << CUP(qq[2]) << " " << CUP(qq[3]) << " "
        << CUP(lv[0]) << " " << CUP(lv[1]) << " " << CUP(lv[2]) << " "
        << CUP(av[0]) << " " << CUP(av[1]) << " " << CUP(av[2]) << "\n";
  }
}

void CapBody::saveJointProperties(QString filename)
{
  QFile file(filename);
  if (!file.open(QFile::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
  QTextStream out(&file);
  for (int ii=0;ii<JOINT_COUNT;++ii) {
    int jType = dJointGetType(joints[ii]);
    int bb[2];
    bb[0] = joint_info[ii].link[0].id;
    bb[1] = joint_info[ii].link[1].id;
    dVector3 lo={0};
    dVector3 hi={0};
    dVector3 fmax={0};
    dVector3 anchor={0};
    dVector3 axes[3]={0};

    switch(jType) {
      case dJointTypeHinge:
        lo[0] = dJointGetHingeParam(joints[ii],dParamLoStop1);
        hi[0] = dJointGetHingeParam(joints[ii],dParamHiStop1);
        fmax[0] = dJointGetAMotorParam(motors[ii],dParamFMax1);
        dJointGetHingeAnchor(joints[ii],anchor);
        dJointGetHingeAxis(joints[ii],axes[0]);
      break;
      case dJointTypeUniversal:
        lo[0] = dJointGetUniversalParam(joints[ii],dParamLoStop1);
        hi[0] = dJointGetUniversalParam(joints[ii],dParamHiStop1);
        lo[1] = dJointGetUniversalParam(joints[ii],dParamLoStop2);
        hi[1] = dJointGetUniversalParam(joints[ii],dParamHiStop2);
        fmax[0] = dJointGetAMotorParam(motors[ii],dParamFMax1);
        fmax[1] = dJointGetAMotorParam(motors[ii],dParamFMax2);
        dJointGetUniversalAnchor(joints[ii],anchor);
        dJointGetUniversalAxis1(joints[ii],axes[0]);
        dJointGetUniversalAxis2(joints[ii],axes[1]);
      break;
      case dJointTypeBall:
        lo[0] = dJointGetAMotorParam(limits[ii],dParamLoStop1);
        hi[0] = dJointGetAMotorParam(limits[ii],dParamHiStop1);
        lo[1] = dJointGetAMotorParam(limits[ii],dParamLoStop2);
        hi[1] = dJointGetAMotorParam(limits[ii],dParamHiStop2);
        lo[2] = dJointGetAMotorParam(limits[ii],dParamLoStop3);
        hi[2] = dJointGetAMotorParam(limits[ii],dParamHiStop3);
        fmax[0] = dJointGetAMotorParam(motors[ii],dParamFMax1);
        fmax[1] = dJointGetAMotorParam(motors[ii],dParamFMax2);
        fmax[2] = dJointGetAMotorParam(motors[ii],dParamFMax3);
        dJointGetBallAnchor(joints[ii],anchor);
        dJointGetAMotorAxis(motors[ii],0,axes[0]);
        dJointGetAMotorAxis(motors[ii],1,axes[1]);
        dJointGetAMotorAxis(motors[ii],2,axes[2]);
      break;
      case dJointTypeLMotor:
        fmax[0] = dJointGetLMotorParam(motors[ii],dParamFMax1);
        fmax[1] = dJointGetLMotorParam(motors[ii],dParamFMax2);
        fmax[2] = dJointGetLMotorParam(motors[ii],dParamFMax3);
        dJointGetLMotorAxis(joints[ii],0,axes[0]);
        dJointGetLMotorAxis(joints[ii],1,axes[1]);
        dJointGetLMotorAxis(joints[ii],2,axes[2]);
      break;
      case dJointTypeAMotor:
        lo[0] = dJointGetAMotorParam(joints[ii],dParamLoStop1);
        hi[0] = dJointGetAMotorParam(joints[ii],dParamHiStop1);
        fmax[0] = dJointGetAMotorParam(joints[ii],dParamFMax1);
        lo[1] = dJointGetAMotorParam(joints[ii],dParamLoStop2);
        hi[1] = dJointGetAMotorParam(joints[ii],dParamHiStop2);
        fmax[1] = dJointGetAMotorParam(joints[ii],dParamFMax2);
        lo[2] = dJointGetAMotorParam(joints[ii],dParamLoStop3);
        hi[2] = dJointGetAMotorParam(joints[ii],dParamHiStop3);
        fmax[2] = dJointGetAMotorParam(joints[ii],dParamFMax3);
        dJointGetAMotorAxis(joints[ii],0,axes[0]);
        dJointGetAMotorAxis(joints[ii],1,axes[1]);
        dJointGetAMotorAxis(joints[ii],2,axes[2]);
      break;
      default:
      break;
    }
    out << jType << " "
        << bb[0] << " " << bb[1] << " "
        << (lo[0]) << " " << (lo[1]) << " " << (lo[2]) << " "
        << (hi[0]) << " " << (hi[1]) << " " << (hi[2]) << " "
        << (fmax[0]) << " " << (fmax[1]) << " " << (fmax[2]) << " "
        << CUP(anchor[0]) << " " << CUP(anchor[1]) << " " << CUP(anchor[2]) << " "
        << CUP(axes[0][0]) << " " << CUP(axes[0][1]) << " " << CUP(axes[0][2]) << " "
        << CUP(axes[1][0]) << " " << CUP(axes[1][1]) << " " << CUP(axes[1][2]) << " "
        << CUP(axes[2][0]) << " " << CUP(axes[2][1]) << " " << CUP(axes[2][2]) << "\n";
  }
}

void CapBody::saveJointAnchors(QString filename)
{
  QFile file(filename);
  if (!file.open(QFile::WriteOnly | QIODevice::Truncate | QIODevice::Text)) return;
  QTextStream out(&file);
  for (int ii=0;ii<BODY_COUNT;++ii) {
    for (QList<BodyLink>::iterator lit=part_info[ii].links.begin();
         lit!=part_info[ii].links.end();++lit)
    {
      out << ii << " "
          << lit->id << " "
          << lit->position[0] << " "
          << lit->position[1] << " "
          << lit->position[2] << "\n";
    }

  }
}

double CapBody::jointValue(int jj,int ax)
{
  double val=0;
  switch (dJointGetType(joints[jj])) {
    case dJointTypeHinge:
      if (ax==0) val = dJointGetHingeAngle(joints[jj]);
    break;
    case dJointTypeUniversal:
      if (ax==0) val = dJointGetUniversalAngle1(joints[jj]);
      if (ax==1) val = dJointGetUniversalAngle2(joints[jj]);
    break;
    case dJointTypeBall:
      val = dJointGetAMotorAngle(motors[jj],ax);
    break;
    case dJointTypeLMotor: {
      dBodyID bb = dJointGetBody(joints[jj],0);
      const dReal* pos = dBodyGetPosition(bb);
      val = pos[ax];
    } break;
    case dJointTypeAMotor: {
      //dBodyID bb = dJointGetBody(joints[jj],0);
      //const dReal* rot = dBodyGetRotation(bb);
      //dReal yaw = atan2(rot[1],rot[5]);
      //dReal pitch = asin(rot[9]);
      //dReal roll = atan2(rot[8],rot[10]);
      //if (ax==0) val = asin(rot[9]);
      //if (ax==1) val = atan2(rot[10],rot[8]);
      //if (ax==2) val = -atan2(rot[1],rot[5]);
      val = dJointGetAMotorAngle(motors[jj],ax);
    } break;
    default:
    break;

  }
  return val;
}



double CapBody::jointMax(int jj,int ax)
{
  double val=0;
  int param=dParamHiStop1;
  if (ax==1) param = dParamHiStop2;
  if (ax==2) param = dParamHiStop3;

  switch (dJointGetType(joints[jj])) {
    case dJointTypeHinge:
      if (ax==0) {
        val = dJointGetHingeParam(joints[jj],param);
      }
    break;
    case dJointTypeUniversal:
      if (ax==0 || ax==1) {
        val = dJointGetUniversalParam(joints[jj],param);
      }
    break;
    case dJointTypeBall:
      val = dJointGetAMotorParam(limits[jj],param);

    break;
    case dJointTypeLMotor:
      val = 5;
    break;
    case dJointTypeAMotor:
      val = dJointGetAMotorParam(joints[jj],param);
    break;
    default:
    break;

  }
  return val;
}

double CapBody::jointMin(int jj,int ax)
{
  double val=0;
  int param=dParamLoStop1;
  if (ax==1) param = dParamLoStop2;
  if (ax==2) param = dParamLoStop3;

  switch (dJointGetType(joints[jj])) {
    case dJointTypeHinge:
      if (ax==0) {
        val = dJointGetHingeParam(joints[jj],param);
      }
    break;
    case dJointTypeUniversal:
      if (ax==0 || ax==1) {
        val = dJointGetUniversalParam(joints[jj],param);
      }
    break;
    case dJointTypeBall:
      val = dJointGetAMotorParam(limits[jj],param);

    break;
    case dJointTypeLMotor:
      val = -5;
    break;
    case dJointTypeAMotor:
      val = dJointGetAMotorParam(joints[jj],param);
    break;
    default:
    break;

  }
  return val;
}

/**
  Body bb is connected to another body through
  joint jj.  Re-attach that connection
  at pos.
  */
void CapBody::setLink(int bb,int jj,dVector3 pos)
{
  //We need to know if we're the first body or the
  // second body in the joint.
  dVector3 dim;
  bodyDims((BodyType)bb,dim);

  dReal px = pos[0]*dim[0]/2;
  dReal py = pos[1]*dim[1]/2;
  dReal pz = pos[2]*dim[2]/2;



  // Annoyingly, we also need to know what type
  // of joint we are so that we can call the right
  // anchor function that will then do exactly the
  // same thing as the other anchor function setters.
  if (dJointGetBody(joints[jj],0) == body_segments[bb] ) {
    switch (dJointGetType(joints[jj])) {
      case dJointTypeHinge:
        dJointSetHingeAnchor1Rel(joints[jj],px,py,pz);
      break;
      case dJointTypeUniversal:
        dJointSetUniversalAnchor1Rel(joints[jj],px,py,pz);
      break;
      case dJointTypeBall:
        dJointSetBallAnchor1Rel(joints[jj],px,py,pz);
      break;
      default:
      break;
    }

  } else if (dJointGetBody(joints[jj],1) == body_segments[bb]) {
    switch (dJointGetType(joints[jj])) {
      case dJointTypeHinge:
        dJointSetHingeAnchor2Rel(joints[jj],px,py,pz);
      break;
      case dJointTypeUniversal:
        dJointSetUniversalAnchor2Rel(joints[jj],px,py,pz);
      break;
      case dJointTypeBall:
        dJointSetBallAnchor2Rel(joints[jj],px,py,pz);
      break;
      default:
      break;
    }

  } else {
    // Uh oh.
  }
}

void CapBody::setBodySpin(int bb,int ax,DSBoxWithParams* spin)
{
  part_info[bb].spin_box[ax]=spin;
  spin->setValue(part_info[bb].dimensions[ax]);
  connect(spin,SIGNAL(valueChanged(int,int,double)),this,SLOT(setBodyDim(int,int,double)));
}

void CapBody::setBody(int ii)
{

  dGeomID gg = dBodyGetFirstGeom(body_segments[ii]);

  int gType = dGeomGetClass(gg);
  dMass mm;

  switch(gType) {
    case dSphereClass:
    {
      dReal radius = part_info[ii].dimensions[0]/2;
      dGeomSphereSetRadius(gg,radius);
      dMassSetSphere(&mm,density,radius);
      dBodySetMass(body_segments[ii],&mm);
    } break;
    case dCapsuleClass:
    {
      dReal radius = part_info[ii].dimensions[0]/2;
      dReal length = part_info[ii].dimensions[2]-part_info[ii].dimensions[0];
      dGeomCapsuleSetParams( gg ,radius,length );
      dMassSetCapsule(&mm,density,3,radius,length);
      dBodySetMass(body_segments[ii],&mm);

    } break;
    case dBoxClass:
    {
      dGeomBoxSetLengths( gg,
                         part_info[ii].dimensions[0],
                         part_info[ii].dimensions[1],
                         part_info[ii].dimensions[2] );
      dMassSetBox(&mm,density,
                  part_info[ii].dimensions[0],
                  part_info[ii].dimensions[1],
                  part_info[ii].dimensions[2] );
      dBodySetMass(body_segments[ii],&mm);
    } break;
    default:
    {

    } break;
  }
  // ***** Re-set joint anchors

  if (keep_rel_anchors) {
    for (QList<BodyLink>::iterator lit=part_info[ii].links.begin();
         lit!=part_info[ii].links.end();++lit)
    {
      setLink(ii,lit->id,lit->position);
    }
  } else {
    // ***** find the new relative values
  }

}

/**
  Capture body data from the simulation.
  */
void CapBody::getBody(int ii)
{
  bodyDims((BodyType)ii,part_info[ii].dimensions);
  part_info[ii].classtype = dGeomGetClass(dBodyGetFirstGeom(body_segments[ii]));

  // ***** We don't really have an easy way to
  // grab the default orientation of the body.
  // We grab the anchors separately

}


void CapBody::findMassProperties()
{
  dMass m;
  dMass tmp;

  const dReal* rot;
  const dReal* pos;
  const dReal* lin;
  const dReal* ang;

  dVector3 endPos;
  dVector3 vSum;
  dVector3 aSum;


  // Find the aggregate inertial tensor
  // And the weighted sum of positions
  dMassSetZero(&m);
  for (int ii=0;ii<BODY_COUNT;++ii) {
    dBodyGetMass(body_segments[ii],&tmp);
    rot = dBodyGetRotation(body_segments[ii]);
    pos = dBodyGetPosition(body_segments[ii]);
    dMassRotate(&tmp,rot);
    dMassTranslate(&tmp,pos[0],pos[1],pos[2]);
    dMassAdd(&m,&tmp);
  }

  //Translate CoM to local origin
  dOPE(endPos,=,m.c);
  dMassTranslate(&m,-m.c[0],-m.c[1],-m.c[2]);

  //Find the aggregate linear and angular velocities
  dOPEC(vSum,=,0);
  dOPEC(aSum,=,0);
  dVector3 mmt;
  dVector3 amt;
  dVector3 relPos;
  for (int ii=0;ii<BODY_COUNT;++ii) {
    dBodyGetMass(body_segments[ii],&tmp);
    lin = dBodyGetLinearVel(body_segments[ii]);
    ang = dBodyGetAngularVel(body_segments[ii]);
    pos = dBodyGetPosition(body_segments[ii]);

    // Lin Momentum of body
    dOPC(mmt,*,lin,tmp.mass);

    // Weighted sum linear velocity (joint linear momentum)
    dOPE(vSum,+=,mmt);

    // Angular momentum of body
    dMULTIPLY0_331(amt,tmp.I,ang);

    // We take the cross-product of the
    // linear velocity and the vector from the
    // center of mass
    //relPos = pos - endPos
    dOP(relPos,-,pos,endPos);

    // Joint angular momentum
    dOPE(aSum,+=,amt);
    dCROSS(aSum,+=,relPos,mmt);
  }

  dVector3 vFinal;
  dVector3 aFinal;

  dOPC(vFinal,/,vSum,m.mass);
  dMatrix3 mInv;
  if (dInvertPDMatrix(m.I,mInv,3)==0) {
    // Something went very wrong
    // and the inertia tensor is
    // no longer positive definite.
  } else {
    dMULTIPLY0_331(aFinal,mInv,aSum);
  }

  // We now have
  // endPos = center of mass
  // aFinal = joint angular velocity
  // vFinal = joint linear velocity
  // m = mass and inertial tensor

  // If we can do an eigendecomposition on
  // the inertia tensor, then we can find the
  // principal axes of the system.  That would
  // be useful for deciding if you're upright

  dOPE(center_of_mass,=,endPos);
  dOPE(senter_lin_vel,=,vFinal);
  dOPE(center_ang_vel,=,aFinal);

}

void CapBody::saveState()
{
  for (int ii=0;ii<BODY_COUNT;++ii) {
    dBodyCopyPosition(body_segments[ii],saved[ii].position);
    dBodyCopyQuaternion(body_segments[ii],saved[ii].orientation_quaternion);
    dBodyCopyLinearVel(body_segments[ii],saved[ii].linear_velocity);
    dBodyCopyAngularVel(body_segments[ii],saved[ii].angular_velocity);
  }
}

void CapBody::restoreState()
{
  dReal* src;
  for (int ii=0;ii<BODY_COUNT;++ii) {
    src = saved[ii].position;
    dBodySetPosition(body_segments[ii],src[0],src[1],src[2]);
    src = saved[ii].orientation_quaternion;
    dBodySetQuaternion(body_segments[ii],src);
    src = saved[ii].linear_velocity;
    dBodySetLinearVel(body_segments[ii],src[0],src[1],src[2]);
    src = saved[ii].angular_velocity;
    dBodySetAngularVel(body_segments[ii],src[0],src[1],src[2]);
  }
}

/**
 * @brief CapBody::loadBody Grab model info from file and update the simulation
 */
void CapBody::loadBody()
{
  loadMarkToBodyMap("data/markMap.txt");
  loadMarkRelPosMap("data/posMap.txt");
  loadBodyProperties("data/dimMap.txt");
  loadJointAnchors("data/anchorMap.txt");
  copyToSim();
}

/**
 * @brief CapBody::saveBody Grab the current model state and write it to file
 *
 * This saves body dimensions, marker attachments, and joint anchors
 * for loading again in the future.
 */
void CapBody::saveBody()
{
  copyFromSim();
  saveMarkToBodyMap("data/markMap.txt");
  saveMarkRelPosMap("data/posMap.txt");
  saveBodyProperties("data/dimMap.txt");
  saveJointAnchors("data/anchorMap.txt");
}

/**
  Save details about the current state
  of the model.
  */
void CapBody::saveModel()
{

  saveMarkRelPosMap("modelMap.txt",true);
  saveFullBodyProperties("modelBodies.txt");
  saveJointProperties("modelJoints.txt");
}


/**
  For every body part, resize the thing
  and then re-position the anchors.
  */
void CapBody::copyToSim()
{
  for (int ii=0;ii<BODY_COUNT;++ii) {
    setBody(ii);
  }
  int markCnt =(int) marker_to_body.size();

  for (int ii=0;ii<markCnt;++ii) {
    emit markMap(ii,marker_to_body[ii].id);
    emit markPoint(ii,
                   marker_to_body[ii].position[0],
                   marker_to_body[ii].position[1],
                   marker_to_body[ii].position[2]);

  }
}

void CapBody::copyFromSim()
{
  for (int ii=0;ii<BODY_COUNT;++ii) {
    getBody(ii);
  }

}

/**
  All of the control is now
  handled with AMotors (and one
  LMotor) but we still need to
  be sure to set the right stuff?
  */
void CapBody::updateControl()
{
  dReal gain = 1/(stepsize*4);
  //dReal gain = 1/.05;



  // Get the angle from each DoF
  // Set the target velocity based on the
  // distance from target.

  dReal currentVal;
  dReal targetVel;
  for (int ii=0;ii<JOINT_COUNT;++ii) {
    // Zero-out the feedback structures (they're not
    // updated if the constraint isn't active).
    memset(&(feedback[ii]),0,sizeof(dJointFeedback));
    for (int ax=0;ax<3;++ax) {
      int param;
      // Choose the correct axis.
      switch (ax) {
        case 0:
          param=dParamVel1;
          break;
        case 1:
          param=dParamVel2;
          break;
        case 2:
        default:
          param=dParamVel3;
          break;
      }

      currentVal = jointValue(ii,ax);
      double targetVal = joint_target[ii][ax]+joint_offset[ii][ax];
      double jMax = jointMax(ii,ax)-.001;
      double jMin = jointMin(ii,ax)+.001;
      if (targetVal>jMax) targetVal=jMax;
      if (targetVal<jMin) targetVal=jMin;

      targetVel = (targetVal-currentVal)*gain;

      switch (dJointGetType(motors[ii])) {
        case dJointTypeAMotor:
          dJointSetAMotorParam(motors[ii],param,targetVel);
        break;
        case dJointTypeLMotor:
          dJointSetLMotorParam(motors[ii],param,targetVel);
        break;
        default:
        break;
      }
    }
  }

}

void CapBody::changeMarkLinks(int numMarkers)
{
  if (numMarkers<0) numMarkers=0;
  marker_to_body.resize(numMarkers);
}

void CapBody::setJointTarget(int jj,int ax, double val)
{
  if (jj<0) jj=0;
  if (jj>=JOINT_COUNT) jj = JOINT_COUNT-1;
  if (ax<0) ax=0;
  if (ax>2) ax=2;
  double jMax = jointMax(jj,ax);
  double jMin = jointMin(jj,ax);
  if (val>jMax) val=jMax;
  if (val<jMin) val=jMin;
  joint_target[jj][ax]=val;
}

void CapBody::setJointOffset(int jj,int ax, double val)
{
  if (jj<0) jj=0;
  if (jj>=JOINT_COUNT) jj = JOINT_COUNT-1;
  if (ax<0) ax=0;
  if (ax>2) ax=2;
  joint_offset[jj][ax]=val;
}

void CapBody::setJointForce(int jj,int ax,double val)
{
  int param;
  // Choose the correct axis.
  switch (ax) {
    case 0:
      param=dParamFMax1;
      break;
    case 1:
      param=dParamFMax2;
      break;
    case 2:
    default:
      param=dParamFMax3;
      break;
  }

  // Set the appropriate value
  switch(dJointGetType(motors[jj])) {
    case dJointTypeAMotor:
      dJointSetAMotorParam(motors[jj],param,val);
    break;
  case dJointTypeLMotor:
    dJointSetLMotorParam(motors[jj],param,val);
  break;
    default:
    break;
  }
  control_limit[jj][ax] = val;

}

/**

  */
double CapBody::forceValue(int jj,int ax)
{
  double val=feedback[jj].gf[ax];;

  if (global_forces) {
    // Use global coordinates
    switch (dJointGetType(joints[jj])) {
    case dJointTypeLMotor:
      val = feedback[jj].f1[ax];
      break;
    default:
      val = feedback[jj].t1[ax];
      break;
    }
  }
  return val;
}

void CapBody::applyJointForce(int jj,int ax,double val)
{
  dVector3 v={0};
  v[ax]=val;

  if (global_forces) {
    //Global forces -- directly apply the equal and opposite torques
    // in the global frame
    dBodyID b1,b2;
    b1 = dJointGetBody(joints[jj],0);
    b2 = dJointGetBody(joints[jj],1);

    switch (dJointGetType(joints[jj])) {
    case dJointTypeLMotor:
      dBodyAddForce(b1,v[0],v[1],v[2]);
      if (b2) dBodyAddForce(b2,-v[0],-v[1],-v[2]);
      break;
    default:
      dBodyAddTorque(b1,v[0],v[1],v[2]);
      if (b2) dBodyAddTorque(b2,-v[0],-v[1],-v[2]);
      break;
    }
  } else {

    // Generalized forces -- add the torques in to the body-relative frame.
    switch (dJointGetType(joints[jj])) {
    case dJointTypeLMotor:
      dJointAddLMotorForces(motors[jj],v[0],v[1],v[2]);
      break;
    default:
      dJointAddAMotorTorques(motors[jj],v[0],v[1],v[2]);
      break;
    }
  }

}

void CapBody::setBodyDim(int bb,int ax,double val)
{
  if (part_info[bb].dimensions[ax]!=val) {
    part_info[bb].dimensions[ax]=val;
    setBody(bb);
    if (part_info[bb].spin_box[ax] && part_info[bb].spin_box[ax]->value()!=val) {
      part_info[bb].spin_box[ax]->setValue(val);
    }
  }
}

void CapBody::bigHand(bool big)
{
  dMass m;
  dReal radius;

  if (big) {
    radius=0.16;
  } else {
    radius = 0.055;
  }
  dGeomSphereSetRadius(geometries[R_HAND_BODY],radius);
  dMassSetSphere(&m,density,radius);
  dBodySetMass(body_segments[R_HAND_BODY],&m);

}



#include <QMessageBox>
#include <QtSql>
#include <QSqlDatabase>
#include <map>

using std::map;

/**
  This little hack is designed to write out
  the old model in the new model format.
  */
void CapBody::writeModel()
{
  // Open and init the database
  QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
  db.setDatabaseName("./tmpData.db");
  if (!db.open()) {
      QMessageBox::critical(0, qApp->tr("Cannot open database"),
          qApp->tr("Unable to establish a database connection.\n"
                   "Click Cancel to exit."), QMessageBox::Cancel);
      return;
  }

  // Create the tables
  QSqlQuery query;
  query.exec("CREATE TABLE body "
             "(pk INTEGER primary key, "
              "label TEXT, "                           // Human label
              "px REAL, py REAL, pz REAL, "            // Position
              "qw REAL, qx REAL, qy REAL, qz REAL, "   // Orienation
              "vx REAL, vy REAL, vz REAL, "            // Linear velocity
              "wx REAL, wy REAL, wz REAL, "            // Angular velocity
              "mass REAL, I11 REAL, I22 REAL, I33 REAL, " // Inertial tensor
                         "I12 REAL, I13 REAL, I23 REAL, "
              "kinematic BOOLEAN, massFromGeom BOOLEAN "// Switches
             ")");
  // Prepare an insertion query
  QSqlQuery bodyQ;
  bodyQ.prepare("INSERT INTO body VALUES "
               "(:pk, :label, "
                ":px, :py, :pz, "
                ":qw, :qx, :qy, :qz, "
                ":vx, :vy, :vz, "
                ":wx, :wy, :wz, "
                ":mass, :I11, :I22, :I33, "
                ":I12, :I13, :I23, "
                ":kinematic, :massFromGeom"
               ")");

  // Setup defaults
  bodyQ.bindValue(0,0);
  bodyQ.bindValue(1,"");
  bodyQ.bindValue(2,0); bodyQ.bindValue(3,0); bodyQ.bindValue(4,0);
  bodyQ.bindValue(5,1); bodyQ.bindValue(6,0); bodyQ.bindValue(7,0); bodyQ.bindValue(8,0);
  bodyQ.bindValue(9,0); bodyQ.bindValue(10,0); bodyQ.bindValue(11,0);
  bodyQ.bindValue(12,0); bodyQ.bindValue(13,0); bodyQ.bindValue(14,0);
  bodyQ.bindValue(15, 800); bodyQ.bindValue(16, 1); bodyQ.bindValue(17, 1); bodyQ.bindValue(18, 1);
  bodyQ.bindValue(19, 0); bodyQ.bindValue(20, 0); bodyQ.bindValue(21, 0);
  bodyQ.bindValue(22, false); bodyQ.bindValue(23, true);

  query.exec("create table geom "
             "(pk INTEGER primary key, "
             "label text, "                       // Human readable label
             "type INTEGER, body INTEGER, "       // Class and body ptr
             "dx REAL, dy REAL, dz REAL, "        // Dimensions
             "offset BOOLEAN, "                   // Uses offsets?
             "px REAL, py REAL, pz REAL, "        // Offset position
             "qw REAL, qx REAL, qy REAL, qz REAL" // Offset orientation
             ")");
  QSqlQuery geomQ;
  geomQ.prepare("INSERT INTO geom VALUES "
                "(:pk, "
                ":label, "                       // Human readable label
                ":type, :body, "       // Class and body ptr
                ":dx, :dy, :dz, "        // Dimensions
                ":offset, "                   // Uses offsets?
                ":px, :py, :pz, "        // Offset position
                ":qw, :qx, :qy, :qz" // Offset orientation
                ")");
  geomQ.bindValue(0,0);
  geomQ.bindValue(1,"");
  geomQ.bindValue(2,dCapsuleClass); geomQ.bindValue(3,-1);
  geomQ.bindValue(4,1); geomQ.bindValue(5,1); geomQ.bindValue(6,1);
  geomQ.bindValue(7,false);
  geomQ.bindValue(8,0); geomQ.bindValue(9,0); geomQ.bindValue(10,0);
  geomQ.bindValue(11,1); geomQ.bindValue(12,0); geomQ.bindValue(13,0); geomQ.bindValue(14,0);

  query.exec("create table joint "
             "(pk INTEGER primary key, "
             "label text, "                             // Human readable label
             "body1 INTEGER, body2 INTEGER,"
             "a1x REAL, a1y REAL, a1z REAL, "           // Anchor point 1
             "a2x REAL, a2y REAL, a2z REAL, "           // Anchor point 2
             "q1w REAL, q1x REAL, q1y REAL, q1z REAL, " // Quaternion limit 1
             "q2w REAL, q2x REAL, q2y REAL, q2z REAL, " // Quaternion limit 2
             "q3w REAL, q3x REAL, q3y REAL, q3z REAL, " // Quaternion limit 3
             "l1cfm REAL,  l1erp REAL, "                // Linear limit param1
             "l1hi REAL,   l1lo REAL, "
             "l1fmax REAL, l1vel REAL, "
             "l2cfm REAL,  l2erp REAL, "                // Linear limit param2
             "l2hi REAL,   l2lo REAL, "
             "l2fmax REAL, l2vel REAL, "
             "l3cfm REAL,  l3erp REAL, "                // Linear limit param3
             "l3hi REAL,   l3lo REAL, "
             "l3fmax REAL, l3vel REAL, "
             "a1cfm REAL,  a1erp REAL, "                // Angular limit param1
             "a1hi REAL,   a1lo REAL, "
             "a1fmax REAL, a1vel REAL, "
             "a2cfm REAL,  a2erp REAL, "                // Angular limit param2
             "a2hi REAL,   a2lo REAL, "
             "a2fmax REAL, a2vel REAL, "
             "a3cfm REAL,  a3erp REAL, "                // Angular limit param3
             "a3hi REAL,   a3lo REAL, "
             "a3fmax REAL, a3vel REAL "
             ")");
  QSqlQuery jointQ;
  jointQ.prepare("INSERT INTO joint VALUES"
                 "(:pk, "
                 ":label, "                             // Human readable label
                 ":body1, :body2, "
                 ":a1x, :a1y, :a1z, "           // Anchor point 1
                 ":a2x, :a2y, :a2z, "           // Anchor point 2
                 ":q1w, :q1x, :q1y, :q1z, " // Quaternion limit 1
                 ":q2w, :q2x, :q2y, :q2z, " // Quaternion limit 2
                 ":q3w, :q3x, :q3y, :q3z, " // Quaternion limit 3
                 ":l1cfm,  :l1erp, "                // Linear limit param1
                 ":l1hi,   :l1lo, "
                 ":l1fmax, :l1vel, "
                 ":l2cfm,  :l2erp, "                // Linear limit param2
                 ":l2hi,   :l2lo, "
                 ":l2fmax, :l2vel, "
                 ":l3cfm,  :l3erp, "                // Linear limit param3
                 ":l3hi,   :l3lo, "
                 ":l3fmax, :l3vel, "
                 ":a1cfm,  :a1erp, "                // Angular limit param1
                 ":a1hi,   :a1lo, "
                 ":a1fmax, :a1vel, "
                 ":a2cfm,  :a2erp, "                // Angular limit param2
                 ":a2hi,   :a2lo, "
                 ":a2fmax, :a2vel, "
                 ":a3cfm,  :a3erp, "                // Angular limit param3
                 ":a3hi,   :a3lo, "
                 ":a3fmax, :a3vel "
                 ")");
  //Joint defaults
  jointQ.bindValue(0,0);
  jointQ.bindValue(1,"");
  jointQ.bindValue(2,-1); jointQ.bindValue(3,-1);
  jointQ.bindValue(4,0); jointQ.bindValue(5,0); jointQ.bindValue(6,0);
  jointQ.bindValue(7,0); jointQ.bindValue(8,0); jointQ.bindValue(9,0);
  jointQ.bindValue(10,1); jointQ.bindValue(11,0); jointQ.bindValue(12,0); jointQ.bindValue(13,0);
  jointQ.bindValue(14,1); jointQ.bindValue(15,0); jointQ.bindValue(16,0); jointQ.bindValue(17,0);
  jointQ.bindValue(18,1); jointQ.bindValue(19,0); jointQ.bindValue(20,0); jointQ.bindValue(21,0);

  jointQ.bindValue(22,1e-5); jointQ.bindValue(23,0.2);
  jointQ.bindValue(24,0); jointQ.bindValue(25,0);
  jointQ.bindValue(26,0); jointQ.bindValue(27,0);
  jointQ.bindValue(28,1e-5); jointQ.bindValue(29,0.2);
  jointQ.bindValue(30,0); jointQ.bindValue(31,0);
  jointQ.bindValue(32,0); jointQ.bindValue(33,0);
  jointQ.bindValue(34,1e-5); jointQ.bindValue(35,0.2);
  jointQ.bindValue(36,0); jointQ.bindValue(37,0);
  jointQ.bindValue(38,0); jointQ.bindValue(39,0);

  jointQ.bindValue(40,1e-5); jointQ.bindValue(41,0.2);
  jointQ.bindValue(42,dInfinity); jointQ.bindValue(43,-dInfinity);
  jointQ.bindValue(44,250); jointQ.bindValue(45,0);
  jointQ.bindValue(46,1e-5); jointQ.bindValue(47,0.2);
  jointQ.bindValue(48,dInfinity); jointQ.bindValue(49,-dInfinity);
  jointQ.bindValue(50,250); jointQ.bindValue(51,0);
  jointQ.bindValue(52,1e-5); jointQ.bindValue(53,0.2);
  jointQ.bindValue(54,dInfinity); jointQ.bindValue(55,-dInfinity);
  jointQ.bindValue(56,250); jointQ.bindValue(57,0);


  map<dBodyID,int> pmap;

  // Write out the bodies and geometries
  for (int ii=0;ii<BODY_COUNT;++ii) {
    pmap[body_segments[ii] ] = ii;
    dVector3 dim;
    int gType = dGeomGetClass(dBodyGetFirstGeom(body_segments[ii]));
    bodyDims(BodyType(ii),dim);
    const dReal* pos = dBodyGetPosition(body_segments[ii]);
    const dReal* qq = dBodyGetQuaternion(body_segments[ii]);
    //const dReal* lv = dBodyGetLinearVel(bodies[ii]);
    //const dReal* av = dBodyGetAngularVel(bodies[ii]);
    bodyQ.bindValue(":pk",ii);
    bodyQ.bindValue(":px",CUP(pos[0]));
    bodyQ.bindValue(":py",CUP(pos[1]));
    bodyQ.bindValue(":pz",CUP(pos[2]));
    bodyQ.bindValue(":qw",CUP(qq[0]));
    bodyQ.bindValue(":qx",CUP(qq[1]));
    bodyQ.bindValue(":qy",CUP(qq[2]));
    bodyQ.bindValue(":qz",CUP(qq[3]));
    bodyQ.exec();

    geomQ.bindValue(":pk",ii);
    geomQ.bindValue(":type",gType);
    geomQ.bindValue(":body",ii);
    geomQ.bindValue(":dx",dim[0]);
    geomQ.bindValue(":dy",dim[1]);
    geomQ.bindValue(":dz",dim[2]);
    geomQ.exec();
  }

  // Write out the joints
  for (int ii=0;ii<JOINT_COUNT;++ii) {

    dBodyID b1 = dJointGetBody(joints[ii],0);
    dBodyID b2 = dJointGetBody(joints[ii],1);
    int body1 = pmap.count(b1)?pmap[b1]:-1;
    int body2 = pmap.count(b2)?pmap[b2]:-1;

    dVector3 anchor;
    dVector3 relAnchor1 = {0},relAnchor2 = {0};



    int jType = dJointGetType(joints[ii]);
    switch(jType) {
      case dJointTypeHinge:

        dJointGetHingeAnchor(joints[ii],anchor);
        dBodyGetPosRelPoint(b1,anchor[0],anchor[1],anchor[2],relAnchor1);
        dBodyGetPosRelPoint(b2,anchor[0],anchor[1],anchor[2],relAnchor2);

      break;
      case dJointTypeUniversal:

        dJointGetUniversalAnchor(joints[ii],anchor);
        dBodyGetPosRelPoint(b1,anchor[0],anchor[1],anchor[2],relAnchor1);
        dBodyGetPosRelPoint(b2,anchor[0],anchor[1],anchor[2],relAnchor2);
      break;
      case dJointTypeBall:
        dJointGetBallAnchor(joints[ii],anchor);
        dBodyGetPosRelPoint(b1,anchor[0],anchor[1],anchor[2],relAnchor1);
        dBodyGetPosRelPoint(b2,anchor[0],anchor[1],anchor[2],relAnchor2);
      break;
      case dJointTypeLMotor:
      case dJointTypeAMotor:
        dSetZero(relAnchor1,4);
        dSetZero(relAnchor2,4);
      break;
      default:
      break;
    }

    jointQ.bindValue(":pk",ii);
    jointQ.bindValue(":body1",body1);
    jointQ.bindValue(":body2",body2);

    jointQ.bindValue(":a1x",CUP(relAnchor1[0]));
    jointQ.bindValue(":a1y",CUP(relAnchor1[1]));
    jointQ.bindValue(":a1z",CUP(relAnchor1[2]));
    jointQ.bindValue(":a2x",CUP(relAnchor2[0]));
    jointQ.bindValue(":a2y",CUP(relAnchor2[1]));
    jointQ.bindValue(":a2z",CUP(relAnchor2[2]));

    jointQ.exec();
  }
}

void CapBody::writeMarkers()
{
  // Open and init the database
  QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
  db.setDatabaseName("./tmpData.db");
  if (!db.open()) {
      QMessageBox::critical(0, qApp->tr("Cannot open database"),
          qApp->tr("Unable to establish a database connection.\n"
                   "Click Cancel to exit."), QMessageBox::Cancel);
      return;
  }

  // Create the tables
  QSqlQuery query;
  query.exec("CREATE TABLE markerMap ("
             "pk INTEGER primary key, "
             "label TEXT, "                         // Human label
             "body INTEGER, "
             "px REAL, py REAL, pz REAL, "            // Position
             "cfm REAL, erp REAL"
             ")");
  QSqlQuery markQ;
  markQ.prepare("INSERT INTO markerMap VALUES ("
                ":pk, "
                ":label, "         // Human label
                ":body, "
                ":px, :py, :pz, "  // Position
                ":cfm, :erp"
                ")");
  markQ.bindValue(":label","");
  markQ.bindValue(":cfm",1e-3);
  markQ.bindValue(":erp",0.2);
  int markCnt = (int)marker_to_body.size();
  for (int ii=0;ii<markCnt;++ii) {
    markQ.bindValue(":pk",ii);
    markQ.bindValue(":body",marker_to_body[ii].id);
    markQ.bindValue(":px",marker_to_body[ii].position[0]);
    markQ.bindValue(":py",marker_to_body[ii].position[1]);
    markQ.bindValue(":pz",marker_to_body[ii].position[2]);
    markQ.exec();
  }
}


void CapBody::saveTempState()
{
  for (int ii=0;ii<BODY_COUNT;++ii) {
    dBodyCopyPosition(body_segments[ii],temp_state[ii].position);
    dBodyCopyQuaternion(body_segments[ii],temp_state[ii].orientation_quaternion);
    dBodyCopyLinearVel(body_segments[ii],temp_state[ii].linear_velocity);
    dBodyCopyAngularVel(body_segments[ii],temp_state[ii].angular_velocity);
  }
}

void CapBody::restoreTempState()
{
  for (int ii=0;ii<BODY_COUNT;++ii) {
    dBodySetPosition(body_segments[ii],
                     temp_state[ii].position[0],
                     temp_state[ii].position[1],
                     temp_state[ii].position[2]);
    dBodySetQuaternion(body_segments[ii],temp_state[ii].orientation_quaternion);
    dBodySetLinearVel(body_segments[ii],
                      temp_state[ii].linear_velocity[0],
                      temp_state[ii].linear_velocity[1],
                      temp_state[ii].linear_velocity[2]);
    dBodySetAngularVel(body_segments[ii],
                       temp_state[ii].angular_velocity[0],
                       temp_state[ii].angular_velocity[1],
                       temp_state[ii].angular_velocity[2]);
  }
}

void CapBody::zeroAccumulators()
{
  for (int ii=0;ii<BODY_COUNT;++ii) {
    dBodySetForce(body_segments[ii],0,0,0);
    dBodySetTorque(body_segments[ii],0,0,0);
  }
}

void CapBody::zeroControl()
{
  for (int jj=0;jj<JOINT_COUNT;++jj) {
    switch (dJointGetType(joints[jj])) {
      case dJointTypeHinge:
        dJointSetAMotorParam(motors[jj],dParamFMax1,0);
      break;
      case dJointTypeUniversal:
        dJointSetAMotorParam(motors[jj],dParamFMax1,0);
        dJointSetAMotorParam(motors[jj],dParamFMax2,0);
      break;
      case dJointTypeBall:
        dJointSetAMotorParam(motors[jj],dParamFMax1,0);
        dJointSetAMotorParam(motors[jj],dParamFMax2,0);
        dJointSetAMotorParam(motors[jj],dParamFMax3,0);
      break;
      case dJointTypeLMotor: {
        dJointSetLMotorParam(motors[jj],dParamFMax1,0);
        dJointSetLMotorParam(motors[jj],dParamFMax2,0);
        dJointSetLMotorParam(motors[jj],dParamFMax3,0);
      } break;
      case dJointTypeAMotor: {
        dJointSetAMotorParam(motors[jj],dParamFMax1,0);
        dJointSetAMotorParam(motors[jj],dParamFMax2,0);
        dJointSetAMotorParam(motors[jj],dParamFMax3,0);
      } break;
      default:
      break;

    }
  }
}

void CapBody::restoreControl()
{
  for (int jj=0;jj<JOINT_COUNT;++jj) {
    switch (dJointGetType(joints[jj])) {
      case dJointTypeHinge:
        dJointSetAMotorParam(motors[jj],dParamFMax1,control_limit[jj][0]);
      break;
      case dJointTypeUniversal:
        dJointSetAMotorParam(motors[jj],dParamFMax1,control_limit[jj][0]);
        dJointSetAMotorParam(motors[jj],dParamFMax2,control_limit[jj][1]);
      break;
      case dJointTypeBall:
        dJointSetAMotorParam(motors[jj],dParamFMax1,control_limit[jj][0]);
        dJointSetAMotorParam(motors[jj],dParamFMax2,control_limit[jj][1]);
        dJointSetAMotorParam(motors[jj],dParamFMax3,control_limit[jj][2]);
      break;
      case dJointTypeLMotor: {
        dJointSetLMotorParam(motors[jj],dParamFMax1,control_limit[jj][0]);
        dJointSetLMotorParam(motors[jj],dParamFMax2,control_limit[jj][1]);
        dJointSetLMotorParam(motors[jj],dParamFMax3,control_limit[jj][2]);
      } break;
      case dJointTypeAMotor: {
        dJointSetAMotorParam(motors[jj],dParamFMax1,control_limit[jj][0]);
        dJointSetAMotorParam(motors[jj],dParamFMax2,control_limit[jj][1]);
        dJointSetAMotorParam(motors[jj],dParamFMax3,control_limit[jj][2]);
      } break;
      default:
      break;

    }
  }
}



