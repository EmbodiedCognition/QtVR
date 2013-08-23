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
#ifndef __CAPBODY_H
#define __CAPBODY_H

#include <QObject>
#include <ode/ode.h>
#include <QList>

class JointFrame;
class DataFrame;
class DSBoxWithParams;

/**
  A marker or another body connects to a body
  segment.  Joint anchors are specified
  relative to the body.
  */
struct BodyLink
{
  int id; ///< Which body part does the marker link to?
  dVector3 pos; ///< Where (in body-relative coords) does the marker connect.

  BodyLink() {id = -1; pos[0]=0;pos[1]=0;pos[2]=0; }
};

/**
  The dynamic state of a body segment.
  */
struct BodyState
{
  dVector3 pos;  ///< Position
  dQuaternion q; ///< Orientation
  dVector3 lvel; ///< Linear velocity
  dVector3 avel; ///< Angular velocity
};

/**
  Data, loaded from file, about a particular body part.
  */
struct PartInfo
{
  int classtype; ///< Sphere, caps, or box
  dVector3 dim;  ///< radius, length, or x,y,z
  DSBoxWithParams* spinBox[3]; ///< Widgets for setting body dimensions
  dQuaternion qq; ///< Orientation in default pose
  QList<BodyLink> links; ///< Links that need anchoring

  PartInfo() {classtype=-1;
              dim[0]=0;dim[1]=0;dim[2]=0;
              spinBox[0]=0;spinBox[1]=0;spinBox[2]=0;
              qq[0]=1;qq[1]=0;qq[2]=0;qq[3]=0; }
};

/**
  Joint info loaded from file includes the anchor
  points for the two linked bodies and the type of
  joint doing the connecting.

  We should also load joint limits and axes.
  */
struct JointInfo
{
  int classtype;
  BodyLink link[2];
};

/**
  An ODE humanoid character model.
  Collision geometry, dynamic bodies, and joints.
  The topology of the model is currently hardcoded.
  The dimensions and joint anchors are loaded from file.
  I was working at storing and loading topology from
  a database, but didn't finish.
  */
class CapBody : public QObject
{
    Q_OBJECT

public:
  enum BodyType {
    HEAD_BODY,    
    NECK_BODY,    
    UP_TORSO_BODY,
    R_COLLAR_BODY,
    RUP_ARM_BODY,
    RLO_ARM_BODY,
    R_HAND_BODY,
    L_COLLAR_BODY,
    LUP_ARM_BODY,
    LLO_ARM_BODY,
    L_HAND_BODY,
    LO_TORSO_BODY,
    WAIST_BODY,
    RUP_LEG_BODY,
    RLO_LEG_BODY,
    R_HEEL_BODY,
    R_TARSAL_BODY,
 //   R_TOE_BODY,
    LUP_LEG_BODY,
    LLO_LEG_BODY,
    L_HEEL_BODY,
    L_TARSAL_BODY,
 //   L_TOE_BODY,
    BODY_COUNT
  };

  enum GeomType {
    HEAD_GEOM,
    NECK_GEOM,
    UP_TORSO_GEOM,
    R_COLLAR_GEOM,
    RUP_ARM_GEOM,
    RLO_ARM_GEOM,
    R_HAND_GEOM,
    L_COLLAR_GEOM,
    LUP_ARM_GEOM,
    LLO_ARM_GEOM,
    L_HAND_GEOM,
    LO_TORSO_GEOM,
    WAIST_GEOM,
    RUP_LEG_GEOM,
    RLO_LEG_GEOM,
    R_HEEL_GEOM,
    R_TARSAL_GEOM,
 //   R_TOE_GEOM,
    LUP_LEG_GEOM,
    LLO_LEG_GEOM,
    L_HEEL_GEOM,
    L_TARSAL_GEOM,
//    L_TOE_GEOM,
    GEOM_COUNT
  };

  enum JointType {
    CHIN_JOINT,
    THROAT_JOINT,
    R_COLLAR_JOINT,
    R_SHOULDER_JOINT,
    R_ELBOW_JOINT,
    R_WRIST_JOINT,
    L_COLLAR_JOINT,
    L_SHOULDER_JOINT,
    L_ELBOW_JOINT,
    L_WRIST_JOINT,
    SPINE_JOINT,
    WAIST_JOINT,
    R_HIP_JOINT,
    R_KNEE_JOINT,
    R_ANKLE_JOINT,
    R_FOOT_JOINT,
 //   R_TOE_JOINT,
    L_HIP_JOINT,
    L_KNEE_JOINT,
    L_ANKLE_JOINT,
    L_FOOT_JOINT,
//    L_TOE_JOINT,
    ROOT_LINMOTOR_JOINT,
    ROOT_ANGMOTOR_JOINT,
    JOINT_COUNT
  };


  /// 'Long-term' storage of model state
  BodyState saved[BODY_COUNT];
  /// 'Short-term' (per frame) storage of model state
  BodyState tempState[BODY_COUNT];

  dBodyID bodies[BODY_COUNT];   ///< Body parts
  dGeomID geoms[GEOM_COUNT];    ///< Collision geometry
  dJointID joints[JOINT_COUNT]; ///< Joints
  dJointID limits[JOINT_COUNT]; ///< Joint limits for 3dof joints
  dJointID motors[JOINT_COUNT]; ///< Joint motors for all joints
  /// Feedback from the motors for getting the inverse dynamics
  dJointFeedback feedback[JOINT_COUNT];

  dWorldID world; ///< The ODE world, passed in from SimWorld
  dSpaceID space; ///< The collision space passed in from SimWorld

  /*
    We assume constant density (approximately that of water)
    for all body parts.  We could do much better by using
    real anthropometric data for each limb.
    */
  static dReal density; ///< Density of body parts


  double jointTarget[JOINT_COUNT][3]; ///< Target angle for each joint
  double jointOffset[JOINT_COUNT][3]; ///< Offset applied to the target (from the GUI)
  double controlLimit[JOINT_COUNT][3]; ///< Max force a joint motor can apply to pursue the target

  BodyLink markerToBody[MARKER_COUNT]; ///< Relationship between markers and bodies.
  PartInfo partInfo[BODY_COUNT];
  JointInfo jointInfo[JOINT_COUNT];
  double stepsize; ///< Delta time for each step of dynamics (passed in from SimWorld)

  bool globalForces; ///< Should we compute/apply forces in body local or global coordinates

  /*
    These data are computed only for display purposes at
    the moment.  However, they could be used as part of
    a control strategy eventually.
    */
  dVector3 com;  ///< Model's center of mass
  dVector3 cLin; ///< Model's aggregate linear velocity
  dVector3 cAng; ///< Model's aggregate angular velocity

  /// When body dimensions are changed, what should be do with the anchors?
  bool keepRelAnchors;

public:

    explicit CapBody(QObject *parent = 0);
    ~CapBody(void);

  void fillJointFrame(JointFrame* jf);
  void fillForceFrame(JointFrame* jf);
  void setTargetFrame(DataFrame* jf);
  void applyForceFrame(DataFrame* jf);

  void setStepSize(double ss) { stepsize = ss; }
  void createBody(dWorldID world,dSpaceID space);

  void createSphere(BodyType id,GeomType g,dReal radius);
  void createCaps(BodyType id,GeomType g,dReal radius,dReal zLen);
  void createBox(BodyType id,GeomType g,dReal xLen,dReal yLen,dReal zLen);

  void createBall(JointType jj,BodyType id1,BodyType id2,
                  dReal px1,dReal py1,dReal pz1,
                  dReal px2,dReal py2,dReal pz2,
                  dReal ax1,dReal ay1,dReal az1,
                  dReal ax3,dReal ay3,dReal az3,
                  dReal lo1,dReal hi1,
                  dReal lo2,dReal hi2,
                  dReal lo3,dReal hi3);

  void createUni(JointType jj,BodyType id1,BodyType id2,
                  dReal px1,dReal py1,dReal pz1,
                  dReal px2,dReal py2,dReal pz2,
                  dReal rx1,dReal ry1,dReal rz1,
                  dReal rx2,dReal ry2,dReal rz2,
                  dReal lo1,dReal hi1,
                  dReal lo2,dReal hi2);
  void createHinge(JointType jj,BodyType id1,BodyType id2,
                   dReal px1,dReal py1,dReal pz1,
                   dReal px2,dReal py2,dReal pz2,
                   dReal hx1,dReal hy1,dReal hz1,
                   dReal lo1,dReal hi1);
  void bodyDims(BodyType id,dVector3 dims);


  void loadMarkToBodyMap(QString filename);
  void loadMarkRelPosMap(QString filename);
  void loadBodyProperties(QString filename);
  void loadJointAnchors(QString filename);

  void saveMarkToBodyMap(QString filename);
  void saveMarkRelPosMap(QString filename,bool bodyID=false);
  void saveBodyProperties(QString filename);
  void saveFullBodyProperties(QString filename);
  void saveJointProperties(QString filename);
  void saveJointAnchors(QString filename);

  double jointValue(int jj,int ax);
  double forceValue(int jj,int ax);
  double jointMax(int jj,int ax);
  double jointMin(int jj,int ax);



  void setLink(int bb,int jj,dVector3 pos);
  void setBodySpin(int bb,int ax,DSBoxWithParams* spin);
  void setBody(int ii);
  void getBody(int ii);

  void findMassProperties();
  void saveTempState();
  void restoreTempState();

  void zeroAccumulators();
  void zeroControl();
  void restoreControl();

signals:
  void markMap(int mark,int bodyID);
  void markPoint(int mark,double px,double py,double pz);
  void targetChange(int jj,int ax,double val);

public slots:
  void loadBody();
  void saveBody();

  void saveState();
  void restoreState();

  void saveModel();

  void copyToSim();
  void copyFromSim();

  void updateControl();
  void setJointTarget(int jj,int ax, double val);
  void setJointOffset(int jj,int ax, double val);
  void setJointForce(int jj,int ax,double val);
  void applyJointForce(int jj,int ax,double val);

  void setBodyDim(int bb,int ax,double val);
  void bigHand(bool big);

  void writeModel();
  void writeMarkers();

protected:



};

#endif
