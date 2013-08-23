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
#ifndef SIMWORLD_H
#define SIMWORLD_H

/// The maximum number of ground contacts we'll
/// track ground forces from.
/// Generally, we only expect three per foot.
#define MAX_GROUND_FEEDBACK 40

#include <QObject>
#include <ode/ode.h>


//#include "swingdata.h"
//#include "livemarkerdata.h"
#if defined( BOARD_DATA )
#include "boardmarkerdata.h"
#elif defined( POKE_DATA )
#include "pokemarkerdata.h"
#else
#include "markerdata.h"
#endif

class Sequence;
class CapBody;
class JointFrame;
class DataFrame;

/**
  A simple object holding a line segment and a color.
  GLine is used for providing graphical feedback about
  ground forces and torques.
  */
struct GLine {
  dVector3 p1;
  dVector3 p2;

  dVector3 col;
};

/**
  A wrapper around the physics engine and associated
  objects including marker, angle, and torque data.
  */
class SimWorld : public QObject
{
    Q_OBJECT
public:
    explicit SimWorld(QObject *parent = 0);
    virtual ~SimWorld();



  //SwingData* getMarkerData() {return markData;}
  //LiveMarkerData* getMarkerData() {return markData;}
#if defined( BOARD_DATA )
  BoardMarkerData* getMarkerData() {return markData;}
#elif defined( POKE_DATA )
  PokeMarkerData* getMarkerData() {return markData;}
#else
  MarkerData* getMarkerData() {return markData;}
#endif
  dSpaceID getSpace() { return space; }
  dSpaceID getMarkSpace() { return markSpace; }
  dSpaceID getTargetSpace() {return targetSpace;}
  CapBody* getBody() { return body; }

  static void collideGeoms(void* data,dGeomID o1,dGeomID o2);

  void updateSeqFrame();
  void applyTorquesToBody(DataFrame *jf);

#if defined( BOARD_DATA )
  void updateBoards();
#endif


  void writeVMarkerFrame(int cc,int tri,FILE* file);
  void writeAngleFrame(int cc,int tri,FILE* file);



signals:

  /// The GUI keys off this signal to connect or
  /// disconnect the joints between the markers
  /// and the character model.  We should make
  /// this less convoluted.
  void useMarkers(bool);

public slots:

  void setStepSize(double ss);
  void step();  ///< Advance the physical simulation forward in time
  void pauseSim(bool);
  void setSingleStep();

  void setGroundFriction(double friction);
  void setTerrainSoftness(double terrain);

  void setTerrainZ(double zVal); ///< Set the ground plane height

  /// Set the play-back state:
  /// Following InverseKinematics, InverseDynamics, or ForwardDynamics
  void follow(int type);

  void setSelfCollide(bool hitSelf);

  /// Open the specified file and write out recorded kinematics
  void anglesToFile(QString filename);



protected:
  dWorldID world; ///< Container for ODE dynamics
  dSpaceID space; ///< Container for ODE collision geometry

  dGeomID groundPlane; ///< Ground plane
  dGeomID mouseRay;    ///< For ray-picking
#if defined( BOARD_DATA )
  dGeomID board0;
  dGeomID board1;
#endif
  dJointGroupID contactGroup; ///< Holds contact joints

  bool paused;      ///< Is the simulation paused
  bool singleStep;  ///< Should the simulation advance by a single step

  double stepsize;  ///< How much time for a single step
  double groundSquish; ///< How 'soft' (CFM) are ground contacts
  double groundFriction; ///< A strong (Fmax) is ground friction

  CapBody* body;  ///< The character model


  /*
    In a fit of laziness, I took a quick and dirty route
    and simply copied MarkerData, rather than creating
    a proper parent class and subclassing it.
    Different incarnations of "MarkerData" expect different
    file formats and may contain other data such as ground
    forces or EMG data.
    */
  //SwingData* markData;
  //LiveMarkerData* markData;
#if defined( BOARD_DATA )
  BoardMarkerData* markData;
#elif defined( POKE_DATA )
  PokeMarkerData* markData;
#else
  MarkerData* markData;   ///< MoCap Marker data (from Phasespace C3D file)
#endif


  dSpaceID targetSpace;  ///< When analysis includes target geometry, they go here.
  dSpaceID markSpace;    ///< Spheres representing the markers go here

  bool selfCollide;  ///< Should the body collide with itself
  int followState;   ///< Are we following markers, joint angles, or torques?
  int seqFrame;      ///< What frame are we on as we move through an angle or torque sequence

  FILE* mainFile;    ///< Kludge file handle for writing out whatever data is interesting at the moment

public:

  Sequence* angleSequence;  ///< The stored sequence of joint angles (kinematics)
  Sequence* torqueSequence; ///< The stored sequence of torques/forces (dynamics)

  int activeGroundContacts; ///< How many contacts were made with the ground
  dJointFeedback groundFeedback[MAX_GROUND_FEEDBACK]; ///< Feedback structures for extracting ground forces
  dVector3 contactPoint[MAX_GROUND_FEEDBACK]; ///< Contact points between body and ground

  /// Lines showing whatever information happened to be needed:
  /// Ground forces, residual torques, center of mass, aggregate momentum, etc.
  QList<GLine> lineList;
  QList<int> leftFootFeedback;  ///< Lines specifically showing ground forces from left foot
  QList<int> rightFootFeedback; ///< Ground forces from right foot.


};


#endif // SIMWORLD_H
