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
#ifndef MARKERDATA_H
#define MARKERDATA_H

#include <QObject>
#include <ode/ode.h>
#include "c3ddata.h"



class GaussRand;
class CapBody;

/**
  This class can load a .c3d file and then
  keeps a collection of ODE bodies aligned
  with the data.  You can advance through
  the data at a given rate or keep things
  constant or jump to a given frame.
  */
class MarkerData : public QObject
{
    Q_OBJECT
public:
    explicit MarkerData(dWorldID world,dSpaceID space,QObject *parent = 0);
  virtual ~MarkerData();



  /// We set the markers to be where the data frame
  /// says they should be.  We set their velocity to
  /// match where they will be with the next step.
  void step();

  // Need to check on dimensions

  void setStepSize(int stepSize) {frames_per_step=stepSize;}
  /// This allows us to set the body's velocities correctly.
  void setStepTime(double stepTime) {time_per_step = stepTime;}

  /// For rendering purposes.
  dSpaceID getSpace() { return space; }

  int size();

  bool isPaused() {return paused;}


signals:
  /// We announce when the frame is changed so that the
  /// scroll-bar can stay current.
  void frameChanged(int);

public slots:

  void setPaused(bool paused);
  void setSingleStep();
  void setFrame(int frame);

  void changeBodyConnect(int mark,int body);
  void changeBodyLink(int mark,bool link);
  void changeLinkPos(int mark,double xx,double yy,double zz);

  void captureVirtualMarkers(); ///< Grab a frame of virtual marker data
  void clearVirtualMarkers();   ///< Erase saved v.marker data
  void useVirtualMarkers(bool); ///< Use v.markers.


  void perturbShadowFrame(double sigma);


public:
  int frames_per_step;
  double time_per_step;
  int current_frame;
  /// If the object is paused, the markers have constant velocity.
  bool paused;
  bool single_step;
  dSpaceID space;
  dWorldID world;

  dGeomID* geom;
  dBodyID* body;
  dJointID* joint;
  dJointGroupID joint_group;
  bool* try_link;

  C3dFloatFrame* c_frame;
  C3dFloatFrame* n_frame;

  C3dFloatFrame* old_c_frame;
  C3dFloatFrame* old_n_frame;

  int marker_count;
  CapBody* body_pointer;


  // C3d Data Structure
  C3dFile data;

  /*
    For my perturbation experiment, I hacked in a second
    marker data source.  The virtual marker positions
    are captured from the character model during a forward
    dynamics pass.
    We can then perturb the virtual markers with Gaussian
    noise.
    */
  int virtual_point; ///< Where we are if following virtual data.
  QList<C3dFloatFrame*> virtual_data; ///< Virtual marker data
  QList<C3dFloatFrame*> shadow_data; ///< Perturbed copy of vData

  bool use_virtual_markers; ///< Should we draw marker data from the virtual source?


  GaussRand* grand;

};

#endif // MARKERDATA_H
