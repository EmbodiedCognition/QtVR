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
#ifndef EXPERIMENTSCRIPT_H
#define EXPERIMENTSCRIPT_H

#include "scriptbase.h"
class Sequence;
class MainWindow;

class ExperimentScript : public ScriptBase
{
    Q_OBJECT
public:
    explicit ExperimentScript(MainWindow* win,QObject *parent = 0);

  void resetTrial();

signals:

public slots:
  /// Connect up events
  virtual void startScript();

  virtual void updateScript();

  virtual void endScript();

protected:



  /// The Pseudo marker data recorded from the first pass
  Sequence* baseSequence;

  /// Data being captured during a trial
  Sequence* markerSave;
  Sequence* angleSave;
  Sequence* torqueSave;

  /*
    We need to pass control away from this object
    to allow time to pass.  So we need to keep
    track of where we are when the time comes for
    us to take another step.
    */

  enum StateType {
    ExStopped,      ///< Not running
    ExInit,         ///< Started, getting things ready
    ExRelax,        ///< Attached markers to model, fitting model
    ExSavingAngle,  ///< Advancing time with attached markers to save angles
    ExSavingMarker, ///< Advancing time with angle targets, saving marker positions
    ExMarkerPass,
    ExAnglePass,
    ExForcePass,
    ExCleanup,      ///< Wrapping up
  } state;

  bool paused;      ///< We might pause the script temporarily and then continue
  int condition;    ///< What noise level are we on.
  int trial;        ///< Which trial are we on.
  int frameCount;

  static double noiseVal[];
  static int noiseNum;

  FILE* groundTruth;
  FILE* markerPassFile;
  FILE* anglePassFile;
};

#endif // EXPERIMENTSCRIPT_H
