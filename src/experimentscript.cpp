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
#include "experimentscript.h"

#include "mainwindow.h"
#include "CapBody.h"
#include "simworld.h"
#include "markerdata.h"


double ExperimentScript::noiseVal[] = {0.1,0.5,1,2,4,8,16,32,64};
int ExperimentScript::noiseNum = 9;

ExperimentScript::ExperimentScript(MainWindow* win,QObject *parent) :
    ScriptBase(win,parent)
{
}


/**
  */
void ExperimentScript::startScript()
{
  paused = false;
  state = ExInit;

  SimWorld* sw = mainWin->getWorld();
  CapBody* cb = sw->getBody();


  groundTruth = fopen("groundTruth.txt","w");
  markerPassFile = fopen("markerPass.txt","w");
  anglePassFile= fopen("anglePass.txt","w");

  // Load data to the model
  cb->loadBody();
  cb->copyToSim();
  // Set the marker file to a target frame.
  mainWin->setDataFrame(49960);
  mainWin->playPauseSim(true);
  mainWin->lightForces();
  mainWin->clearData();

  state = ExRelax;
  frameCount = 0;

  condition = 0;
  trial = 0;


}


void ExperimentScript::updateScript()
{
  switch (state) {
    case ExRelax: // We're just counting down.
      frameCount++;
      if (frameCount==80) {
        // Start the data moving
        mainWin->playPauseData(true);
      }
      if (frameCount==90) {
        // Start recording
        // Capture the pose
        mainWin->saveModel();
        // Clear trajectories
        mainWin->clearData();
        // Keep a little bit of stiffness
        mainWin->lightForces();
        // Attach the markers (generally already attached)
        mainWin->useMarkers(true);
        // Hit play.
        mainWin->playPauseData(true);
      }
      if (frameCount>=100) {
        //We're now at 50000
        frameCount = 0;
        state = ExSavingAngle;
      }
      break;
    case ExSavingAngle:
    frameCount++;
    if (frameCount>=271) {
      // writeout the angle trajectories
      mainWin->saveBodyAngles("startAngles.txt");
      mainWin->restoreModel();
      mainWin->setDataFrame(49980);
      mainWin->useAltForces(true);
      mainWin->strongForces();

      frameCount = 0;
      state = ExSavingMarker;
      //mainWin->getWorld()->getMarkerData()->captureVirtualMarkers();
    }
    break;
  case ExSavingMarker:
    frameCount++;
    // We need to save virtual marker frames and joint angles.

    // Writes marker positions and joint angles
    mainWin->writeAngleFrame(-1,0,groundTruth);


    if (frameCount==10) {
      mainWin->saveModel();
    }
    if (frameCount>=10) {
      mainWin->getWorld()->getMarkerData()->captureVirtualMarkers();
    }

    if (frameCount>=271) {
      // todo: writeout the marker/force data


      trial = -1;
      resetTrial();
    }
    break;
  case ExMarkerPass:
    frameCount++;

    mainWin->writeVMarkerFrame(condition,trial,markerPassFile);

    if (frameCount>=261) {
      //mainWin->playPauseData(false);
      //mainWin->playPauseSim(false);

      //mainWin->useAngles(true);
      mainWin->useAltForces(true);
      mainWin->strongForces();
      mainWin->setDataFrame(50000);
      mainWin->restoreModel();

      mainWin->getWorld()->getMarkerData()->useVirtualMarkers(true);

      frameCount = 0;
      state = ExAnglePass;
    }

    break;
  case ExAnglePass:
    frameCount++;

    // Write out vmarkers, angles, and forces
    mainWin->writeAngleFrame(condition,trial,anglePassFile);

    if (frameCount>=261) {
      //mainWin->playPauseData(false);
      //mainWin->playPauseSim(false);

      mainWin->useTorques(true);
      mainWin->zeroForces();
      mainWin->setDataFrame(50000);
      mainWin->restoreModel();

      mainWin->getWorld()->getMarkerData()->useVirtualMarkers(true);

      frameCount = 0;
      state = ExForcePass;
    }
    break;
  case ExForcePass:
    frameCount++;
    if (frameCount>=261) {
//      frameCount = 0;
//      mainWin->useTorques(true);
//      mainWin->setDataFrame(50000);
//      mainWin->restoreModel();

//      state = ExForcePass;
      resetTrial();
    }
    break;

  default:
    break;
  }
}

void ExperimentScript::endScript()
{

  fclose(anglePassFile);
  fclose(markerPassFile);
  fclose(groundTruth);

  mainWin->close();
}

/**
  20 times per condition,

  */
void ExperimentScript::resetTrial()
{
  trial++;

  fflush(anglePassFile);
  fflush(markerPassFile);
  fflush(groundTruth);

  if (trial>=20) {
    trial = 0;
    condition++;
    if (condition>=noiseNum) {
      state = ExForcePass;
      endScript();

    }
  }

  // Data are in milimeters
  mainWin->getWorld()->getMarkerData()->perturbShadowFrame(noiseVal[condition]);
  mainWin->playPauseData(false);
  mainWin->playPauseSim(false);
  mainWin->useMarkers(true);
  mainWin->lightForces();
  mainWin->setDataFrame(50000);
  mainWin->clearData();
  mainWin->restoreModel();
  mainWin->getWorld()->getMarkerData()->useVirtualMarkers(true);

  mainWin->playPauseData(true);
  mainWin->playPauseSim(true);
  // Each time through, we restore, reset markers, set noise level
  frameCount = 0;
  state = ExMarkerPass;
}

