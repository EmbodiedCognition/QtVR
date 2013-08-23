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
#include "testscript.h"

#include "simworld.h"
#include "CapBody.h"
#include "mainwindow.h"

TestScript::TestScript(MainWindow* win,QObject *parent) :
  ScriptBase(win,parent)
{
  testState = 0;

}


//virtual
void TestScript::startScript()
{
  SimWorld* sw = mainWin->getWorld();
  CapBody* cb = sw->getBody();

  // Load data to the model
  cb->loadBody();
  cb->copyToSim();
  // Set the marker file to a target frame.
  mainWin->setDataFrame(20000);
  mainWin->playPauseSim(true);
  mainWin->lightForces();
  mainWin->clearData();
  mainWin->useGlobalForces(false);

  testState = 1;
  eFrame = 0;

}

//virtual
void TestScript::updateScript()
{
  eFrame+=1;

  switch (testState) {
    case 1:
    // Start following markers
    if (eFrame>=100) {
      testState = 2;
      eFrame=0;
      mainWin->setDataFrame(20000);
      mainWin->lightForces();
      mainWin->clearData();
      mainWin->useMarkers(true);
      mainWin->playPauseData(true);
      mainWin->playPauseSim(true);
      mainWin->saveModel();
      mainWin->restoreModel();
    }
    break;
  case 2:
    if (eFrame>=500) {
      testState = 3;
      eFrame=0;
      mainWin->setDataFrame(20000);
      mainWin->strongForces();
      mainWin->useAltForces(true);
      mainWin->playPauseData(true);
      mainWin->playPauseSim(true);
      mainWin->restoreModel();
    }
    break;
  case 3:
    if (eFrame>=500) {
      testState = 4;
      eFrame = 0;
      mainWin->setDataFrame(20000);
      mainWin->zeroForces();
      mainWin->useTorques(true);
      mainWin->playPauseData(true);
      mainWin->playPauseSim(true);
      mainWin->restoreModel();
    }
    break;
  case 4:
    if (eFrame>=500) {
      testState = 0;
      eFrame = 0;
      mainWin->playPauseData(false);
      mainWin->playPauseSim(false);

    }
    break;
  default:
    eFrame=0;
    break;
  }

}

//virtual
void TestScript::endScript()
{

}
