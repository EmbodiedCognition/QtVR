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
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <jointwidget.h>
#include <markerwidget.h>
#include <QColorDialog>
#include <QColor>
#include <QDockWidget>
#include <QFileDialog>
#include <QMessageBox>
#include <QScrollArea>

#include "simworld.h"
#include "CapBody.h"

#include "markertab.h"
#include "plottab.h"
#include "boarddata.h"
#include "dsboxwithparams.h"
#include "testscript.h"


/**
 * @brief MainWindow::MainWindow Initialize the main window and everything else
 * @param parent
 */
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  // Initialize the interface
  ui->setupUi(this);
  ui->glWidget->addAction(ui->actionFullscreen);
  ui->glWidget->addAction(ui->actionExit);

  connect(ui->actionExit,SIGNAL(triggered()),this,SLOT(close()));
  connect(ui->actionFullscreen,SIGNAL(triggered()),this,SLOT(toggleFullscreenSlot()));

  // The test script runs a simple experiment
  experimentScript = new TestScript(this,this);

  // Add a cosmetic line to the splitter handle
  // so that it's easier to grab with the mouse
  QSplitterHandle *handle = ui->splitter->handle(1);
  QVBoxLayout *layout = new QVBoxLayout(handle);
  layout->setSpacing(0);
  layout->setMargin(0);
  QFrame *line = new QFrame(handle);
  line->setFrameShape(QFrame::VLine);
  line->setFrameShadow(QFrame::Sunken);
  layout->addWidget(line);

  // In a future revision, we'll move the SimWorld
  // to a more intelligent place.  It doesn't
  // belong as a child to the graphics widget.
  SimWorld* world = ui->glWidget->getWorld();
  CapBody* capBody = world->getBody();
#if defined( BOARD_DATA )
  BoardMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#elif defined( POKE_DATA )
  PokeMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#else
  MarkerData* md = world->getMarkerData();
#endif

  // We populate a ScrollArea with the same number of
  // MarkerWidgets as there are markers in the data file.
  QScrollArea* sa = new QScrollArea;
  QWidget* wid = new QWidget;
  layout = new QVBoxLayout;
  layout->setContentsMargins(0,0,0,0);

  // This part needs to become part of a function
  // that is called everytime a new data file is
  // loaded.
  int markCnt = md->marker_count;
  ui->markerCountLineEdit->setText(QString::number(markCnt));
  ui->frameCountLineEdit->setText(QString::number(md->size()));
  ui->markerFileLineEdit->setText("data.c3d");
  ui->markerFrameStartBox->setValue(0);
  setDataRange(md->size());

  QHBoxLayout* innerLayout = new QHBoxLayout;
  QPushButton* connectButton = new QPushButton("Connect");
  QPushButton* releaseButton = new QPushButton("Release");
  QPushButton* updateButton = new QPushButton("Update Anchors");
  innerLayout->addWidget(connectButton);
  innerLayout->addWidget(releaseButton);
  innerLayout->addWidget(updateButton);
  connect(connectButton,SIGNAL(clicked()),this,SLOT(connectMarkers()));
  connect(releaseButton,SIGNAL(clicked()),this,SLOT(releaseMarkers()));
  connect(updateButton,SIGNAL(clicked()),this,SLOT(updateMarkerAnchors()));
  layout->addLayout(innerLayout);

  markerWidgetArray.resize(markCnt);
  for (int ii=0;ii<markCnt;++ii) {
    markerWidgetArray[ii] = new MarkerWidget(ii,0);
    layout->addWidget(markerWidgetArray[ii]);
  }
  wid->setLayout(layout);
  sa->setWidget(wid);
  ui->controlTabWidget->addTab(sa,"Markers");

  // Joint control tab
  sa = new QScrollArea;
  wid = new QWidget;
  layout = new QVBoxLayout;
  layout->setContentsMargins(0,0,0,0);

  // A couple buttons at the top of the joint tab make it possible
  // to rapidly change all the force limits
  innerLayout = new QHBoxLayout;
  QPushButton* zeroButton = new QPushButton("Zero");
  QPushButton* lightButton = new QPushButton("Light");
  QPushButton* resetButton = new QPushButton("Strong");
  innerLayout->addWidget(zeroButton);
  innerLayout->addWidget(lightButton);
  innerLayout->addWidget(resetButton);
  layout->addLayout(innerLayout);
  connect(zeroButton,SIGNAL(clicked()),this,SLOT(zeroForces()));
  connect(lightButton,SIGNAL(clicked()),this,SLOT(lightForces()));
  connect(resetButton,SIGNAL(clicked()),this,SLOT(strongForces()));

  // Fill the tab with hard-coded data about the model...
  populateJointTab(capBody,layout);
  wid->setLayout(layout);
  sa->setWidget(wid);
  ui->controlTabWidget->addTab(sa,"Joints");

  // Body dimensions widget
  sa = new QScrollArea;
  wid = new QWidget;
  layout = new QVBoxLayout;
  layout->setContentsMargins(0,0,0,0);
  QCheckBox* keepBodyRel = new QCheckBox("Body Rel");
  layout->addWidget(keepBodyRel);

  // Fill the tab with model-specific widgets
  populateBodyTab(capBody,layout);
  wid->setLayout(layout);
  sa->setWidget(wid);
  ui->controlTabWidget->addTab(sa,"Model dimensions");

  // Hook up the interface elements to their respecitve functionality
  connect(ui->saveButton,SIGNAL(clicked()),capBody,SLOT(saveBody()));
  connect(ui->loadButton,SIGNAL(clicked()),capBody,SLOT(loadBody()));
  connect(ui->testButton,SIGNAL(clicked()),this,SLOT(experimentSlot()));

  // When a the CapBody loads the marker map, tell the interface
  // how and where the markers are connected.
  connect(capBody,SIGNAL(markMap(int,int)),this,SLOT(setMarkMap(int,int)));
  connect(capBody,SIGNAL(markPoint(int,double,double,double)),
          this,SLOT(setMarkPoint(int,double,double,double)));

  // *****
  // (Each MarkerWidget informs the capBody when the interface
  //  changes).
  for (int ii=0;ii<markCnt;++ii) {
    connect(markerWidgetArray[ii],SIGNAL(markBodySet(int,int)),md,SLOT(changeBodyConnect(int,int)));
    connect(markerWidgetArray[ii],SIGNAL(markConnect(int,bool)),md,SLOT(changeBodyLink(int,bool)));
    connect(markerWidgetArray[ii],SIGNAL(markPosSet(int,double,double,double)),
            md,SLOT(changeLinkPos(int,double,double,double)));
    connect(markerWidgetArray[ii],SIGNAL(markGrab(int)),this, SLOT(grabMarkPos(int)));
  }

  connect(ui->clearPlotButton,SIGNAL(clicked()),this,SLOT(clearData()));
  connect(ui->glWidget->getWorld(),SIGNAL(useMarkers(bool)),this,SLOT(usingMarkers(bool)));
  connect(ui->frictionSpinBox,SIGNAL(valueChanged(double)),world,SLOT(setGroundFriction(double)));
  connect(ui->terrainSpinBox,SIGNAL(valueChanged(double)),world,SLOT(setTerrainSoftness(double)));
  connect(ui->zBox,SIGNAL(valueChanged(double)),ui->glWidget->getWorld(),SLOT(setTerrainZ(double)));
  connect(ui->forceLinesCheckBox,SIGNAL(clicked(bool)),ui->glWidget,SLOT(setDrawLines(bool)));
  connect(ui->camFollowCheckBox,SIGNAL(clicked(bool)),ui->glWidget,SLOT(setFollowCamera(bool)));
  connect(ui->timeSlider,SIGNAL(valueChanged(int)),world->getMarkerData(),SLOT(setFrame(int)));
  connect(ui->glWidget->getWorld()->getMarkerData(),SIGNAL(frameChanged(int)),ui->timeSlider,SLOT(setValue(int)));
  connect(ui->bodyAlpha,SIGNAL(valueChanged(double)),ui->glWidget,SLOT(setBodyAlpha(double)));
  connect(ui->showMarkBox,SIGNAL(toggled(bool)),ui->glWidget,SLOT(setShowMarkers(bool)));
  connect(ui->selfCollideBox,SIGNAL(toggled(bool)),world,SLOT(setSelfCollide(bool)));
  connect(ui->saveStateButton,SIGNAL(clicked()),this,SLOT(saveModel()));
  connect(ui->restoreButton,SIGNAL(clicked()),this,SLOT(restoreModel()));

  connect(ui->markerRadio,SIGNAL(toggled(bool)),this,SLOT(useMarkers(bool)));
  connect(ui->torqueRadio,SIGNAL(toggled(bool)),this,SLOT(useTorques(bool)));
  connect(ui->altRadio,SIGNAL(toggled(bool)),this,SLOT(useAltForces(bool)));

  connect(ui->playAllButton,SIGNAL(clicked(bool)),this,SLOT(playPauseAll(bool)));
  connect(ui->playPauseDataButton,SIGNAL(clicked(bool)),this,SLOT(playPauseData(bool)));
  connect(ui->playPauseSimButton,SIGNAL(clicked(bool)),this,SLOT(playPauseSim(bool)));

  connect(ui->stepAllButton,SIGNAL(clicked()),this,SLOT(stepAll()));
  connect(ui->stepDataButton,SIGNAL(clicked()),this,SLOT(stepData()));
  connect(ui->stepSimButton,SIGNAL(clicked()),this,SLOT(stepSim()));


  connect(ui->selectFileToolButton,SIGNAL(clicked()),this,SLOT(markerFileDialog()));
  connect(ui->dataStepBox,SIGNAL(valueChanged(int)),this,SLOT(setDataStep(int)));
  connect(ui->dataFrameBox,SIGNAL(valueChanged(int)),this,SLOT(setDataFrame(int)));
  connect(ui->timeSlider,SIGNAL(valueChanged(int)),ui->dataFrameBox,SLOT(setValue(int)));

  // *****
  // Need to dynamically change this
  ui->timeSlider->setRange(0,world->getMarkerData()->size()-1);
  connect(ui->globalBox,SIGNAL(toggled(bool)),this,SLOT(useGlobalForces(bool)));

#if defined( BOARD_DATA )
  bd = new BoardData(this);
  bd->loadData("boardData.dat");
  ui->glWidget->setBoardData(bd);
#endif

  // Start the timer for updating the data, sim, and graphics
  updateTimer.setSingleShot(false);
  connect(&updateTimer,SIGNAL(timeout()),this,SLOT(updateLoop()));

  connect(ui->frameTimeBox,SIGNAL(valueChanged(double)),this,SLOT(setFrameTime(double)));
  ui->frameTimeBox->setValue(1/60.0);
  ui->dataStepBox->setValue(2);

  capBody->loadBody();
  useMarkers(true);
  saveModel();
}

MainWindow::~MainWindow()
{
  delete ui;

#if defined( BOARD_DATA )
  delete bd;
#endif
}

void MainWindow::makeJointWidget(QString label,int joint,
                                 int axis,CapBody* cb,
                                 QVBoxLayout* lay)
{
  JointWidget* jw = new JointWidget(label,joint,axis,0);
  connect(jw,SIGNAL(valueChanged(int,int,double)),
          cb,SLOT(setJointOffset(int,int,double)));
  connect(jw,SIGNAL(forceChanged(int,int,double)),
          cb,SLOT(setJointForce(int,int,double)));
  connect(cb,SIGNAL(targetChange(int,int,double)),
          jw,SLOT(setValue(int,int,double)));

  lay->addWidget(jw);
  jointWidgets.push_back(jw);
}

/**
 * @brief MainWindow::populateJointTab Fill the "Joints" tab with controls
 * @param cb Pointer to the model
 * @param lay Layout for the Tab widget
 *
 *  This function creates all of the JointWidgets that allow
 * the user to control the independent degrees of freedom
 * of the character model.
 *
 *  In the future, the DoF labels and associated degrees of freedom
 * should be loaded from a file or database so that you don't have
 * to remember to change this function everytime you make a change
 * to the model.
 */
void MainWindow::populateJointTab(CapBody* capBody,
                                  QVBoxLayout* layout)
{
  makeJointWidget("RootX",CapBody::ROOT_LINMOTOR_JOINT,0,capBody,layout);
  makeJointWidget("RootY",CapBody::ROOT_LINMOTOR_JOINT,1,capBody,layout);
  makeJointWidget("RootZ",CapBody::ROOT_LINMOTOR_JOINT,2,capBody,layout);

  makeJointWidget("Pitch",CapBody::ROOT_ANGMOTOR_JOINT,0,capBody,layout);
  makeJointWidget("Roll",CapBody::ROOT_ANGMOTOR_JOINT,1,capBody,layout);
  makeJointWidget("Yaw",CapBody::ROOT_ANGMOTOR_JOINT,2,capBody,layout);

  makeJointWidget("ChinX",CapBody::CHIN_JOINT,0,capBody,layout);
  makeJointWidget("ChinY",CapBody::CHIN_JOINT,2,capBody,layout);
  makeJointWidget("ChinZ",CapBody::CHIN_JOINT,1,capBody,layout);

  makeJointWidget("NeckX",CapBody::THROAT_JOINT,0,capBody,layout);
  makeJointWidget("NeckY",CapBody::THROAT_JOINT,2,capBody,layout);
  makeJointWidget("NeckZ",CapBody::THROAT_JOINT,1,capBody,layout);

  makeJointWidget("R.CollarX",CapBody::R_COLLAR_JOINT,0,capBody,layout);
  makeJointWidget("R.CollarY",CapBody::R_COLLAR_JOINT,2,capBody,layout);
  makeJointWidget("R.CollarZ",CapBody::R_COLLAR_JOINT,1,capBody,layout);

  makeJointWidget("R.ShoulderX",CapBody::R_SHOULDER_JOINT,0,capBody,layout);
  makeJointWidget("R.ShoulderY",CapBody::R_SHOULDER_JOINT,2,capBody,layout);
  makeJointWidget("R.ShoulderZ",CapBody::R_SHOULDER_JOINT,1,capBody,layout);

  makeJointWidget("R.ElbowX",CapBody::R_ELBOW_JOINT,0,capBody,layout);
  makeJointWidget("R.ElbowZ",CapBody::R_ELBOW_JOINT,1,capBody,layout);

  makeJointWidget("R.WristX",CapBody::R_WRIST_JOINT,0,capBody,layout);
  makeJointWidget("R.WristY",CapBody::R_WRIST_JOINT,1,capBody,layout);

  makeJointWidget("L.CollarX",CapBody::L_COLLAR_JOINT,0,capBody,layout);
  makeJointWidget("L.CollarY",CapBody::L_COLLAR_JOINT,2,capBody,layout);
  makeJointWidget("L.CollarZ",CapBody::L_COLLAR_JOINT,1,capBody,layout);

  makeJointWidget("L.ShoulderX",CapBody::L_SHOULDER_JOINT,0,capBody,layout);
  makeJointWidget("L.ShoulderY",CapBody::L_SHOULDER_JOINT,2,capBody,layout);
  makeJointWidget("L.ShoulderZ",CapBody::L_SHOULDER_JOINT,1,capBody,layout);

  makeJointWidget("L.ElbowX",CapBody::L_ELBOW_JOINT,0,capBody,layout);
  makeJointWidget("L.ElbowZ",CapBody::L_ELBOW_JOINT,1,capBody,layout);

  makeJointWidget("L.WristX",CapBody::L_WRIST_JOINT,0,capBody,layout);
  makeJointWidget("L.WristY",CapBody::L_WRIST_JOINT,1,capBody,layout);

  makeJointWidget("SpineX",CapBody::SPINE_JOINT,0,capBody,layout);
  makeJointWidget("SpineY",CapBody::SPINE_JOINT,2,capBody,layout);
  makeJointWidget("SpineZ",CapBody::SPINE_JOINT,1,capBody,layout);

  makeJointWidget("WaistX",CapBody::WAIST_JOINT,0,capBody,layout);
  makeJointWidget("WaistY",CapBody::WAIST_JOINT,2,capBody,layout);
  makeJointWidget("WaistZ",CapBody::WAIST_JOINT,1,capBody,layout);

  makeJointWidget("R.HipX",CapBody::R_HIP_JOINT,0,capBody,layout);
  makeJointWidget("R.HipY",CapBody::R_HIP_JOINT,2,capBody,layout);
  makeJointWidget("R.HipZ",CapBody::R_HIP_JOINT,1,capBody,layout);

  makeJointWidget("R.KneeX",CapBody::R_KNEE_JOINT,0,capBody,layout);
  makeJointWidget("R.KneeZ",CapBody::R_KNEE_JOINT,1,capBody,layout);

  makeJointWidget("R.AnkleX",CapBody::R_ANKLE_JOINT,0,capBody,layout);
  makeJointWidget("R.AnkleY",CapBody::R_ANKLE_JOINT,1,capBody,layout);

  makeJointWidget("R.Foot",CapBody::R_FOOT_JOINT,0,capBody,layout);

  makeJointWidget("L.HipX",CapBody::L_HIP_JOINT,0,capBody,layout);
  makeJointWidget("L.HipY",CapBody::L_HIP_JOINT,2,capBody,layout);
  makeJointWidget("L.HipZ",CapBody::L_HIP_JOINT,1,capBody,layout);

  makeJointWidget("L.KneeX",CapBody::L_KNEE_JOINT,0,capBody,layout);
  makeJointWidget("L.KneeZ",CapBody::L_KNEE_JOINT,1,capBody,layout);

  makeJointWidget("L.AnkleX",CapBody::L_ANKLE_JOINT,0,capBody,layout);
  makeJointWidget("L.AnkleY",CapBody::L_ANKLE_JOINT,1,capBody,layout);

  makeJointWidget("L.Foot",CapBody::L_FOOT_JOINT,0,capBody,layout);
}

void MainWindow::makeBodyWidget(QString label,int body,
                                int axis,CapBody* cb,
                                QVBoxLayout* lay)
{
  QHBoxLayout* hl = new QHBoxLayout;
  QLabel* lab = new QLabel(label);
  hl->addWidget(lab);
  DSBoxWithParams* spin=new DSBoxWithParams(body,axis);
  spin->setMinimum(0.01);
  spin->setMaximum(2);
  spin->setDecimals(3);     // Millimeter resolution
  spin->setSingleStep(.01); // Centimeter steps
  hl->addWidget(spin);

  // We need a way to connect this thing to the
  // appropriate setter in the body object.
  cb->setBodySpin(body,axis,spin);

  lay->addLayout(hl);
}

/**
 * @brief MainWindow::populateBodyTab Allow control of body-part dimensions
 * @param capBody Pointer to the character model
 * @param layout Layout of the BodyDim
 *
 *  This function creates widgets for manipulating the dimensions
 * of the individual body parts of the character model.
 *  As with the populateJointsTab above, this should be loaded
 * from a model file rather than hard-coded here.
 */
void MainWindow::populateBodyTab(CapBody* capBody,
                                 QVBoxLayout* layout)
{
  makeBodyWidget("Head Rad",CapBody::HEAD_BODY,0,capBody,layout);
  makeBodyWidget("Neck X",CapBody::NECK_BODY,0,capBody,layout);
  makeBodyWidget("Neck Y",CapBody::NECK_BODY,1,capBody,layout);
  makeBodyWidget("Neck Z",CapBody::NECK_BODY,2,capBody,layout);
  makeBodyWidget("UpTorso Rad",CapBody::UP_TORSO_BODY,0,capBody,layout);
  makeBodyWidget("UpTorso Len",CapBody::UP_TORSO_BODY,2,capBody,layout);
  makeBodyWidget("R.Collar Rad",CapBody::R_COLLAR_BODY,0,capBody,layout);
  makeBodyWidget("R.Collar Len",CapBody::R_COLLAR_BODY,2,capBody,layout);
  makeBodyWidget("R.UpArm Rad",CapBody::RUP_ARM_BODY,0,capBody,layout);
  makeBodyWidget("R.UpArm Len",CapBody::RUP_ARM_BODY,2,capBody,layout);
  makeBodyWidget("R.LoArm Rad",CapBody::RLO_ARM_BODY,0,capBody,layout);
  makeBodyWidget("R.LoArm Len",CapBody::RLO_ARM_BODY,2,capBody,layout);
  makeBodyWidget("R.Hand Rad",CapBody::R_HAND_BODY,0,capBody,layout);
  makeBodyWidget("L.Collar Rad",CapBody::L_COLLAR_BODY,0,capBody,layout);
  makeBodyWidget("L.Collar Len",CapBody::L_COLLAR_BODY,2,capBody,layout);
  makeBodyWidget("L.UpArm Rad",CapBody::LUP_ARM_BODY,0,capBody,layout);
  makeBodyWidget("L.UpArm Len",CapBody::LUP_ARM_BODY,2,capBody,layout);
  makeBodyWidget("L.LoArm Rad",CapBody::LLO_ARM_BODY,0,capBody,layout);
  makeBodyWidget("L.LoArm Len",CapBody::LLO_ARM_BODY,2,capBody,layout);
  makeBodyWidget("L.Hand Rad",CapBody::L_HAND_BODY,0,capBody,layout);
  makeBodyWidget("LoTorso X",CapBody::LO_TORSO_BODY,0,capBody,layout);
  makeBodyWidget("LoTorso Y",CapBody::LO_TORSO_BODY,1,capBody,layout);
  makeBodyWidget("LoTorso Z",CapBody::LO_TORSO_BODY,2,capBody,layout);
  makeBodyWidget("Waist Rad",CapBody::WAIST_BODY,0,capBody,layout);
  makeBodyWidget("Waist Len",CapBody::WAIST_BODY,2,capBody,layout);
  makeBodyWidget("R.UpLeg Rad",CapBody::RUP_LEG_BODY,0,capBody,layout);
  makeBodyWidget("R.UpLeg Len",CapBody::RUP_LEG_BODY,2,capBody,layout);
  makeBodyWidget("R.LoLeg Rad",CapBody::RLO_LEG_BODY,0,capBody,layout);
  makeBodyWidget("R.LoLeg Len",CapBody::RLO_LEG_BODY,2,capBody,layout);
  makeBodyWidget("R.Heel Rad",CapBody::R_HEEL_BODY,0,capBody,layout);
  makeBodyWidget("R.Tarsal Rad",CapBody::R_TARSAL_BODY,0,capBody,layout);
  makeBodyWidget("R.Tarsal Len",CapBody::R_TARSAL_BODY,2,capBody,layout);
  makeBodyWidget("L.UpLeg Rad",CapBody::LUP_LEG_BODY,0,capBody,layout);
  makeBodyWidget("L.UpLeg Len",CapBody::LUP_LEG_BODY,2,capBody,layout);
  makeBodyWidget("L.LoLeg Rad",CapBody::LLO_LEG_BODY,0,capBody,layout);
  makeBodyWidget("L.LoLeg Len",CapBody::LLO_LEG_BODY,2,capBody,layout);
  makeBodyWidget("L.Heel Rad",CapBody::L_HEEL_BODY,0,capBody,layout);
  makeBodyWidget("L.Tarsal Rad",CapBody::L_TARSAL_BODY,0,capBody,layout);
  makeBodyWidget("L.Tarsal Len",CapBody::L_TARSAL_BODY,2,capBody,layout);
}

SimWorld* MainWindow::getWorld()
{
  return ui->glWidget->getWorld();
}

void MainWindow::toggleFullscreenSlot()
{
  ui->glWidget->setWindowFlags(
        ui->glWidget->windowFlags()^Qt::Window);
  // This does nothing if we're not a window.
  ui->glWidget->showFullScreen();
}



/**
 * @brief MainWindow::playPauseAll Start or stop both simulation and data
 * @param play
 */
void MainWindow::playPauseAll(bool play)
{
  playPauseSim(play);
  playPauseData(play);
}

void MainWindow::playPauseSim(bool play)
{
  SimWorld* world = ui->glWidget->getWorld();
  if (world->isPaused() != play) return;
  world->pauseSim(!play);

  updatePlayButtons();
}

void MainWindow::playPauseData(bool play)
{
  MarkerData* markerData = ui->glWidget->getWorld()->getMarkerData();
  if (markerData->isPaused() != play) return;
  markerData->setPaused(!play);

  updatePlayButtons();
}

void MainWindow::updatePlayButtons()
{
  SimWorld* world = ui->glWidget->getWorld();
  MarkerData* markerData = world->getMarkerData();

  if (world->isPaused()) {
    ui->playPauseSimButton->setIcon(QIcon(":/icons/data/media-playback-start-3.png"));
    ui->playPauseSimButton->setChecked(false);
  } else {
    ui->playPauseSimButton->setIcon(QIcon(":/icons/data/media-playback-pause-3.png"));
    ui->playPauseSimButton->setChecked(true);
  }
  if (markerData->isPaused()) {
    ui->playPauseDataButton->setIcon(QIcon(":/icons/data/media-playback-start-3.png"));
    ui->playPauseDataButton->setChecked(false);
  } else {
    ui->playPauseDataButton->setIcon(QIcon(":/icons/data/media-playback-pause-3.png"));
    ui->playPauseDataButton->setChecked(true);  }

  if (!(world->isPaused()||markerData->isPaused())) {
    ui->playAllButton->setIcon(QIcon(":/icons/data/media-playback-pause-3.png"));
    ui->playAllButton->setChecked(true);
  } else {
    ui->playAllButton->setIcon(QIcon(":/icons/data/media-playback-start-3.png"));
    ui->playAllButton->setChecked(false);
  }
}




void MainWindow::setMarkMap(int id,int bod)
{
  markerWidgetArray[id]->setMarkBody(bod+1);
  markerWidgetArray[id]->setConnect(bod>=0);
}

void MainWindow::setMarkPoint(int id,double xx,double yy,double zz)
{
  markerWidgetArray[id]->setMarkPos(xx,yy,zz);
}

void MainWindow::stepSim()
{
  playPauseSim(false);
  ui->glWidget->getWorld()->setSingleStep();

}

void MainWindow::stepData()
{
  playPauseData(false);
  ui->glWidget->getWorld()->getMarkerData()->setSingleStep();
}

void MainWindow::stepAll()
{
  stepSim();
  stepData();
}

/**
 * @brief MainWindow::setDataFrame Set the active data frame
 * @param frame
 */
void MainWindow::setDataFrame(int frame)
{
  //LiveMarkerData* md = ui->glWidget->getWorld()->getMarkerData();

#if defined( BOARD_DATA )
  BoardMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#elif defined( POKE_DATA )
  PokeMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#else
  MarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#endif

  md->setFrame(frame);
}

/**
 * @brief MainWindow::setDataStep  Set how many frames pass with each time step
 * @param stepSize Number of frames to advance with each step
 */
void MainWindow::setDataStep(int stepSize)
{
  //LiveMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#if defined( BOARD_DATA )
  BoardMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#elif defined( POKE_DATA )
  PokeMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#else
  MarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#endif

  md->setStepSize(stepSize);
}

/**
 * @brief MainWindow::setFrameTime How long is each time step
 * @param step Time in milliseconds
 *
 * This also has the side-effect of setting the animation
 * speed.  Maybe it shouldn't.
 */
void MainWindow::setFrameTime(double step)
{
  getWorld()->setStepSize(step);
  updateTimer.start(int(1000*step));
}

/**
 * @brief MainWindow::grabMarkPos Anchor a marker at its current position
 * @param mark The marker to anchor
 *
 *  Compute the location of the marker in body-relative
 * coordinates.  Set the joint anchor at that point.
 */
void MainWindow::grabMarkPos(int mark)
{
  CapBody* body = ui->glWidget->getWorld()->getBody();

  //LiveMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#if defined( BOARD_DATA )
  BoardMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#elif defined( POKE_DATA )
  PokeMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#else
  MarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#endif

  const dReal* markPos = dBodyGetPosition(md->body[mark]);

  dVector3 pos;
  dBodyGetPosRelPoint(body->body_segments[body->marker_to_body[mark].id],
                      markPos[0],markPos[1],markPos[2],pos);

  markerWidgetArray[mark]->setMarkPos(pos[0],pos[1],pos[2]);
}

/**
 * @brief MainWindow::updateLoop Main animation loop
 *
 * Advance the script.  Then advance the simulation and
 * data if appropriate.  Then render.
 */
void MainWindow::updateLoop()
{
  experimentScript->updateScript();
  // The animate function currently
  // updates the simulation.  This
  // should change.
  ui->glWidget->animate();
}

void MainWindow::usingMarkers(bool used)
{
  int markCnt = (int) markerWidgetArray.size();
  for (int ii=0;ii<markCnt;++ii) {
    markerWidgetArray[ii]->setConnect(used);
  }
}

void MainWindow::connectMarkers()
{
  usingMarkers(true);
}

void MainWindow::releaseMarkers()
{
  usingMarkers(false);
}

void MainWindow::updateMarkerAnchors()
{
  int markCnt = (int) markerWidgetArray.size();
  for (int ii=0;ii<markCnt;++ii) {
    markerWidgetArray[ii]->grab();
  }
}


void MainWindow::useAngles(bool toggle)
{
  if (toggle) {
    ui->glWidget->getWorld()->follow(1);
  }
}

void MainWindow::useMarkers(bool toggle)
{
  if (toggle) {
    ui->glWidget->getWorld()->follow(0);
    lightForces();
  }
}

void MainWindow::useTorques(bool toggle)
{
  if (toggle) {
    ui->glWidget->getWorld()->follow(2);
    zeroForces();
  }
}

void MainWindow::useAltForces(bool toggle)
{
  if (toggle) {
    ui->glWidget->getWorld()->follow(3);
    strongForces();
  }
}

void MainWindow::setDataRange(int maxSize)
{
  ui->timeSlider->setRange(0,maxSize);
  ui->dataFrameBox->setRange(0,maxSize);
  ui->markerFrameStartBox->setRange(0,maxSize);
  ui->markerFrameStopBox->setRange(0,maxSize);
  ui->markerFrameStartBox->setValue(0);
  ui->markerFrameStopBox->setValue(maxSize);
  ui->timeSlider->setValue(0);
  ui->dataFrameBox->setValue(0);
}

/**
 * @brief MainWindow::loadMarkerFile Open and load some marker data
 * @param filename
 *
 * Pending...
 *  We determine an appropriate sub-class of
 * MarkerData from the extension.  We create it and then
 * we hook up all the appropriate pieces.
 */
void MainWindow::loadMarkerFile(QString filename)
{
#if defined( POKE_DATA )
  if (!filename.isNull()) {
    ui->glWidget->getWorld()->getMarkerData()->loadData(filename.toAscii());
  }
#else
  ui->markerFileLineEdit->setText(filename);
  QMessageBox::warning(this,"Unimplemented",
                       "Dynamic data loading not yet implemented.\n"
                       "Filename currently hardcoded\n"
                       "See MarkerData constructor in markerdata.cpp."
                       );
#endif
}


void MainWindow::experimentSlot()
{
  experimentScript->startScript();
}

void MainWindow::saveModel()
{
  getWorld()->getBody()->saveState();
}

void MainWindow::restoreModel()
{
  getWorld()->getBody()->restoreState();
}

void MainWindow::clearData()
{
  SimWorld* w = getWorld();
  w->angle_sequence->clear();
  w->torque_sequence->clear();
}

void MainWindow::strongForces()
{
  int count = jointWidgets.size();
  for (int ii=0;ii<6;++ii) {
    jointWidgets[ii]->setForceLimit(9999);
  }

  for (int ii=6;ii<count;++ii) {
    jointWidgets[ii]->setForceLimit(9999);
  }
}

void MainWindow::lightForces()
{
  int count = jointWidgets.size();
  for (int ii=0;ii<6;++ii) {
    jointWidgets[ii]->setForceLimit(0);
  }

  for (int ii=6;ii<count;++ii) {
    jointWidgets[ii]->setForceLimit(50);
  }
}

void MainWindow::zeroForces()
{
  int count = jointWidgets.size();
  for (int ii=0;ii<6;++ii) {
    jointWidgets[ii]->setForceLimit(0);
  }

  for (int ii=6;ii<count;++ii) {
    jointWidgets[ii]->setForceLimit(0);
  }
}


void MainWindow::saveBodyAngles(QString filename)
{
  getWorld()->anglesToFile(filename);
}


void MainWindow::writeVMarkerFrame(int cc,int tri,FILE* file)
{
  getWorld()->writeVMarkerFrame(cc,tri,file);
}

void MainWindow::writeAngleFrame(int cc,int tri,FILE* file)
{
  getWorld()->writeAngleFrame(cc,tri,file);
}


void MainWindow::useGlobalForces(bool use)
{
  getWorld()->getBody()->global_forces = use;
}

void MainWindow::markerFileDialog()
{
  QString filename = QFileDialog::getOpenFileName(this,"Select marker data file",QString(),"*.c3d");
  if (filename.isNull() || filename.isEmpty()) return;

  loadMarkerFile(filename);

}
