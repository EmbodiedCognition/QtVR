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
#include <QScrollArea>

#include "simworld.h"
#include "CapBody.h"

#include "markertab.h"
#include "plottab.h"
#include "boarddata.h"
#include "dsboxwithparams.h"
//#include "experimentscript.h"
#include "testscript.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->glWidget->addAction(ui->actionFullscreen);
    ui->glWidget->addAction(ui->actionExit);

    //es = new ExperimentScript(this,this);
    experimentScript = new TestScript(this,this);

    QSplitterHandle *handle = ui->splitter->handle(1);
    QVBoxLayout *layout = new QVBoxLayout(handle);
    layout->setSpacing(0);
    layout->setMargin(0);

    QFrame *line = new QFrame(handle);
    line->setFrameShape(QFrame::VLine);
    line->setFrameShadow(QFrame::Sunken);
    layout->addWidget(line);


    ui->controlTabWidget->addTab(new MarkerTab,"Markers");
    ui->controlTabWidget->addTab(new PlotTab,"Plots");


    SimWorld* world = ui->glWidget->getWorld();
    CapBody* capBody = world->getBody();
#if defined( BOARD_DATA )
    BoardMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#elif defined( POKE_DATA )
    PokeMarkerData* md = ui->glWidget->getWorld()->getMarkerData();
#else
    MarkerData* md = world->getMarkerData();
#endif

    // Markers widget
    QDockWidget* dw = new QDockWidget("Markers",this);


    QScrollArea* sa = new QScrollArea;
    QWidget* wid = new QWidget;
    layout = new QVBoxLayout;
    layout->setContentsMargins(0,0,0,0);
    markerWidgetArray = new MarkerWidget*[md->marker_count];
    for (int ii=0;ii<md->marker_count;++ii) {
      markerWidgetArray[ii] = new MarkerWidget(ii,0);
      layout->addWidget(markerWidgetArray[ii]);
    }
    wid->setLayout(layout);
    sa->setWidget(wid);
    dw->setWidget(sa);
    this->addDockWidget(Qt::RightDockWidgetArea,dw);
    //this->tabifyDockWidget(ui->bodyDockWidget,dw);

    QDockWidget* ow = dw;

    // Joints widget
    dw = new QDockWidget("Joints",this);
    sa = new QScrollArea;
    wid = new QWidget;
    layout = new QVBoxLayout;
    layout->setContentsMargins(0,0,0,0);

    QHBoxLayout* innerLayout = new QHBoxLayout;
    QPushButton* zeroButton = new QPushButton("Zero");
    QPushButton* lightButton = new QPushButton("Light");
    QPushButton* resetButton = new QPushButton("Strong");
    innerLayout->addWidget(zeroButton);
    innerLayout->addWidget(lightButton);
    innerLayout->addWidget(resetButton);
    layout->addLayout(innerLayout);

    connect(ui->saveStateButton,SIGNAL(clicked()),this,SLOT(saveModel()));
    connect(ui->restoreButton,SIGNAL(clicked()),this,SLOT(restoreModel()));

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
    //makeJointWidget("R.Toe",CapBody::R_TOE_JOINT,0,capBody,layout);

    makeJointWidget("L.HipX",CapBody::L_HIP_JOINT,0,capBody,layout);
    makeJointWidget("L.HipY",CapBody::L_HIP_JOINT,2,capBody,layout);
    makeJointWidget("L.HipZ",CapBody::L_HIP_JOINT,1,capBody,layout);

    makeJointWidget("L.KneeX",CapBody::L_KNEE_JOINT,0,capBody,layout);
    makeJointWidget("L.KneeZ",CapBody::L_KNEE_JOINT,1,capBody,layout);

    makeJointWidget("L.AnkleX",CapBody::L_ANKLE_JOINT,0,capBody,layout);
    makeJointWidget("L.AnkleY",CapBody::L_ANKLE_JOINT,1,capBody,layout);

    makeJointWidget("L.Foot",CapBody::L_FOOT_JOINT,0,capBody,layout);
    //makeJointWidget("L.Toe",CapBody::L_TOE_JOINT,0,capBody,layout);

    connect(zeroButton,SIGNAL(clicked()),this,SLOT(zeroForces()));
    connect(lightButton,SIGNAL(clicked()),this,SLOT(lightForces()));
    connect(resetButton,SIGNAL(clicked()),this,SLOT(strongForces()));

    wid->setLayout(layout);
    sa->setWidget(wid);
    dw->setWidget(sa);
    this->addDockWidget(Qt::RightDockWidgetArea,dw);
    this->tabifyDockWidget(ow,dw);

    // Body dimensions widget
    dw = new QDockWidget("Body Dims",this);
    sa = new QScrollArea;
    wid = new QWidget;
    layout = new QVBoxLayout;
    layout->setContentsMargins(0,0,0,0);
    QCheckBox* keepBodyRel = new QCheckBox("Body Rel");
    layout->addWidget(keepBodyRel);

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
    //makeBodyWidget("R.Toe Rad",CapBody::R_TOE_BODY,0,capBody,layout);
    //makeBodyWidget("R.Toe Len",CapBody::R_TOE_BODY,2,capBody,layout);
    makeBodyWidget("L.UpLeg Rad",CapBody::LUP_LEG_BODY,0,capBody,layout);
    makeBodyWidget("L.UpLeg Len",CapBody::LUP_LEG_BODY,2,capBody,layout);
    makeBodyWidget("L.LoLeg Rad",CapBody::LLO_LEG_BODY,0,capBody,layout);
    makeBodyWidget("L.LoLeg Len",CapBody::LLO_LEG_BODY,2,capBody,layout);
    makeBodyWidget("L.Heel Rad",CapBody::L_HEEL_BODY,0,capBody,layout);
    makeBodyWidget("L.Tarsal Rad",CapBody::L_TARSAL_BODY,0,capBody,layout);
    makeBodyWidget("L.Tarsal Len",CapBody::L_TARSAL_BODY,2,capBody,layout);
    //makeBodyWidget("L.Toe Rad",CapBody::L_TOE_BODY,0,capBody,layout);
    //makeBodyWidget("L.Toe Len",CapBody::L_TOE_BODY,2,capBody,layout);

    wid->setLayout(layout);
    sa->setWidget(wid);
    dw->setWidget(sa);
    this->addDockWidget(Qt::RightDockWidgetArea,dw);
    this->tabifyDockWidget(ow,dw);

    connect(ui->saveButton,SIGNAL(clicked()),capBody,SLOT(saveBody()));
    connect(ui->loadButton,SIGNAL(clicked()),capBody,SLOT(loadBody()));
    connect(ui->fromSimButton,SIGNAL(clicked()),capBody,SLOT(copyFromSim()));
    connect(ui->toSimButton,SIGNAL(clicked()),capBody,SLOT(copyToSim()));

    connect(ui->testButton,SIGNAL(clicked()),this,SLOT(experimentSlot()));


    //SwingData* md = ui->glWidget->getWorld()->getMarkerData();
    //LiveMarkerData* md = ui->glWidget->getWorld()->getMarkerData();



    //connect(ui->stepDataButton,SIGNAL(clicked()),md,SLOT(setSingleStep()));
    //connect(ui->playPauseDataButton,SIGNAL(toggled(bool)),md,SLOT(setPaused(bool)));

    connect(capBody,SIGNAL(markMap(int,int)),this,SLOT(setMarkMap(int,int)));
    connect(capBody,SIGNAL(markPoint(int,double,double,double)),
            this,SLOT(setMarkPoint(int,double,double,double)));

    /* */
    for (int ii=0;ii<41;++ii) {
      connect(markerWidgetArray[ii],SIGNAL(markBodySet(int,int)),md,SLOT(changeBodyConnect(int,int)));
      connect(markerWidgetArray[ii],SIGNAL(markConnect(int,bool)),md,SLOT(changeBodyLink(int,bool)));
      connect(markerWidgetArray[ii],SIGNAL(markPosSet(int,double,double,double)),
              md,SLOT(changeLinkPos(int,double,double,double)));
      connect(markerWidgetArray[ii],SIGNAL(markGrab(int)),this, SLOT(grabMarkPos(int)));
    }

    //ui->graphicsView->setScene(ui->glWidget->getGraphicsScene());
    //ui->graphicsView->setSceneRect(-1,4,510,8);
    //ui->graphicsView->scale(1,20);


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

    connect(ui->bigHandBox,SIGNAL(toggled(bool)),capBody,SLOT(bigHand(bool)));
    connect(ui->showMarkBox,SIGNAL(toggled(bool)),ui->glWidget,SLOT(setShowMarkers(bool)));
    connect(ui->selfCollideBox,SIGNAL(toggled(bool)),world,SLOT(setSelfCollide(bool)));

    //connect(ui->writeButton,SIGNAL(toggled(bool)),ui->glWidget->getWorld(),SLOT(writeMarks(bool)));
    //connect(ui->writeButton,SIGNAL(toggled(bool)),ui->glWidget->getWorld(),SLOT(writeModel()));

    connect(ui->angleRadio,SIGNAL(toggled(bool)),this,SLOT(useAngles(bool)));
    connect(ui->markerRadio,SIGNAL(toggled(bool)),this,SLOT(useMarkers(bool)));
    connect(ui->torqueRadio,SIGNAL(toggled(bool)),this,SLOT(useTorques(bool)));
    connect(ui->altRadio,SIGNAL(toggled(bool)),this,SLOT(useAltForces(bool)));

    ui->timeSlider->setRange(0,world->getMarkerData()->size()-1);
    //connect(world->getMarkerData(),SIGNAL(maxSize(int)),this,SLOT(setDataRange(int)));

    connect(ui->loadMarkerButton,SIGNAL(pressed()),this,SLOT(loadMarkerFile()));

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





}

MainWindow::~MainWindow()
{
  delete[] markerWidgetArray;
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

void MainWindow::clickPPA(bool play)
{
  ui->playPauseDataButton->setChecked(play);
  ui->playPauseSimButton->setChecked(play);
}

void MainWindow::clickPPS(bool play)
{
  play;
  ui->playAllButton->setChecked(false);
}

void MainWindow::clickPPD(bool play)
{
  play;
  ui->playAllButton->setChecked(false);
}

void MainWindow::playPauseAll(bool play)
{
  if (play) {
    ui->playAllButton->setIcon(QIcon(":/icons/data/media-playback-pause-3.png"));
  } else {
    ui->playAllButton->setIcon(QIcon(":/icons/data/media-playback-start-3.png"));
  }

}

void MainWindow::playPauseSim(bool play)
{
  if (play) {
    ui->playPauseSimButton->setIcon(QIcon(":/icons/data/media-playback-pause-3.png"));
  } else {
    ui->playPauseSimButton->setIcon(QIcon(":/icons/data/media-playback-start-3.png"));
  }
  ui->glWidget->getWorld()->pauseSim(!play);

}

void MainWindow::playPauseData(bool play)
{
  if (play) {
    ui->playPauseDataButton->setIcon(QIcon(":/icons/data/media-playback-pause-3.png"));
  } else {
    ui->playPauseDataButton->setIcon(QIcon(":/icons/data/media-playback-start-3.png"));
  }
  ui->glWidget->getWorld()->getMarkerData()->setPaused(play);
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
  ui->playPauseSimButton->setChecked(false);
  ui->glWidget->getWorld()->setSingleStep();

}

void MainWindow::stepData()
{
  ui->playPauseDataButton->setChecked(false);
  ui->glWidget->getWorld()->getMarkerData()->setSingleStep();
}

void MainWindow::stepAll()
{
  clickPPA(false);
  stepSim();
  stepData();
}

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

void MainWindow::setFrameTime(double step)
{
  //connect(ui->frameTimeBox,SIGNAL(valueChanged(double)),world,SLOT(setStepSize(double)));
  //connect(ui->frameTimeBox,SIGNAL(valueChanged(double)),this,SLOT(setAnimationRate(double)));

  getWorld()->setStepSize(step);
  updateTimer.start(int(1000*step));

}

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

  //body->markerToBody[mark].pos[0]=pos[0];
  //body->markerToBody[mark].pos[1]=pos[1];
  //body->markerToBody[mark].pos[2]=pos[2];
  markerWidgetArray[mark]->setMarkPos(pos[0],pos[1],pos[2]);
}


void MainWindow::updateLoop()
{
  // If processing data, advance the data frame

  // The simulation grabs the data from
  // the appropriate sequences
  // Or we pass the data to the sim.
  // If the simulation is advancing
  // Step the simulation

  //ui->graphicsView->centerOn(250,0);
  // Render data

  experimentScript->updateScript();
  ui->glWidget->animate();

}

void MainWindow::usingMarkers(bool used)
{

  for (int ii=0;ii<50;++ii) {
    markerWidgetArray[ii]->setConnect(used);
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
  }
}

void MainWindow::useTorques(bool toggle)
{
  if (toggle) {
    ui->glWidget->getWorld()->follow(2);
  }
}

void MainWindow::useAltForces(bool toggle)
{
  if (toggle) {
    ui->glWidget->getWorld()->follow(3);
  }
}

void MainWindow::setDataRange(int maxSize)
{
  ui->timeSlider->setRange(0,maxSize);
  ui->dataFrameBox->setRange(0,maxSize);
  ui->timeSlider->setValue(0);
  ui->dataFrameBox->setValue(0);

}

/**
  Throw up a file dialog so we can load in
  the new data.
  */
void MainWindow::loadMarkerFile()
{
#if defined( POKE_DATA )
  QString filename = QFileDialog::getOpenFileName(this);

  if (!filename.isNull()) {
    ui->glWidget->getWorld()->getMarkerData()->loadData(filename.toAscii());
  }
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
