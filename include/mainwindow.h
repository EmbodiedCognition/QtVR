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
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <vector>

class MarkerWidget;
class JointWidget;
class CapBody;
class QVBoxLayout;
class BoardData;
class QPushButton;
class SimWorld;
class ScriptBase;

namespace Ui {
    class MainWindow;
}

/**
  The main GUI window.  In windowed mode, this
  provides a docking place for the openGL window
  as well as GUI elements for setting various
  modes and controlling the flow of information.
  */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

  void makeJointWidget(QString label,int joint,
                       int axis,CapBody* cb,
                       QVBoxLayout* lay);
  void populateJointTab(CapBody* capBody,
                        QVBoxLayout* layout);
  void makeBodyWidget(QString label,int joint,
                      int axis,CapBody* cb,
                      QVBoxLayout* lay);
  void populateBodyTab(CapBody* capBody,
                       QVBoxLayout* layout);
  SimWorld* getWorld();

  void writeVMarkerFrame(int cc,int tri,FILE* file);
  void writeAngleFrame(int cc,int tri,FILE* file);


public slots:
  void toggleFullscreenSlot();

  void playPauseAll(bool);
  void playPauseSim(bool);
  void playPauseData(bool);
  void updatePlayButtons();

  void setMarkMap(int id,int bod);
  void setMarkPoint(int id,double xx,double yy,double zz);

  void stepSim();
  void stepData();
  void stepAll();

  void setDataFrame(int);
  void setDataStep(int);
  void setFrameTime(double);

  void grabMarkPos(int);

  void updateLoop();

  void usingMarkers(bool used);
  void connectMarkers();
  void releaseMarkers();
  void updateMarkerAnchors();

  void useAngles(bool toggle);
  void useMarkers(bool toggle);
  void useTorques(bool toggle);
  void useAltForces(bool toggle);

  void setDataRange(int maxSize);
  void loadMarkerFile(QString filename);


  void experimentSlot();
  void saveModel();
  void restoreModel();
  void clearData();

  void strongForces();
  void lightForces();
  void zeroForces();

  void saveBodyAngles(QString filename);

  void useGlobalForces(bool use);

  void markerFileDialog();


public:
    Ui::MainWindow *ui;  ///< Holds the widgets created in the designer
    /// An array of widgets for controlling marker attachments
    std::vector<MarkerWidget*> markerWidgetArray;

#if defined( BOARD_DATA )
    BoardData* bd;
#endif

    /// Driver code for running a specific process to get kinematic or dynamic data
    ScriptBase* experimentScript;

    /// Timer object controls the main update loop
    QTimer updateTimer;

    /// An array of widgets controlling individual joint degrees of freedom
    QList<JointWidget*> jointWidgets;
};

#endif // MAINWINDOW_H
