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
#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QGLWidget>
#include <GL/GLU.h>
#include <QTimer>
//#include <QGraphicsScene>
#include "mygraphicsscene.h"
#include "quatcamera.h"
#include <ode/ode.h>

#include "sequence.h"

class SimWorld;
class BoardData;


class MyGLWidget : public QGLWidget
{
    Q_OBJECT
public:
    explicit MyGLWidget(QWidget *parent = 0);

  SimWorld* getWorld() { return simWorld; }

#if defined( BOARD_DATA )
  void setBoardData(BoardData* dat) {bd = dat;}
#endif
  QGraphicsScene* getGraphicsScene() { return &scene; }
signals:

public slots:
  void setDrawLines(bool);
  void setShowMarkers(bool);
  void setFollowCamera(bool);
  void setBodyAlpha(double);
  //void setAnimationRate(double);

  void animate();

protected:
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int width, int height);
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseMoveEvent(QMouseEvent *event);
  virtual void keyPressEvent(QKeyEvent* event);

  void applyGeomTransform(dGeomID gg);
  void rotateByMatrix(const dReal* R);
  void rotateByQuaternion(const dReal* Q);

  void renderGeom(dGeomID gg);
  void renderSphere(dReal radius,int sphSlice=15,int sphStack=10);
  void renderCapsule(dReal radius,dReal length);
  void renderBox(dVector3 sides);
  void renderRay(dReal length);
  void renderPlane(dVector4 planeEq);
  void renderRoom();

  void renderGeomShadow(dGeomID gg);
  void findSphereShadow(const float* light,const double* center,double radius,double shadowLength,double shadowRad[2],double tm[16],double ff[3]);
  void findCapsuleShadow(const float* light,
                         const double* center,double radius,
                         const double* rotation,double length,
                         double shadowLength,
                         double capCenter[2][3],double capRadius[2][2],
                         double capTM[2][16],
                         double bodyPoint[8][3]);
  void findBoxShadow(const float* light,const double* center,
                     const double* sides,const double* rotation,
                     double shadowLength,
                     int& caseVal,double boxPoint[2][8][3]);

  void renderSphereShadow(double shadowLength,const double center[3],const double radius[2],const double transform[16]);
  void renderCapsuleShadow(double shadowLength,
                           const double capCenter[2][3],const double capRadius[2][2],const double capTM[2][16],
                           const double bodyPoint[8][3]);
  void renderBoxShadow(const double* center, const double* rotation,
                       int caseVal,const double boxPoint[2][8][3]);


  void renderSpace(dSpaceID space);
  void renderLineList();
  void renderSimWorld(SimWorld* world);

  void renderSpaceShadows(dSpaceID space);
  void renderSimWorldShadows(SimWorld* world);

  void renderScene();

  void initializeGraphicsScene();
  void updateGraphicsScene();

  void setPathItem(const QString& label,int jj, int ax, int pp);

public:

  GLUquadric* quadric;

  double aRatio;
  double fovy;
  double nearClip,farClip;
  double angle;
  QTimer animationTimer;

  GLuint noiseTex;
  GLuint racquetTex;
  GLuint woodTex;
  GLuint cementTex;
  GLuint ceilingTex;

  SimWorld* simWorld;

#if defined( BOARD_DATA )
  BoardData* bd;
#endif
  //QGraphicsScene scene;
  MyGraphicsScene scene;
  QuatCamera camera;
  static float lightPos[4];
  static float lightAmb[4];
  static float lightDif[4];
  static float lightSpc[4];
  static float lightZro[4];

  GLuint frontList;
  GLuint shadowList;

  //QGraphicsLineItem* sceneItem[500];

  QGraphicsPathItem* gpItem[11];
  QGraphicsTextItem* textItem[11];

  bool drawLines;
  bool followCamera;
  bool showMarkers;

  QVector3D offset;

  double bodyAlpha;



};

#endif // MYGLWIDGET_H
