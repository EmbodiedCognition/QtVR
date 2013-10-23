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
#include <GL/glu.h>
#include <QTimer>
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

  SimWorld* getWorld() { return sim_world; }

#if defined( BOARD_DATA )
  void setBoardData(BoardData* dat) {bd = dat;}
#endif
signals:

public slots:
  void setDrawLines(bool);
  void setShowMarkers(bool);
  void setFollowCamera(bool);
  void setBodyAlpha(double);
  void animate();

protected:
  virtual void initializeGL();
  virtual void paintGL();
  virtual void resizeGL(int width, int height);
  virtual void mousePressEvent(QMouseEvent *event);
  virtual void mouseMoveEvent(QMouseEvent *event);
  virtual void mouseReleaseEvent(QMouseEvent *event);
  virtual void wheelEvent(QWheelEvent * event);
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

  double aspect_ratio;
  double fovy;
  double near_clip,far_clip;
  double angle;
  QTimer animationTimer;

  GLuint noise_texture;
  GLuint racquet_texture;
  GLuint wood_texture;
  GLuint cement_texture;
  GLuint ceiling_texture;

  SimWorld* sim_world;

#if defined( BOARD_DATA )
  BoardData* bd;
#endif
  QuatCamera camera; ///< Quaternion based camera
  QPoint lastPos;    ///< Tracks mouse changes for camera control
  static float light_pos[4];
  static float light_ambient[4];
  static float light_diffuse[4];
  static float light_spectral[4];
  static float light_zro[4];

  GLuint front_list;
  GLuint shadow_list;

  bool draw_lines;
  bool follow_camera;
  bool show_markers;

  QVector3D offset;

  double body_alpha;
};

#endif // MYGLWIDGET_H
