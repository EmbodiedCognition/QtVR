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
#include "myglwidget.h"

#include <QMouseEvent>
#include <QGraphicsLineItem>
#include <QImage>


#include "simworld.h"
#include "CapBody.h"
#include "boarddata.h"

float MyGLWidget::light_pos[4] = {3, 2, 8, 1};
float MyGLWidget::light_ambient[4] = {.3f,.3f,.3f,1};
float MyGLWidget::light_diffuse[4] = {1.0f,1.0f,1.0f,1};
float MyGLWidget::light_spectral[4] = {.8f,.8f,.8f,1};
float MyGLWidget::light_zro[4] = {0};

MyGLWidget::MyGLWidget(QWidget *parent) :
    QGLWidget(parent)
{
  fovy     = 40;
  near_clip = .25;
  far_clip  = 50;
  angle = 0;

  //animationTimer.setSingleShot(false);
  //connect(&animationTimer,SIGNAL(timeout()),this,SLOT(animate()));
  //animationTimer.start(15);

  camera.translateRelative(QVector3D(0,0,10));

  sim_world = new SimWorld(this);
  std::cout << "SimWorld object created" << std::endl;
  setFocusPolicy(Qt::StrongFocus);
  initializeGraphicsScene();
  draw_lines=false;
  follow_camera=false;
  show_markers=true;

#if defined( BOARD_DATA )
  bd=NULL;
#endif

  body_alpha = 0.5;
}

void MyGLWidget::setDrawLines(bool should)
{
  draw_lines=should;
}

void MyGLWidget::setShowMarkers(bool should)
{
  show_markers=should;
}

void MyGLWidget::setFollowCamera(bool should)
{
  follow_camera=should;
  if (should) {
    dBodyID waist = sim_world->getBody()->body_segments[CapBody::WAIST_BODY];
    const dReal* pos = dBodyGetPosition(waist);
    QVector3D oldPos = camera.getPosition();
    offset.setX(oldPos.x()-pos[0]);
    offset.setY(oldPos.y()-pos[1]);
  }
}

void MyGLWidget::setBodyAlpha(double alpha)
{
  if (alpha<0) alpha = 0;
  if (alpha>1) alpha = 1;

  body_alpha = alpha;
}

//void MyGLWidget::setAnimationRate(double rate)
//{
  //animationTimer.start(int(rate*1000));
//}

void MyGLWidget::initializeGL()
{
  glClearColor(0.0f, 0.0f, 0.0f, 0.0f);			// Black Background
  glClearDepth(1.0f);	  				// Depth Buffer Setup
  glDepthFunc(GL_LEQUAL);				// Type Of Depth Testing
  glEnable(GL_DEPTH_TEST);				// Enable Depth Testing
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_BLEND);  					// Enable Blending
  glEnable(GL_TEXTURE_2D);				// Enable Texture Mapping
  glEnable(GL_CULL_FACE);				// Remove Back Faces
  glEnable(GL_NORMALIZE);
  glEnable(GL_COLOR_MATERIAL);  // Use glColor to color things
  // We place the light during the render so that it doesn't move
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);



  glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);		// Set Ambient light
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);		// Set Diffuse
  glLightfv(GL_LIGHT0, GL_SPECULAR, light_spectral);		// Set Specular

  //glEnable(GL_POLYGON_SMOOTH);                          // Anti-alias polygons
  glHint(GL_POLYGON_SMOOTH_HINT,GL_NICEST);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);


  QImage img(QString("data/seamlessNoise.png"));
  noise_texture = bindTexture(img,GL_TEXTURE_2D,GL_RGB,QGLContext::DefaultBindOption);
  racquet_texture = bindTexture(QImage(QString("data/racquetTex.png")),GL_TEXTURE_2D,GL_RGBA,QGLContext::DefaultBindOption);
  wood_texture = bindTexture(QImage(QString("data/wood.bmp")),GL_TEXTURE_2D,GL_RGB,QGLContext::DefaultBindOption);
  cement_texture = bindTexture(QImage(QString("data/concrete.bmp")),GL_TEXTURE_2D,GL_RGB,QGLContext::DefaultBindOption);
  ceiling_texture = bindTexture(QImage(QString("data/ceiling.bmp")),GL_TEXTURE_2D,GL_RGB,QGLContext::DefaultBindOption);

  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
  glEnable (GL_TEXTURE_GEN_S);
  glEnable (GL_TEXTURE_GEN_T);
  glTexGeni (GL_S,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
  glTexGeni (GL_T,GL_TEXTURE_GEN_MODE,GL_OBJECT_LINEAR);
  //static GLfloat s_params[4] = {1.0f,1.0f,0.0f,1};
  //static GLfloat t_params[4] = {0.817f,-0.817f,0.817f,1};
  //glTexGenfv (GL_S,GL_OBJECT_PLANE,s_params);
  //glTexGenfv (GL_T,GL_OBJECT_PLANE,t_params);
  glTexEnvf( GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE );



  quadric = gluNewQuadric();

  front_list = glGenLists(1);
  shadow_list = glGenLists(1);
}

void MyGLWidget::paintGL()
{
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  renderScene();
  /*
  glMatrixMode( GL_MODELVIEW );
  glLoadIdentity();
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

  glPushMatrix();
  //glTranslated(0,0,-20);
  double mat[16];
  camera.getCMajorInvTrans(mat);
  glMultMatrixd(mat);

  // Place the light _after_ moving the eye

  glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

  renderSimWorld(simWorld);

  glPopMatrix();*/
}

void MyGLWidget::animate()
{
  sim_world->step();

#if defined( BOARD_DATA )
  bd->step();
#endif

  //this->updateGraphicsScene();

  if (follow_camera) {
    dBodyID waist = sim_world->getBody()->body_segments[CapBody::WAIST_BODY];
    const dReal* position = dBodyGetPosition(waist);
    QVector3D old_position = camera.getPosition();

    camera.setPosition(QVector3D(position[0]+offset.x(),
                                 position[1]+offset.y(),
                                 old_position.z()));
  }
  this->updateGL();
}

void MyGLWidget::resizeGL(int width, int height)
{
  if ( height == 0 ) height = 1;
  aspect_ratio = width / double(height);

  glViewport( 0, 0, ( GLint )width, ( GLint )height );
  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
  gluPerspective( fovy, aspect_ratio, near_clip, far_clip );

}

void MyGLWidget::mousePressEvent(QMouseEvent *event)
{
  event->ignore();
}

void MyGLWidget::mouseMoveEvent(QMouseEvent *event)
{
  event->ignore();
}

void MyGLWidget::keyPressEvent(QKeyEvent* event)
{
  double trans=0.1;
  double rot = 5.0; // In degrees.
  if (event->modifiers()&Qt::ControlModifier) {
    switch(event->key()) {
      case Qt::Key_1:
        camera.translateRelative(QVector3D(-trans,0,0));
      break;
      case Qt::Key_2:
        camera.rotateRelative(QQuaternion::fromAxisAndAngle(1,0,0,-rot));
      break;
      case Qt::Key_3:
        camera.translateRelative(QVector3D( trans,0,0));
      break;
      case Qt::Key_4:
        camera.rotateRelative(QQuaternion::fromAxisAndAngle(0,1,0,rot));
      break;

      case Qt::Key_5:
        camera.reset();
        camera.translateRelative(QVector3D(0,0,0));
      break;
      case Qt::Key_6:
        camera.rotateRelative(QQuaternion::fromAxisAndAngle(0,1,0,-rot));
      break;
      case Qt::Key_7:
        camera.rotateRelative(QQuaternion::fromAxisAndAngle(0,0,1,rot));
      break;
      case Qt::Key_8:
        camera.rotateRelative(QQuaternion::fromAxisAndAngle(1,0,0,rot));
      break;
      case Qt::Key_9:
        camera.rotateRelative(QQuaternion::fromAxisAndAngle(0,0,1,-rot));
      break;
      case Qt::Key_Plus:
        camera.translateRelative(QVector3D( 0,-trans,0));
      break;
      case Qt::Key_Minus:
        camera.translateRelative(QVector3D( 0,trans,0));
      break;
      case Qt::Key_Asterisk:
        camera.translateRelative(QVector3D( 0,0,-trans));
      break;
      case Qt::Key_Slash:
        camera.translateRelative(QVector3D( 0,0,trans));
      break;



    }
  }

  event->accept();
}

///////////////////////////////////////////////////////////////////////

void MyGLWidget::applyGeomTransform(dGeomID gg)
{
  const dReal* pos = dGeomGetPosition( gg );
  const dReal* R   = dGeomGetRotation( gg );

  GLdouble matrix[16];
  matrix[0]  = R[0];
  matrix[1]  = R[4];
  matrix[2]  = R[8];
  matrix[3]  = 0;
  matrix[4]  = R[1];
  matrix[5]  = R[5];
  matrix[6]  = R[9];
  matrix[7]  = 0;
  matrix[8]  = R[2];
  matrix[9]  = R[6];
  matrix[10] = R[10];
  matrix[11] = 0;
  matrix[12] = pos[0];
  matrix[13] = pos[1];
  matrix[14] = pos[2];
  matrix[15] = 1;

  glMultMatrixd(matrix);
}

void MyGLWidget::rotateByMatrix(const dReal* R)
{
  GLdouble matrix[16];
  matrix[0]  = R[0];
  matrix[1]  = R[4];
  matrix[2]  = R[8];
  matrix[3]  = 0;
  matrix[4]  = R[1];
  matrix[5]  = R[5];
  matrix[6]  = R[9];
  matrix[7]  = 0;
  matrix[8]  = R[2];
  matrix[9]  = R[6];
  matrix[10] = R[10];
  matrix[11] = 0;
  matrix[12] = 0;
  matrix[13] = 0;
  matrix[14] = 0;
  matrix[15] = 1;

  glMultMatrixd(matrix);
}

void MyGLWidget::rotateByQuaternion(const dReal* Q)
{
  dMatrix3 R;

  dRfromQ(R,Q);
  rotateByMatrix(R);

}

void MyGLWidget::renderGeom(dGeomID gg)
{
  int geometry_class = dGeomGetClass( gg );

  /*if (mouseSet.count(gg)>0) {
    glColor3f(1,0,0);
  } else {
    glColor3f(1,1,1);
  }*/
  switch (geometry_class) {
    case dSphereClass:
      {
        glPushMatrix();
        applyGeomTransform(gg);
        dReal radius = dGeomSphereGetRadius( gg );
        glColor4d(1,.7,.2,body_alpha);
        renderSphere(radius);
        glPopMatrix();
      } break;
    case dBoxClass:
      {
        glPushMatrix();
        applyGeomTransform(gg);
        dVector3 sides;
        dGeomBoxGetLengths( gg,sides );
        glColor4d(1,.7,.2,body_alpha);
        renderBox(sides);
        glPopMatrix();
      } break;
    case dCapsuleClass:
      {
        glPushMatrix();
        applyGeomTransform(gg);
        dReal radius, length;
        dGeomCapsuleGetParams( gg ,&radius,&length );
        glColor4d(1,.7,.2,body_alpha);
        renderCapsule(radius,length);
        glPopMatrix();
      } break;
    case dRayClass:
      {
        glPushMatrix();
        applyGeomTransform(gg);
        dReal length;
        length = dGeomRayGetLength( gg );
        glColor3d(1,1,1);
        renderRay( length );
        glPopMatrix();
      } break;
    case dPlaneClass:
      {
        // A plane is not 'placeable' so we can't get its position.
        //dVector4 planeEq;
        //dGeomPlaneGetParams(gg,planeEq);
        //glColor3d(1,1,1);
        //renderPlane(planeEq);
      } break;
    default:
      // We're not dealing with height fields,
      // meshes, cylinders, or spaces.
      break;
  }
}


void MyGLWidget::renderSphere(dReal radius,int sphSlice,int sphStack)
{
  gluSphere(quadric,radius,sphSlice,sphStack);
}

void MyGLWidget::renderCapsule(dReal radius,dReal length)
{
  int capped_cylinder_quality = 3;

  int i,j;
  double tmp,nx,ny,nz,start_nx,start_ny,a,ca,sa;
  // number of sides to the cylinder (divisible by 4):
  const int n = capped_cylinder_quality*4;

  length *= 0.5;
  a = M_PI*2.0/n;
  sa =  sin(a);
  ca =  cos(a);

  // draw cylinder body
  ny=1; nz=0;		  // normal vector = (0,ny,nz)
  glBegin (GL_TRIANGLE_STRIP);
  for (i=0; i<=n; i++) {
    glNormal3d (ny,nz,0);
    glVertex3d (ny*radius,nz*radius,length);
    glNormal3d (ny,nz,0);
    glVertex3d (ny*radius,nz*radius,-length);
    // rotate ny,nz
    tmp = ca*ny - sa*nz;
    nz = sa*ny + ca*nz;
    ny = tmp;
  }
  glEnd();

  // draw first cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    double start_nx2 =  ca*start_nx + sa*start_ny;
    double start_ny2 = -sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    double nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*radius,nz2*radius,length+nx2*radius);
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*radius,nz*radius,length+nx*radius);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }

  // draw second cylinder cap
  start_nx = 0;
  start_ny = 1;
  for (j=0; j<(n/4); j++) {
    // get start_n2 = rotated start_n
    double start_nx2 = ca*start_nx - sa*start_ny;
    double start_ny2 = sa*start_nx + ca*start_ny;
    // get n=start_n and n2=start_n2
    nx = start_nx; ny = start_ny; nz = 0;
    double nx2 = start_nx2, ny2 = start_ny2, nz2 = 0;
    glBegin (GL_TRIANGLE_STRIP);
    for (i=0; i<=n; i++) {
      glNormal3d (ny,nz,nx);
      glVertex3d (ny*radius,nz*radius,-length+nx*radius);
      glNormal3d (ny2,nz2,nx2);
      glVertex3d (ny2*radius,nz2*radius,-length+nx2*radius);
      // rotate n,n2
      tmp = ca*ny - sa*nz;
      nz = sa*ny + ca*nz;
      ny = tmp;
      tmp = ca*ny2- sa*nz2;
      nz2 = sa*ny2 + ca*nz2;
      ny2 = tmp;
    }
    glEnd();
    start_nx = start_nx2;
    start_ny = start_ny2;
  }
}

void MyGLWidget::renderBox(dVector3 sides)
{
  double lx = sides[0]*0.5f;
  double ly = sides[1]*0.5f;
  double lz = sides[2]*0.5f;

  // sides
  glBegin (GL_QUADS);
  glNormal3d (-1,0,0);
  glVertex3d (-lx,-ly,-lz);
  glVertex3d (-lx,-ly, lz);
  glVertex3d (-lx, ly, lz);
  glVertex3d (-lx, ly,-lz);

  glNormal3d (0,-1,0);
  glVertex3d (-lx,-ly,-lz);
  glVertex3d ( lx,-ly,-lz);
  glVertex3d ( lx,-ly, lz);
  glVertex3d (-lx,-ly, lz);

  glNormal3d (0,0,-1);
  glVertex3d (-lx,-ly,-lz);
  glVertex3d (-lx, ly,-lz);
  glVertex3d ( lx, ly,-lz);
  glVertex3d ( lx,-ly,-lz);

  glNormal3d ( 1,0,0);
  glVertex3d ( lx, ly, lz);
  glVertex3d ( lx,-ly, lz);
  glVertex3d ( lx,-ly,-lz);
  glVertex3d ( lx, ly,-lz);

  glNormal3d (0,1,0);
  glVertex3d ( lx, ly, lz);
  glVertex3d ( lx, ly,-lz);
  glVertex3d (-lx, ly,-lz);
  glVertex3d (-lx, ly, lz);

  glNormal3d (0,0,1);
  glVertex3d ( lx, ly, lz);
  glVertex3d (-lx, ly, lz);
  glVertex3d (-lx,-ly, lz);
  glVertex3d ( lx,-ly, lz);
  glEnd();
}

void MyGLWidget::renderRay(dReal length)
{
  glBegin(GL_LINES);
  glVertex3d(0,0,0);
  glVertex3d(0,0,length);
  glEnd();
}

/**
  Plane is determined by a normal and a distance
  from the origin along that normal.
*/
void MyGLWidget::renderPlane(dVector4 planeEq)
{
  // planeEq[0-2] are the new z axis
  // We'll define the x reference axis
  // as close to the first
  // smallest value of those
  dVector3 xAxis, yAxis, zAxis;
  zAxis[0]=planeEq[0];
  zAxis[1]=planeEq[1];
  zAxis[2]=planeEq[2];
  dSafeNormalize3(zAxis);

  int smallAxis=0;
  if (zAxis[1]<zAxis[smallAxis]) smallAxis=1;
  if (zAxis[2]<zAxis[smallAxis]) smallAxis=2;

  xAxis[0]=xAxis[1]=xAxis[2]=0;
  xAxis[smallAxis]=1;

  dCROSS(yAxis,=,zAxis,xAxis);
  dSafeNormalize3(yAxis);
  dCROSS(xAxis,=,yAxis,zAxis);
  dSafeNormalize3(xAxis);

  GLdouble matrix[16];
  matrix[0]  = xAxis[0];
  matrix[1]  = xAxis[1];
  matrix[2]  = xAxis[2];
  matrix[3]  = 0;
  matrix[4]  = yAxis[0];
  matrix[5]  = yAxis[1];
  matrix[6]  = yAxis[2];
  matrix[7]  = 0;
  matrix[8]  = zAxis[0];
  matrix[9]  = zAxis[1];
  matrix[10] = zAxis[2];
  matrix[11] = 0;
  matrix[12] = zAxis[0]*planeEq[3];
  matrix[13] = zAxis[1]*planeEq[3];
  matrix[14] = zAxis[2]*planeEq[3];
  matrix[15] = 1;
  glPushMatrix();
  glMultMatrixd(matrix);
  gluDisk(quadric,0,40,30,2);
  glPopMatrix();
}

void MyGLWidget::renderRoom()
{

  glBindTexture(GL_TEXTURE_2D,wood_texture);
  glColor3d(1,1,1);
  glDisable(GL_TEXTURE_GEN_S);
  glDisable(GL_TEXTURE_GEN_T);
  //glDisable(GL_LIGHTING);
  glBegin (GL_QUADS);
    glColor3f(1,1,1);
    glNormal3f(0,0,1);
    glTexCoord2d(0,0);
    glVertex3f(-3, 9,0);
    glTexCoord2d(4,0);
    glVertex3f(-3,-3,0);
    glTexCoord2d(4,2);
    glVertex3f( 3,-3,0);
    glTexCoord2d(0,2);
    glVertex3f( 3, 9,0);
  glEnd();

  glBindTexture(GL_TEXTURE_2D,ceiling_texture);
  glBegin(GL_QUADS);
    glColor3f(1,1,1);
    glNormal3f(0,0,-1);
    glTexCoord2d(0,0);
    glVertex3f(-3, 9,6);
    glTexCoord2d(0,1);
    glVertex3f( 3, 9,6);
    glTexCoord2d(1,1);
    glVertex3f( 3,-3,6);
    glTexCoord2d(1,0);
    glVertex3f(-3,-3,6);
  glEnd();

  glBindTexture(GL_TEXTURE_2D,cement_texture);
  glBegin(GL_QUADS);
  glNormal3f(1,0,0);
  glTexCoord2d(6,0);
  glVertex3f(-3, 9,0);
  glTexCoord2d(6,3);
  glVertex3f(-3, 9,6);
  glTexCoord2d(0,3);
  glVertex3f(-3,-3,6);
  glTexCoord2d(0,0);
  glVertex3f(-3,-3,0);

  glNormal3f(-1,0,0);
  glTexCoord2d(6,0);
  glVertex3f(3, 9,0);
  glTexCoord2d(0,0);
  glVertex3f(3,-3,0);
  glTexCoord2d(0,3);
  glVertex3f(3,-3,6);
  glTexCoord2d(6,3);
  glVertex3f(3, 9,6);

  glNormal3f(0,1,0);
  glTexCoord2d(0,0);
  glVertex3f(3,-3,0);
  glTexCoord2d(3,0);
  glVertex3f(-3,-3,0);
  glTexCoord2d(3,3);
  glVertex3f(-3,-3,6);
  glTexCoord2d(0,3);
  glVertex3f(3,-3,6);

  glNormal3f(0,-1,0);
  glTexCoord2d(0,0);
  glVertex3f(3,9,0);
  glTexCoord2d(0,3);
  glVertex3f(3,9,6);
  glTexCoord2d(3,3);
  glVertex3f(-3,9,6);
  glTexCoord2d(3,0);
  glVertex3f(-3,9,0);
  glEnd();

  //glEnable(GL_LIGHTING);
  glEnable(GL_TEXTURE_GEN_S);
  glEnable(GL_TEXTURE_GEN_T);
}

////////////////////////////////////////////////////////////////////
void MyGLWidget::renderGeomShadow(dGeomID gg)
{
  switch (dGeomGetClass( gg )) {
    case dSphereClass:
      {
        double shadowRad[2];
        double tm[16];
        double tmp[3]; // This isn't used here.  It's used in the capsule.

        dReal radius = dGeomSphereGetRadius( gg );
        const dReal* pos = dGeomGetPosition(gg);
        findSphereShadow(light_pos,pos,
          radius,25,shadowRad,tm,tmp);
        renderSphereShadow(25,pos,&shadowRad[0],tm);
      } break;
    case dBoxClass:
      {
        dVector3 sides;
        const dReal* pos = dGeomGetPosition(gg);
        const dReal* rotation = dGeomGetRotation(gg);
        int caseVal;
        double boxPoint[2][8][3];

        dGeomBoxGetLengths( gg,sides );
        findBoxShadow(light_pos,pos,sides,
          rotation,25,caseVal,boxPoint);
        renderBoxShadow(pos,rotation,caseVal,boxPoint);
      }	break;
    case dCapsuleClass:
      {
        dReal radius, length;
        dGeomCapsuleGetParams( gg ,&radius,&length );
        const dReal* pos = dGeomGetPosition(gg);
        const dReal* rotation = dGeomGetRotation(gg);
        double capCenter[2][3];
        double capRadius[2][2];
        double capTM[2][16];
        double bodyPoint[8][3];

        findCapsuleShadow(light_pos,pos,radius,rotation,
          length,25,capCenter,capRadius,capTM,bodyPoint);
        renderCapsuleShadow(25,capCenter,capRadius,capTM,bodyPoint);
      }	break;
    case dRayClass:
      break;
    case dPlaneClass:
      break;
    default:
      break;
  }
}


void MyGLWidget::findSphereShadow(const float* light,const double* center,double radius,double shadowLength,double shadowRad[2],double tm[16],double ff[3])
{
  double tx = center[0] - light[0];
  double ty = center[1] - light[1];
  double tz = center[2] - light[2];
  double dist = sqrt(tx*tx + ty*ty + tz*tz);
  double theta = asin(radius/dist);
  //double ff[3];

  shadowRad[0] = tan(theta)*dist;
  shadowRad[1] = tan(theta)*(dist+shadowLength);

  double ss[3];
  double uu[3];

  ff[0] = tx/dist;
  ff[1] = ty/dist;
  ff[2] = tz/dist;

  uu[0] = 0;
  uu[1] = 1;
  uu[2] = 0;

  ss[0] =  ff[1]*uu[2] - ff[2]*uu[1];
  ss[1] = -ff[0]*uu[2] + ff[2]*uu[0];
  ss[2] =  ff[0]*uu[1] - ff[1]*uu[0];

  double len = sqrt(ss[0]*ss[0] + ss[1]*ss[1] + ss[2]*ss[2]);
  ss[0] /= len;
  ss[1] /= len;
  ss[2] /= len;

  uu[0] = -ss[1]*ff[2] + ss[2]*ff[1];
  uu[1] =  ss[0]*ff[2] - ss[2]*ff[0];
  uu[2] = -ss[0]*ff[1] + ss[1]*ff[0];

  tm[0] = ss[0];
  tm[1] = ss[1];
  tm[2] = ss[2];
  tm[3] = 0;

  tm[4] = uu[0];
  tm[5] = uu[1];
  tm[6] = uu[2];
  tm[7] = 0;

  tm[8] = ff[0];
  tm[9] = ff[1];
  tm[10] = ff[2];
  tm[11] = 0;

  tm[12] = 0;
  tm[13] = 0;
  tm[14] = 0;
  tm[15] = 1;

}

void MyGLWidget::findCapsuleShadow(const float* light,
                            const double* center,double radius,
                            const double* rotation,double length,
                            double shadowLength,
                            double capCenter[2][3],double capRadius[2][2],
                            double capTM[2][16],
                            double bodyPoint[8][3])
{
  double halfLength = length/2;
  double ff[2][3];
  double alNorm[3];
  double alLen;

  // Center of first cap
  capCenter[0][0] = center[0] + rotation[2]*halfLength;
  capCenter[0][1] = center[1] + rotation[6]*halfLength;
  capCenter[0][2] = center[2] + rotation[10]*halfLength;

  // Center of other cap
  capCenter[1][0] = center[0] - rotation[2]*halfLength;
  capCenter[1][1] = center[1] - rotation[6]*halfLength;
  capCenter[1][2] = center[2] - rotation[10]*halfLength;

  // It would be nice if we could just calculate the sphere half that we need.
  // But I don't know how to do that.
  findSphereShadow(light,&capCenter[0][0],radius,shadowLength,&capRadius[0][0],&capTM[0][0],&ff[0][0]);
  findSphereShadow(light,&capCenter[1][0],radius,shadowLength,&capRadius[1][0],&capTM[1][0],&ff[1][0]);

  // Find the angle between the main axis and the light.
  alNorm[0] = ff[0][1]*rotation[10]-ff[0][2]*rotation[6];
  alNorm[1] = -ff[0][0]*rotation[10]+ff[0][2]*rotation[2];
  alNorm[2] = ff[0][0]*rotation[6]-ff[0][1]*rotation[2];
  alLen = sqrt(alNorm[0]*alNorm[0] + alNorm[1]*alNorm[1] + alNorm[2]*alNorm[2]);
  alNorm[0]/=alLen;
  alNorm[1]/=alLen;
  alNorm[2]/=alLen;

  // Find the vertices of the hexahedron that projects from the
  // cylindrical portion of the capsule.
  bodyPoint[0][0] = capCenter[0][0] + capRadius[0][0]*alNorm[0];
  bodyPoint[0][1] = capCenter[0][1] + capRadius[0][0]*alNorm[1];
  bodyPoint[0][2] = capCenter[0][2] + capRadius[0][0]*alNorm[2];
  bodyPoint[1][0] = capCenter[0][0] - capRadius[0][0]*alNorm[0];
  bodyPoint[1][1] = capCenter[0][1] - capRadius[0][0]*alNorm[1];
  bodyPoint[1][2] = capCenter[0][2] - capRadius[0][0]*alNorm[2];

  bodyPoint[4][0] = capCenter[0][0] + ff[0][0]*shadowLength + capRadius[0][1]*alNorm[0];
  bodyPoint[4][1] = capCenter[0][1] + ff[0][1]*shadowLength + capRadius[0][1]*alNorm[1];
  bodyPoint[4][2] = capCenter[0][2] + ff[0][2]*shadowLength + capRadius[0][1]*alNorm[2];
  bodyPoint[5][0] = capCenter[0][0] + ff[0][0]*shadowLength - capRadius[0][1]*alNorm[0];
  bodyPoint[5][1] = capCenter[0][1] + ff[0][1]*shadowLength - capRadius[0][1]*alNorm[1];
  bodyPoint[5][2] = capCenter[0][2] + ff[0][2]*shadowLength - capRadius[0][1]*alNorm[2];

  bodyPoint[2][0] = capCenter[1][0] + capRadius[1][0]*alNorm[0];
  bodyPoint[2][1] = capCenter[1][1] + capRadius[1][0]*alNorm[1];
  bodyPoint[2][2] = capCenter[1][2] + capRadius[1][0]*alNorm[2];
  bodyPoint[3][0] = capCenter[1][0] - capRadius[1][0]*alNorm[0];
  bodyPoint[3][1] = capCenter[1][1] - capRadius[1][0]*alNorm[1];
  bodyPoint[3][2] = capCenter[1][2] - capRadius[1][0]*alNorm[2];

  bodyPoint[6][0] = capCenter[1][0] + ff[1][0]*shadowLength + capRadius[1][1]*alNorm[0];
  bodyPoint[6][1] = capCenter[1][1] + ff[1][1]*shadowLength + capRadius[1][1]*alNorm[1];
  bodyPoint[6][2] = capCenter[1][2] + ff[1][2]*shadowLength + capRadius[1][1]*alNorm[2];
  bodyPoint[7][0] = capCenter[1][0] + ff[1][0]*shadowLength - capRadius[1][1]*alNorm[0];
  bodyPoint[7][1] = capCenter[1][1] + ff[1][1]*shadowLength - capRadius[1][1]*alNorm[1];
  bodyPoint[7][2] = capCenter[1][2] + ff[1][2]*shadowLength - capRadius[1][1]*alNorm[2];
}

/**
  The idea is to take every face and compare its normal
  with the vector to the light.  We take those faces with
  a positive dot product and form the shadow volume from
  that.  We consider every pair of neighbor vertices.  If
  the two faces that form the edge have one that faces the
  light and one not facing the light, then we extrude that
  edge away from the light.
*/
void MyGLWidget::findBoxShadow(const float* light,const double* center,
                        const double* sides,const double* rotation,
                        double shadowLength,
                        int& caseVal,double boxPoint[2][8][3])
{
  double sx = sides[0]/2;
  double sy = sides[1]/2;
  double sz = sides[2]/2;

  //Transform the light into the box-centric frame of reference:
  double rotLight[3];
  double tmpLight[3];
  tmpLight[0] = light[0] - center[0];
  tmpLight[1] = light[1] - center[1];
  tmpLight[2] = light[2] - center[2];
  rotLight[0] = rotation[0]*tmpLight[0] + rotation[4]*tmpLight[1] + rotation[8]*tmpLight[2];
  rotLight[1] = rotation[1]*tmpLight[0] + rotation[5]*tmpLight[1] + rotation[9]*tmpLight[2];
  rotLight[2] = rotation[2]*tmpLight[0] + rotation[6]*tmpLight[1] + rotation[10]*tmpLight[2];

  // The axis aligned box divides space into 27 convenient volumes.
  // We just figure out which of those we're in.
  char faceCase[3];

  if (rotLight[0]>sx) {
    faceCase[0] =  1;
  } else if (rotLight[0]<-sx) {
    faceCase[0] = -1;
  } else {
    faceCase[0] =  0;
  }

  if (rotLight[1]>sy) {
    faceCase[1] =  1;
  } else if (rotLight[1]<-sy) {
    faceCase[1] = -1;
  } else {
    faceCase[1] =  0;
  }

  if (rotLight[2]>sz) {
    faceCase[2] =  1;
  } else if (rotLight[2]<-sz) {
    faceCase[2] = -1;
  } else {
    faceCase[2] =  0;
  }

  // Map from a trinary number to decimal.
  caseVal = (faceCase[2]+1)*9 + (faceCase[1]+1)*3 + (faceCase[0]+1);

  // Extrude all 8 points.
  // This time we treat the space as binary.
  double tmpVec[3];
  double tmpLen;

  for (int cc = 0;cc<8;++cc) {
    boxPoint[0][cc][0] = (cc&1)?sx:-sx;
    boxPoint[0][cc][1] = (cc&2)?sy:-sy;
    boxPoint[0][cc][2] = (cc&4)?sz:-sz;
    tmpVec[0] = boxPoint[0][cc][0] - rotLight[0];
    tmpVec[1] = boxPoint[0][cc][1] - rotLight[1];
    tmpVec[2] = boxPoint[0][cc][2] - rotLight[2];
    tmpLen    = sqrt(tmpVec[0]*tmpVec[0] + tmpVec[1]*tmpVec[1] + tmpVec[2]*tmpVec[2]);
    tmpVec[0] /= tmpLen;
    tmpVec[1] /= tmpLen;
    tmpVec[2] /= tmpLen;
    boxPoint[1][cc][0] = boxPoint[0][cc][0] + shadowLength*tmpVec[0];
    boxPoint[1][cc][1] = boxPoint[0][cc][1] + shadowLength*tmpVec[1];
    boxPoint[1][cc][2] = boxPoint[0][cc][2] + shadowLength*tmpVec[2];
  }
}

//////
void MyGLWidget::renderSphereShadow(double shadowLength,const double center[3],const double radius[2],const double transform[16])
{
  // The shadow cone
  glPushMatrix();
  glTranslated(center[0],center[1],center[2]);
  glMultMatrixd(transform);
  gluQuadricOrientation(quadric,GLU_INSIDE);
  gluDisk(quadric,0,radius[0],15,3);
  gluQuadricOrientation(quadric,GLU_OUTSIDE);
  gluCylinder(quadric,radius[0],radius[1],shadowLength,15,1);
  //glTranslatef(0,0,shadowLength);
  // ... only need near disk.  The far is always behind a wall.
  //gluDisk(quadric,0,radius[1],10,3);
  glPopMatrix();
}


void MyGLWidget::renderCapsuleShadow(double shadowLength,
                   const double capCenter[2][3],const double capRadius[2][2],
                   const double capTM[2][16], const double bodyPoint[8][3])
{
  // Draw the cone projections from the end caps
  renderSphereShadow(shadowLength,&capCenter[0][0],&capRadius[0][0],&capTM[0][0]);
  renderSphereShadow(shadowLength,&capCenter[1][0],&capRadius[1][0],&capTM[1][0]);

  // Draw the hexahedron projection
  glBegin(GL_QUAD_STRIP);
  glVertex3dv(&bodyPoint[0][0]);
  glVertex3dv(&bodyPoint[4][0]);

  glVertex3dv(&bodyPoint[1][0]);
  glVertex3dv(&bodyPoint[5][0]);

  glVertex3dv(&bodyPoint[3][0]);
  glVertex3dv(&bodyPoint[7][0]);

  glVertex3dv(&bodyPoint[2][0]);
  glVertex3dv(&bodyPoint[6][0]);

  glVertex3dv(&bodyPoint[0][0]);
  glVertex3dv(&bodyPoint[4][0]);
  glEnd();
  // Cap the top.
  glBegin(GL_QUADS);
    glVertex3dv(&bodyPoint[0][0]);
    glVertex3dv(&bodyPoint[1][0]);
    glVertex3dv(&bodyPoint[3][0]);
    glVertex3dv(&bodyPoint[2][0]);
  glEnd();
}



#define PLOT_BLOCK_POINT(vv) \
  glVertex3dv(boxPoint[0][vv]); \
  glVertex3dv(boxPoint[1][vv]);
#define PLOT_BLOCK_4LIST(v1,v2,v3,v4) \
  PLOT_BLOCK_POINT(v1)\
  PLOT_BLOCK_POINT(v2)\
  PLOT_BLOCK_POINT(v3)\
  PLOT_BLOCK_POINT(v4)\
  PLOT_BLOCK_POINT(v1)
#define PLOT_BLOCK_6LIST(v1,v2,v3,v4,v5,v6) \
  PLOT_BLOCK_POINT(v1)\
  PLOT_BLOCK_POINT(v2)\
  PLOT_BLOCK_POINT(v3)\
  PLOT_BLOCK_POINT(v4)\
  PLOT_BLOCK_POINT(v5)\
  PLOT_BLOCK_POINT(v6)\
  PLOT_BLOCK_POINT(v1)

/**
  This is admittedly ridiculous.  However, I can't think
  of a way to create a general function that handles all
  the cases.  So, rather than waste even more time, I just
  hardcoded all of the 27 solutions.
*/
void MyGLWidget::renderBoxShadow(const double* center, const double* rotation,
               int caseVal,const double boxPoint[2][8][3])
{
  //double sx = sides[0]/2;
  //double sy = sides[1]/2;
  //double sz = sides[2]/2;
  double tm[16];
  glPushMatrix();
  tm[0] = rotation[0]; tm[1] = rotation[4]; tm[2]  = rotation[8];  tm[3] = 0;
  tm[4] = rotation[1]; tm[5] = rotation[5]; tm[6]  = rotation[9];  tm[7] = 0;
  tm[8] = rotation[2]; tm[9] = rotation[6]; tm[10] = rotation[10]; tm[11] = 0;
  tm[12] = 0; tm[13] = 0; tm[14] = 0; tm[15] = 1;
  glTranslated(center[0],center[1],center[2]);
  glMultMatrixd(tm);

  glBegin(GL_QUAD_STRIP);
  switch (caseVal) {
    // Points
    case 0:
      PLOT_BLOCK_6LIST(1,5,4,6,2,3);
      break;
    case 2:
      PLOT_BLOCK_6LIST(0,2,3,7,5,4);
      break;
    case 6:
      PLOT_BLOCK_6LIST(0,4,6,7,3,1);
      break;
    case 8:
      PLOT_BLOCK_6LIST(0,2,6,7,5,1);
      break;
    case 18:
      PLOT_BLOCK_6LIST(0,1,5,7,6,2);
      break;
    case 20:
      PLOT_BLOCK_6LIST(0,1,3,7,6,4);
      break;
    case 24:
      PLOT_BLOCK_6LIST(0,4,5,7,3,2);
      break;
    case 26:
      PLOT_BLOCK_6LIST(1,3,2,6,4,5);
      break;
    // Edges
    case 1: // -z-y
      PLOT_BLOCK_6LIST(0,2,3,1,5,4);
      break;
    case 3: // -z-x
      PLOT_BLOCK_6LIST(0,4,6,2,3,1);
      break;
    case 5: // -z x
      PLOT_BLOCK_6LIST(0,2,3,7,5,1);
      break;
    case 7: // -z y
      PLOT_BLOCK_6LIST(0,2,6,7,3,1);
      break;
    case 9: // -y-x
      PLOT_BLOCK_6LIST(0,1,5,4,6,2);
      break;
    case 11: // -y x
      PLOT_BLOCK_6LIST(0,1,3,7,5,4);
      break;
    case 15: // y-x
      PLOT_BLOCK_6LIST(0,4,6,7,3,2);
      break;
    case 17: // x y
      PLOT_BLOCK_6LIST(7,5,1,3,2,6);
      break;
    case 19: // z-y
      PLOT_BLOCK_6LIST(0,1,5,7,6,4);
      break;
    case 21: // z-x
      PLOT_BLOCK_6LIST(0,4,5,7,6,2);
      break;
    case 23: // z x
      PLOT_BLOCK_6LIST(7,6,4,5,1,3);
      break;
    case 25: // z y
      PLOT_BLOCK_6LIST(7,3,2,6,4,5);
      break;

    // Faces
    case 4:  // -Z
      PLOT_BLOCK_4LIST(0,2,3,1);
      break;
    case 10: // -Y
      PLOT_BLOCK_4LIST(0,1,5,4);
      break;
    case 12: // -X
      PLOT_BLOCK_4LIST(0,4,6,2);
      break;
    case 14: // X
      PLOT_BLOCK_4LIST(7,5,1,3);
      break;
    case 16: // Y
      PLOT_BLOCK_4LIST(7,3,2,6);
      break;
    case 22: // Z face
      PLOT_BLOCK_4LIST(7,6,4,5);
      break;
    // Volume
    case 13:
    default:
      break;
  }
  glEnd();
  // There's no need to cap this shadow because it fits
  // snuggly against the box.  Unless the eye is inside the box.
  glPopMatrix();
}


////////////////////////////////////////////////////////////////////////
void MyGLWidget::renderSpace(dSpaceID space)
{
  int geomCnt = dSpaceGetNumGeoms( space );
  for (int ii=0;ii<geomCnt;++ ii) {
    dGeomID gg = dSpaceGetGeom(space, ii);
    renderGeom(gg);
  }
}

void MyGLWidget::renderLineList()
{
  while (sim_world->line_list.size()>2000) sim_world->line_list.pop_front();
  int cnt = sim_world->line_list.size();

  glLineWidth(5.0);
  glBegin(GL_LINES);
  for (int ii=0;ii<cnt;++ii) {
    glColor3dv(sim_world->line_list[ii].col);
    glVertex3dv(sim_world->line_list[ii].p1);
    glVertex3dv(sim_world->line_list[ii].p2);
  }
  glEnd();
  glLineWidth(1.0);
}

void MyGLWidget::renderSimWorld(SimWorld* world)
{
  dSpaceID space = world->getSpace();


  //space = world->getMarkSpace();
  //glColor3d(.7,0,0);
  //renderSpace(space);

  CapBody* cb = world->getBody();
  //SwingData* md = world->getMarkerData();
  //LiveMarkerData* md = world->getMarkerData();
  //PokeMarkerData* md = world->getMarkerData();
#if defined( BOARD_DATA )
  BoardMarkerData* md = world->getMarkerData();
#elif defined( POKE_DATA )
  PokeMarkerData* md = world->getMarkerData();
#else
  MarkerData* md = world->getMarkerData();
#endif

  /*
    Check the marker.  If it's not attaching, draw it red.
    If it's trying, but can't, draw it yellow.
    Otherwise, green.  If it's trying, draw the attachment
    point and a connecting line.
    */
  glDisable(GL_TEXTURE_2D);
  if (show_markers) {
    const dReal* mPos;
    dVector3 mappedPoint;
    const dReal* rPos;
    for (int ii=0;ii<MARKER_COUNT;++ii) {
      rPos = cb->marker_to_body[ii].position;
      mPos = dBodyGetPosition(md->body[ii]);
      //if (mPos[2]<=0) continue;
      int id = cb->marker_to_body[ii].id;
      if (id>=0 && id<=50) {
        dBodyGetRelPointPos(cb->body_segments[id],
                          rPos[0],rPos[1],rPos[2],mappedPoint);
        if (mPos[2]>0) {
          if (md->try_link[ii]) glColor3d(0,.7,0);
          else glColor3d(.7,0,0);
          glPushMatrix();
          glTranslated(mPos[0],mPos[1],mPos[2]);
          renderSphere(.01,10,5);
          glPopMatrix();
          if (md->try_link[ii]) {
            glBegin(GL_LINES);
              glVertex3dv(mPos);
              glVertex3dv(mappedPoint);
            glEnd();
          }
        }
        if (md->try_link[ii]) {
           glColor3d(0,.2,.5);
           glPushMatrix();
           glTranslated(mappedPoint[0],mappedPoint[1],mappedPoint[2]);
           renderSphere(.005,4,4);
           glPopMatrix();
        }
      }
    }

    if (draw_lines) renderLineList();
  }
  glEnable(GL_TEXTURE_2D);


  /*
  glBegin(GL_QUADS);
  glColor3d(.2,.2,.5);
  glNormal3d(0,0,1);
  glVertex3d(md->cFrame->boards[0][0][2]*.001,
             md->cFrame->boards[0][0][0]*.001,
             md->cFrame->boards[0][0][1]*.001);
  glVertex3d(md->cFrame->boards[0][1][2]*.001,
             md->cFrame->boards[0][1][0]*.001,
             md->cFrame->boards[0][1][1]*.001);
  glVertex3d(md->cFrame->boards[0][2][2]*.001,
             md->cFrame->boards[0][2][0]*.001,
             md->cFrame->boards[0][2][1]*.001);
  glVertex3d(md->cFrame->boards[0][3][2]*.001,
             md->cFrame->boards[0][3][0]*.001,
             md->cFrame->boards[0][3][1]*.001);

  glVertex3d(md->cFrame->boards[1][0][2]*.001,
             md->cFrame->boards[1][0][0]*.001,
             md->cFrame->boards[1][0][1]*.001);
  glVertex3d(md->cFrame->boards[1][1][2]*.001,
             md->cFrame->boards[1][1][0]*.001,
             md->cFrame->boards[1][1][1]*.001);
  glVertex3d(md->cFrame->boards[1][2][2]*.001,
             md->cFrame->boards[1][2][0]*.001,
             md->cFrame->boards[1][2][1]*.001);
  glVertex3d(md->cFrame->boards[1][3][2]*.001,
             md->cFrame->boards[1][3][0]*.001,
             md->cFrame->boards[1][3][1]*.001);
  glEnd();
  */

  /*glDisable(GL_LIGHTING);
  glDisable(GL_TEXTURE_2D);
  glLineWidth(2.5);
  glBegin(GL_LINES);
  glColor3d(1,0,0);

  glVertex3d(md->cFrame->boards[0][4][2]*.001,
             md->cFrame->boards[0][4][0]*.001,
             md->cFrame->boards[0][4][1]*.001);
  glVertex3d(md->cFrame->boards[0][4][2]*.001,
             md->cFrame->boards[0][4][0]*.001,
             md->cFrame->boards[0][4][1]*.001
             + md->cFrame->boards[0][4][3]*0.02);



  glVertex3d(md->cFrame->boards[1][4][2]*.001,
             md->cFrame->boards[1][4][0]*.001,
             md->cFrame->boards[1][4][1]*.001);
  glVertex3d(md->cFrame->boards[1][4][2]*.001,
             md->cFrame->boards[1][4][0]*.001,
             md->cFrame->boards[1][4][1]*.001
             + md->cFrame->boards[1][4][3]*0.02);


  glEnd();
  glEnable(GL_TEXTURE_2D);
  glEnable(GL_LIGHTING);
  glLineWidth(1);*/

  //dVector4 planeEq;
  //planeEq[0]=0;
  //planeEq[1]=0;
  //planeEq[2]=1;
  //planeEq[3]=0;
  //glColor3d(1,1,1);
  //glBindTexture(GL_TEXTURE_2D,woodTex);
  //renderPlane(planeEq);

  renderRoom();

  //SwingDataFrame* df = md->cFrame;
  //MarkerDataFrame* df = md->cFrame;
  //C3dFloatFrame* df = md->cFrame;



  glBindTexture(GL_TEXTURE_2D,noise_texture);

  // Draw the targets
  /*
  if (df->state==3) {
    glPushMatrix();
    glTranslated(df->tPos[0],df->tPos[1],df->tPos[2]);
    glColor3d(0,1,0);
    renderSphere(0.05);
    glPopMatrix();
  } else {

    glPushMatrix();
    glTranslated(df->bPos[0],df->bPos[1],df->bPos[2]);
    glColor3d(0,0.8,0.2);
    renderSphere(0.05);
    glPopMatrix();
  }*/

  /*
  glPushMatrix();
  glTranslated(df->gPos[0],df->gPos[1],df->gPos[2]);
  glColor3d(0.8,0.4,0);
  renderSphere(0.5);
  glPopMatrix();*/


  glColor3d(1,1,1);
  renderSpace(space);
  //renderSpace(simWorld->getTargetSpace());

  /*
  glBindTexture(GL_TEXTURE_2D,racquetTex);
  glPushMatrix();
  double lx=.145, ly=.28, lz=.005;

  glTranslated(df->rPos[0],df->rPos[1],df->rPos[2]);
  rotateByQuaternion(df->rQuat);
  glColor3d(1,1,1);
  glDisable(GL_TEXTURE_GEN_S);
  glDisable(GL_TEXTURE_GEN_T);
  glDisable(GL_LIGHTING);
  glBegin (GL_QUADS);

  glNormal3d (0,0,-1);
  glTexCoord2d(1,0);
  glVertex3d (-lx,-ly,-lz);
  glTexCoord2d(1,1);
  glVertex3d (-lx, ly,-lz);
  glTexCoord2d(0,1);
  glVertex3d ( lx, ly,-lz);
  glTexCoord2d(0,0);
  glVertex3d ( lx,-ly,-lz);

  glNormal3d (0,0,1);
  glTexCoord2d(1,1);
  glVertex3d ( lx, ly, lz);
  glTexCoord2d(0,1);
  glVertex3d (-lx, ly, lz);
  glTexCoord2d(0,0);
  glVertex3d (-lx,-ly, lz);
  glTexCoord2d(1,0);
  glVertex3d ( lx,-ly, lz);
  glEnd();
  glEnable(GL_LIGHTING);
  glEnable (GL_TEXTURE_GEN_S);
  glEnable (GL_TEXTURE_GEN_T);
  glPopMatrix();
  */


}


void MyGLWidget::renderSpaceShadows(dSpaceID space)
{
  int geomCnt = dSpaceGetNumGeoms( space );
  for (int ii=0;ii<geomCnt;++ ii) {
    dGeomID gg = dSpaceGetGeom(space, ii);
    renderGeomShadow(gg);
  }
}

void MyGLWidget::renderSimWorldShadows(SimWorld* world)
{
  dSpaceID space = world->getSpace();
  renderSpaceShadows(space);
  //renderSpaceShadows(simWorld->getTargetSpace());
  space = world->getMarkSpace();
  renderSpaceShadows(space);
}

void MyGLWidget::renderScene()
{
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  static unsigned int ff = 0;
  glPushMatrix();
  glColor3f(1,1,1);

  double mat[16];
  camera.getCMajorInvTrans(mat);
  glMultMatrixd(mat);

  glLightfv(GL_LIGHT0, GL_POSITION, light_pos);    // Set light position

  //glDisable(GL_LIGHTING);
  //bd->render();
  //glEnable(GL_LIGHTING);
  // Draw the unshaded things
  glNewList(front_list,GL_COMPILE_AND_EXECUTE);
  renderSimWorld(sim_world);
  glEndList();

  //Prepare to stencil on some shadows
  glDisable(GL_LIGHTING);
  glDepthMask(GL_FALSE); // Don't write into the depth buffer, but still read from it.
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_STENCIL_TEST);
  glColorMask(0,0,0,0); // Don't write any pixels
  glStencilFunc(GL_ALWAYS, 1, 0xffffffff);

  // first pass, stencil operation increases stencil value
  glFrontFace(GL_CCW); // We draw the "backs" of the shadows.
  glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);

  // Draw the shadow backs.
  glNewList(shadow_list,GL_COMPILE_AND_EXECUTE);
  renderSimWorldShadows(sim_world);
  glEndList();

  // second pass, stencil operation decreases stencil value
  // Draw the shadow fronts
  glFrontFace(GL_CW);
  glStencilOp(GL_KEEP, GL_KEEP, GL_DECR);
  glCallList(shadow_list);

  // Restore the state machine to rendering mode
  glFrontFace(GL_CCW);
  glColorMask(1, 1, 1, 1);
  glStencilFunc(GL_NOTEQUAL, 0, 0xffffffff);
  glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
  glEnable(GL_LIGHTING);

  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_ambient);		// ``Turnoff'' the Diffuse Light
  glClear( GL_DEPTH_BUFFER_BIT );
  glDepthFunc(GL_LEQUAL);
  glDepthMask(GL_TRUE);
  glCallList(front_list);                        // Draw the shadowed areas
  glDisable(GL_STENCIL_TEST);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);		// Restore the Diffuse Light
  glPopMatrix();

  //glFlush();
  //SDL_GL_SwapBuffers( );
}

void MyGLWidget::initializeGraphicsScene()
{
  //scene.addLine(0,3.14,0,-3.14,QPen(Qt::black));
  scene.setAxis(500,10,100);

  for (int ii=0;ii<11;++ii) {
    graphics_path_item[ii]=new QGraphicsPathItem(0,&scene);
    graphics_path_item[ii]->setPen(QPen(QColor(rand()%250,rand()%250,rand()%250)));

    text_item[ii]=new QGraphicsTextItem(0,&scene);
    text_item[ii]->setFont(QFont(QString("Helvetica"),20));
  }


}

void MyGLWidget::updateGraphicsScene()
{
  QPainterPath path;


  //setPathItem("R. Knee",CapBody::R_KNEE_JOINT,0,0);
  //setPathItem("L. Knee",CapBody::L_KNEE_JOINT,0,1);
  //setPathItem("R. Hip",CapBody::R_HIP_JOINT,0,2);
  //setPathItem("L. Hip",CapBody::L_HIP_JOINT,0,3);
  //setPathItem("R. Ankle",CapBody::R_ANKLE_JOINT,0,4);
  //setPathItem("L. Ankle",CapBody::L_ANKLE_JOINT,0,5);
  //setPathItem("Waist",CapBody::WAIST_JOINT,0,6);
  //setPathItem("R. ShoulderX",CapBody::R_SHOULDER_JOINT,0,7);
  //setPathItem("R. ShoulderY",CapBody::R_SHOULDER_JOINT,2,8);
  //setPathItem("R. ShoulderZ",CapBody::R_SHOULDER_JOINT,1,9);
  //setPathItem("L. Shoulder",CapBody::L_SHOULDER_JOINT,0,10);
}

void MyGLWidget::setPathItem(const QString& label,int jj, int ax, int pp)
{
  pp; ax; jj; label;
  /*
  QPainterPath path;
  double  pOff = -30;
  int ss = simWorld->torqueSeq->size();
  //if (ss>scene.axSize) scene.setAxis(ss+100,10,100);
  int mVal = 0;
  if (ss>500) mVal = ss-500;
  simWorld->torqueSeq->fillPath(&path,mVal,ss,jj,ax);

  //QPainterPath* path = simWorld->seq->getPath(jj,ax);
  //if (path) gpItem[pp]->setPath(*path);

  gpItem[pp]->setPath(path);
  gpItem[pp]->setTransform(QTransform(1,0,0,-.1,0,pp*pOff));
  textItem[pp]->setPlainText(label);
  QRectF rect = textItem[pp]->boundingRect();
  textItem[pp]->setPos(-rect.width()-10,pp*pOff-rect.height()/2);
  */
}


