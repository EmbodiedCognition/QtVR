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

#ifndef C3DDATA_H
#define C3DDATA_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <cctype>
#include <iostream>

using std::map;
using std::string;
using std::vector;


typedef unsigned char uint8;
typedef signed char int8;
typedef unsigned short uint16;
typedef short int16;
typedef int int32;
typedef unsigned int uint32;
typedef float float32;



/**
  A simple structure for holding the data
  contained in the header of a C3D file.
*/
struct C3dHeader {
  uint8   parameterBlock;
  uint16  pointCount;
  uint16  analogCount;
  uint16  firstFrame;
  uint16  lastFrame;
  uint16  maxGap;
  float32 scaleFactor;
  uint16  dataBlock;
  uint16  samplePerFrame;
  float32 frameRate;
  uint16  labelBlock;

  // Currently don't care about events
  void loadHeader(FILE* c3dFile);
};

struct C3dParam
{
  int8 groupID;
  string paramName;
  string paramDescription;
  int8 dataType;
  uint8 dimensionCount;

  vector<uint8> paramDimensions;

  //We don't know ahead of time, which datatype
  //we'll be holding.  The typing mechanism doesn't
  //like this.  We can either hold four different
  //pointers and allocate to the appropriate one
  //when we know, or we can just hold a byte array
  //and interpret the data when we need it.
  vector<uint8> paramData;

  //We also don't know how many dimensions are in
  //the data, so that makes accessor functions ugly
  int8 getInt8(uint32 idx);
  uint8 getUint8(uint32 idx);
  int16 getInt16(uint32 idx);
  uint16 getUint16(uint32 idx);
  int32 getInt32(uint32 idx);
  uint32 getUint32(uint32 idx);
  float32 getFloat32(uint32 idx);
  string getString(uint32 idx);

};

struct C3dGroup
{
  int8 groupID;
  string groupName;
  string groupDescription;

  map<string,C3dParam> params;
};

struct C3dParameters
{
  uint8 blockCount;
  //Processor type
  map<string,C3dGroup*> groups;

  // Probably a waste of memory to create all of these
  // but there just aren't that many; so convenience wins.
  C3dGroup groupArray[128];

  void loadParameters(FILE* c3dFile,C3dHeader& header);
};

/*
  These structures shouldn't have any problems with
  boundary alignment; so we should be able to read
  directly into an array of them.
*/
struct IPoint
{
  int16 point[4];
};
struct FPoint
{
  float32 point[4];
};

/**
  The data are a bunch of arrays of either float or integer
  points in 3D and any number of analog channels.
*/
struct C3dData
{
  bool useFloat;
  uint16 pointsPerFrame;
  uint16 analogPerFrame;
  uint32 frames;

  IPoint* iPoints;
  FPoint* fPoints;

  int16*   iAnalog;
  float32* fAnalog;

  void loadData(FILE* c3dFile,C3dHeader& header,C3dParameters& params);
};

struct C3dFloatFrame
{
  uint32 pointCount;
  FPoint* data;
  ////////////////////////
  C3dFloatFrame(uint32 points);
  ~C3dFloatFrame();


  void transform(float32 tm00,float32 tm01,float32 tm02,float32 tm03,
                 float32 tm10,float32 tm11,float32 tm12,float32 tm13,
                 float32 tm20,float32 tm21,float32 tm22,float32 tm23);

};

/**
  Encapsulate the information contained in a .c3d file.

*/
struct C3dFile
{
  C3dHeader header;
  C3dParameters parameters;
  C3dData data;

  void loadFile(const char* filename);
  void readFrame(uint32 frameNo,C3dFloatFrame* frameDat);
};

#endif // C3DDATA_H

