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
#include <cstring>
#include "c3ddata.h"

/**
  A quick and dirty way of converting from
  one type to another.  Stash bytes in union
  memory and pull it out as something else.
*/
union TypeConvert {
  uint8   _bb[4];
  uint16  _uu[2];
  int16   _ss[2];
  uint32  _u32;
  int32   _s32;
  float32 _ff;
};

static TypeConvert TC;


/**
  Quick and dirty load.
  Currently does no error/sanity checking.
  Assumes one type of endianess and floating
  point format.
*/
void C3dHeader::loadHeader(FILE* c3dFile)
{
  uint16 block[256];

  fseek(c3dFile,0,SEEK_SET);
  fread(&block,2,256,c3dFile);

    TC._uu[0] = block[0];
  parameterBlock = TC._bb[0];
  pointCount  = block[1];
  analogCount = block[2];
  firstFrame = block[3];
  lastFrame = block[4];
  maxGap = block[5];
    TC._uu[0] = block[6];
    TC._uu[1] = block[7];
  scaleFactor = TC._ff;
  dataBlock = block[8];
  samplePerFrame = block[9];
    TC._uu[0] = block[10];
    TC._uu[1] = block[11];
  frameRate  = TC._ff;
  labelBlock = block[148];
}

//////////////////////////////////////////////////

int8 C3dParam::getInt8(uint32 idx)
{
  return int8(paramData[idx]);
}

uint8 C3dParam::getUint8(uint32 idx)
{
  return paramData[idx];
}

int16 C3dParam::getInt16(uint32 idx)
{
  TC._bb[0] = paramData[idx*2];
  TC._bb[1] = paramData[idx*2 + 1];
  return TC._ss[0];
}

uint16 C3dParam::getUint16(uint32 idx)
{
  TC._bb[0] = paramData[idx*2];
  TC._bb[1] = paramData[idx*2 + 1];
  return TC._uu[0];
}

int32 C3dParam::getInt32(uint32 idx)
{
  TC._bb[0] = paramData[idx*2];
  TC._bb[1] = paramData[idx*2 + 1];
  TC._bb[2] = paramData[idx*2 + 2];
  TC._bb[3] = paramData[idx*2 + 3];

  return TC._s32;
}

uint32 C3dParam::getUint32(uint32 idx)
{
  TC._bb[0] = paramData[idx*2];
  TC._bb[1] = paramData[idx*2 + 1];
  TC._bb[2] = paramData[idx*2 + 2];
  TC._bb[3] = paramData[idx*2 + 3];

  return TC._u32;
}

float32 C3dParam::getFloat32(uint32 idx)
{
  TC._bb[0] = paramData[idx*2];
  TC._bb[1] = paramData[idx*2 + 1];
  TC._bb[2] = paramData[idx*2 + 2];
  TC._bb[3] = paramData[idx*2 + 3];

  return TC._ff;
}

string C3dParam::getString(uint32 idx)
{
  string rVal;
  uint32 maxSize = paramDimensions[0];


  for (uint32 ii = 0;ii<maxSize;++ii) {
    uint8 chr = paramData[idx*maxSize + ii];
    if (chr==32) break;
    rVal.push_back(chr);
  }
  return rVal;

}
////////////////////////////////////////////////
void C3dParameters::loadParameters(FILE* c3dFile,C3dHeader& header)
{
  int8 buffer[512];

  int8 charsInName;
  int8 groupID;
  string name;
  int16 offsetToNext;

  fseek(c3dFile,(header.parameterBlock-1)*512,SEEK_SET);
  fread(buffer,1,4,c3dFile);

  blockCount = buffer[2];
  bool reading = true;
  int ii = 0;
  while (reading) {
    ii++;
    fread(buffer,1,2,c3dFile);
    // ***** Right now we don't care about locked values.
    charsInName = abs(buffer[0]);
    groupID = buffer[1];
    // ***** There's a chance that this might break
    // if the last parameter ended exactly on the
    // end of a block
    if (groupID==0 || charsInName==0) break;

    name.resize(charsInName);
    fread(&name[0],1,charsInName,c3dFile);
    // There should be string::upperCase... but no, not in the stl
    // instead, we get:
    std::transform(name.begin(), name.end(), name.begin(), (int (*)(int))std::toupper);

    long filePlace = ftell(c3dFile);
    fread(&offsetToNext,1,2,c3dFile);
    if (groupID < 0) {
      // Handle group
      groupID = abs(groupID);
      groupArray[groupID].groupID = groupID; // This step seems dumb
      groupArray[groupID].groupName = name;
      groups[name] = &groupArray[groupID];

      uint8 descriptionLength = 0 ;
      fread(&descriptionLength,1,1,c3dFile);
      if (descriptionLength) {
        groupArray[groupID].groupDescription.resize(descriptionLength);
        fread(&(groupArray[groupID].groupDescription[0]),1,descriptionLength,c3dFile);
      }
    } else {
      // Handle param
      C3dParam& param = groupArray[groupID].params[name];
      param.groupID = groupID;
      param.paramName = name;

      fread(&param.dataType,1,1,c3dFile);
      // ***** We could handle this better...
      if (param.dataType<0) param.dataType = abs(param.dataType);
      //How many dimensions are there?
      fread(&param.dimensionCount,1,1,c3dFile);

      if (param.dimensionCount) {
        // How large is each dimension?
        param.paramDimensions.resize(param.dimensionCount);
        fread(&param.paramDimensions[0],1,param.dimensionCount,c3dFile);
      }

      if (param.dataType) {
        unsigned int sizeNeeded = 1;
        for (int ii=0;ii<param.dimensionCount;++ii) {
          sizeNeeded*=param.paramDimensions[ii];
        }
        param.paramData.resize(sizeNeeded*param.dataType);
        if (sizeNeeded) {
          fread(&param.paramData[0],param.dataType,sizeNeeded,c3dFile);
        } else {
          //printf("Empty parameter\n");
        }
      }
      uint8 descriptionLength = 0 ;
      fread(&descriptionLength,1,1,c3dFile);
      if (descriptionLength) {
        param.paramDescription.resize(descriptionLength);
        fread(&param.paramDescription[0],1,descriptionLength,c3dFile);
      }
    }
    //fread(buffer,1,25,c3dFile);
    fseek(c3dFile,filePlace+offsetToNext,SEEK_SET);

  }
}
//////////////////////////////////////////////////////////
void C3dData::loadData(FILE* c3dFile,C3dHeader& header,C3dParameters& params)
{
  float32 scaleVal = params.groups["POINT"]->params["SCALE"].getFloat32(0);
  if (scaleVal<0) {
    useFloat = true;
    scaleVal*=-1;
  } else {
    useFloat = false;
  }

  uint16 startBlock = params.groups["POINT"]->params["DATA_START"].getUint16(0);
  frames = params.groups["POINT"]->params["FRAMES"].getUint16(0);
  pointsPerFrame = params.groups["POINT"]->params["USED"].getUint16(0);

  uint32 startField = params.groups["TRIAL"]->params["ACTUAL_START_FIELD"].getUint32(0);
  uint32 endField = params.groups["TRIAL"]->params["ACTUAL_END_FIELD"].getUint32(0);
  frames = endField - startField + 1;

  analogPerFrame = params.groups["ANALOG"]->params["USED"].getUint16(0);

  if (startBlock != header.dataBlock) {
    printf("Oops!\n");
  }

  fseek(c3dFile,(header.dataBlock-1)*512,SEEK_SET);
  if (useFloat) {
    iPoints=NULL;
    iAnalog=NULL;
    fPoints = new FPoint[pointsPerFrame*frames];
    fAnalog = new float32[analogPerFrame*frames];

    for (uint32 ii = 0;ii<frames;++ii) {
      fread(&fPoints[ii*pointsPerFrame],sizeof(FPoint),pointsPerFrame,c3dFile);
      fread(&fAnalog[ii*analogPerFrame],sizeof(float32),analogPerFrame,c3dFile);
    }
  } else {
    fPoints=NULL;
    fAnalog=NULL;
    iPoints = new IPoint[pointsPerFrame*frames];
    iAnalog = new int16[analogPerFrame*frames];

    for (uint32 ii = 0;ii<frames;++ii) {
      fread(&iPoints[ii*pointsPerFrame],sizeof(IPoint),pointsPerFrame,c3dFile);
      fread(&iAnalog[ii*analogPerFrame],sizeof(int16),analogPerFrame,c3dFile);
    }
  }
}
//////////////////////////////////////////////////////////
C3dFloatFrame::C3dFloatFrame(uint32 points)
  : pointCount(points)
{
  data = new FPoint[points];
}
C3dFloatFrame::~C3dFloatFrame()
{
  delete[] data;
  data = NULL;
}
////////////////////////

/**
  Transformation matrix.
  Final row omitted since it's always [0 0 0 1]
  Remember that the translation happens after the rotation/scaling
*/
void C3dFloatFrame::transform(float32 tm00,float32 tm01,float32 tm02,float32 tm03,
               float32 tm10,float32 tm11,float32 tm12,float32 tm13,
               float32 tm20,float32 tm21,float32 tm22,float32 tm23)
{
  FPoint point;
  for (uint32 ii=0;ii<pointCount;++ii) {
    point=data[ii];
    data[ii].point[0] = tm00*point.point[0] + tm01*point.point[1] + tm02*point.point[2] + tm03;
    data[ii].point[1] = tm10*point.point[0] + tm11*point.point[1] + tm12*point.point[2] + tm13;
    data[ii].point[2] = tm20*point.point[0] + tm21*point.point[1] + tm22*point.point[2] + tm23;
    // Don't touch the last value,
    // it's a confidence measure.
  }
}

//////////////////////////////////////////////////////////
/**
  Probably ought to have some sanity checks in here...
  We'll crash if the file doesn't exist or has
  anything wrong with it.
*/
void C3dFile::loadFile(const char* filename)
{
  FILE* c3dFile = fopen(filename,"rb");
  header.loadHeader(c3dFile);
  parameters.loadParameters(c3dFile,header);
  data.loadData(c3dFile,header,parameters);
  fclose(c3dFile);
}

void C3dFile::readFrame(uint32 frameNo,C3dFloatFrame* frameDat)
{
  //***** We should check to see if there are any data
  //i.e., has loadFile been executed first.
  // We could avoid this problem by making loadFile into
  // the constructor.

  if (frameNo>=data.frames || frameDat->pointCount!=data.pointsPerFrame) return;

  memcpy(frameDat->data,&data.fPoints[frameNo*data.pointsPerFrame],data.pointsPerFrame*sizeof(FPoint));
}


