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
/*
  Constants that shouldn't change much.
  */
#ifndef __DATACVALUES_H
#define __DATACVALUES_H


/**
  The LED IDs for the suit, gloves, and helmet.
*/
enum MarkerType {
  LED_HEAD_BL=0,
  LED_HEAD_ML,
  LED_HEAD_FL,
  LED_HEAD_FR,
  LED_HEAD_MR,
  LED_HEAD_BR,
  LED_L_COLLAR,
  LED_L_SHOULDER,
  LED_L_ELBOW,
  LED_L_WRIST,
  LED_L_PINKY_FINGER,
  LED_L_RING_FINGER,
  LED_L_MIDDLE_FINGER,
  LED_L_INDEX_FINGER,
  LED_R_COLLAR,
  LED_R_SHOULDER,
  LED_R_ELBOW,
  LED_R_WRIST,
  LED_R_PINKY_FINGER,
  LED_R_RING_FINGER,
  LED_R_MIDDLE_FINGER,
  LED_R_INDEX_FINGER,
  LED_R_OUTER_HAND,
  LED_R_INNER_HAND,
  LED_R_THUMB_JOINT,
  LED_R_THUMB_NAIL,
  LED_L_OUTER_HAND,
  LED_L_INNER_HAND,
  LED_L_THUMB_JOINT,
  LED_L_THUMB_NAIL,
  LED_LOWER_CHEST,
  LED_UPPER_CHEST,
  LED_UPPER_BACK,
  LED_LOWER_BACK,
  LED_L_HAUNCH,
  LED_R_HAUNCH,
  LED_R_HIP,
  LED_R_KNEE,
  LED_R_SHIN,
  LED_R_ANKLE,
  LED_R_HEEL,
  LED_R_OUTER_TOE,
  LED_R_INNER_TOE,
  LED_L_HIP,
  LED_L_KNEE,
  LED_L_SHIN,
  LED_L_ANKLE,
  LED_L_HEEL,
  LED_L_OUTER_TOE,
  LED_L_INNER_TOE,
  LED_COUNT
};

#endif

