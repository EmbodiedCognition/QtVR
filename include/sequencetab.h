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
#ifndef SEQUENCETAB_H
#define SEQUENCETAB_H

#include <QScrollArea>

/**
  Display and control the sequences that are
  currently loaded.
  */
class SequenceTab : public QScrollArea
{
    Q_OBJECT
public:
    explicit SequenceTab(QWidget *parent = 0);

signals:

public slots:



};

#endif // SEQUENCETAB_H


/*
  loadNewSequence() -- File Dialog
  Unload sequence()
  createNewSequence() -- File dialog?
    Type
  saveSequence() -- file dialog?


  Recording to sequence()

  setSequenceOffset()

  If we want to have a single sequence that
  feeds data to different degrees of freedom
  with different offsets, we could either
  load the sequence twice (!) or we can
  have a 'view' onto the data.

  There is always a constant sequence of each
  type and a 'recording' sequence of each type,
  but if you record to the constant (or none)
  then we just don't record.



  */
