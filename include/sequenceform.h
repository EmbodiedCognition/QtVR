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
#ifndef SEQUENCEFORM_H
#define SEQUENCEFORM_H

#include <QWidget>
class QTreeWidgetItem;

namespace Ui {
    class SequenceForm;
}

class SequenceForm : public QWidget
{
    Q_OBJECT

public:
    explicit SequenceForm(QWidget *parent = 0);
    ~SequenceForm();

  void setMode(int);
signals:
  void newPlot(QString);
public slots:

  void newSequenceSlot();
  void loadSequenceSlot();
  void saveSequenceSlot();
  void closeSequenceSlot();

  void newViewSlot();
  void removeViewSlot();

  void newPlotSlot();

  void selectionChangedSlot();
  void changeSlot(QTreeWidgetItem*,int);

private:
    Ui::SequenceForm *ui;
};

#endif // SEQUENCEFORM_H
