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
#include "sequenceform.h"
#include "ui_sequenceform.h"

#include "ui_newSequenceDialog.h"

SequenceForm::SequenceForm(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SequenceForm)
{
    ui->setupUi(this);

  setMode(0);
}

SequenceForm::~SequenceForm()
{
    delete ui;
}

void SequenceForm::setMode(int mode)
{
  switch (mode) {
  case 0: // Nothing selected
    ui->closeSeqButton->setDisabled(true);
    ui->newPlotButton->setDisabled(true);
    ui->newViewButton->setDisabled(true);
    ui->removeViewButton->setDisabled(true);
    ui->saveSeqButton->setDisabled(true);
    break;
  case 1: // A sequence is selected
    ui->closeSeqButton->setDisabled(false);
    ui->newPlotButton->setDisabled(false);
    ui->newViewButton->setDisabled(false);
    ui->removeViewButton->setDisabled(true);
    ui->saveSeqButton->setDisabled(false);
    break;
  case 2: // A view is selected
    ui->closeSeqButton->setDisabled(false);
    ui->newPlotButton->setDisabled(false);
    ui->newViewButton->setDisabled(false);
    ui->removeViewButton->setDisabled(false);
    ui->saveSeqButton->setDisabled(false);
    break;
  default:
    break;
  }

}

/**
  Create a new sequence
  -- open a dialog asking for a name and type
  */
void SequenceForm::newSequenceSlot()
{
  Ui::NewSequenceDialog* dialogUi = new Ui::NewSequenceDialog;
  QDialog* dialog = new QDialog(this);
  dialogUi->setupUi(dialog);
  if (dialog->exec()) {
    QTreeWidgetItem* item =
        new QTreeWidgetItem(
          QStringList()
          << dialogUi->nameEdit->text()
          << ""
          << "0"
          << dialogUi->typeComboBox->currentText());
    item->setCheckState(1,Qt::Unchecked);
    new QTreeWidgetItem(item,
                        QStringList()
                        <<"Default"
                        <<""
                        <<"0-end");
    ui->seqTreeWidget->addTopLevelItem(item);
  }
  delete dialog;

}

/**
  Load a sequence from file
  */
void SequenceForm::loadSequenceSlot()
{

}

/**
  Write a sequence out to file
  */
void SequenceForm::saveSequenceSlot()
{

}

/**
  Unload a sequence from memory
  (close the currently selected)

  Close all plots associated with this
  sequence.
  Change all degrees of freedom reading
  from this sequence to constant.
  */
void SequenceForm::closeSequenceSlot()
{
  QTreeWidgetItem* item = ui->seqTreeWidget->currentItem();
  if (!item) return;
  if (item->parent()) item = item->parent();
  delete item;
}

/**
  Apply a new view to an existing sequence
  -- open a dialog asking for the base frame
  */
void SequenceForm::newViewSlot()
{
  QTreeWidgetItem* item = ui->seqTreeWidget->currentItem();
  if (!item) return;
  if (item->parent()) item = item->parent();

  new QTreeWidgetItem(item,
                      QStringList()
                      <<"View");

}

/**
  Close a view if it's selected.
  Close all associated plots and DoFs
  (change them to default view?)
  */
void SequenceForm::removeViewSlot()
{
  QTreeWidgetItem* item = ui->seqTreeWidget->currentItem();
  if (!item) return;
  if (!item->parent()) return;
  if (item->parent()->indexOfChild(item)<=0) return;
  delete item;
}

/**
  Create a new plot of a particular DoF
  */
void SequenceForm::newPlotSlot()
{
  QTreeWidgetItem* item = ui->seqTreeWidget->currentItem();
  if (!item) return;
  if (item->parent()) item=item->parent() ;

/*
  int type = -1;
  QString typeString = item->text(3);
  if (typeString=="Marker") type=0;
  else if (typeString=="Angle") type =1;
  else if (typeString=="Torque") type=2;
  */
  emit newPlot(item->text(0));
}

/**
  We enable/disable buttons based
  on the current selection.

  Perhaps we can highlight the associated
  plots.
  */
void SequenceForm::selectionChangedSlot()
{
  QTreeWidgetItem* item = ui->seqTreeWidget->currentItem();

  if (item) {
    int idx = ui->seqTreeWidget->indexOfTopLevelItem(item);
    if (idx<=0) {
      if (idx<0) {
        idx = item->parent()->indexOfChild(item);
        if (idx>0) setMode(2);
        else setMode(1);
      } else {
        setMode(0);
      }
    } else {
      setMode(1);
    }
  } else {
    setMode(0);
  }
}

/**
  If a sequence has been selected
  for recording, we need to uncheck
  any other sequence of the same type.
  (We don't actually need to do that,
   we could allow multiple sequences
   to record the same data).

  */
void SequenceForm::changeSlot(QTreeWidgetItem* item,int col)
{
  item;
  col;

}

