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
#include "plottab.h"
#include <QVBoxLayout>
#include <QPushButton>

#include "plotform.h"
#include "sequenceform.h"

PlotTab::PlotTab(QWidget *parent) :
    QScrollArea(parent)
{
  layout = new QVBoxLayout;
  //layout->setContentsMargins(0,0,0,0);


  //addButton = new QPushButton("Add New");
  //layout->addWidget(addButton);
  //connect(addButton,SIGNAL(clicked()),this,SLOT(addNewPlot()));

  seqForm = new SequenceForm();
  layout->addWidget(seqForm);

  connect(seqForm,SIGNAL(newPlot(QString)),this,SLOT(newPlotSlot(QString)));



  layout->addStretch();




  QWidget* mainBody = new QWidget();
  mainBody->setLayout(layout);
  setWidgetResizable(true);
  this->setWidget(mainBody);

}

void PlotTab::addNewPlot()
{
  PlotForm* plot = new PlotForm;

  int cnt = layout->count();
  layout->insertWidget(cnt-1,plot);

  //hide();

  //plot->show();
  //connect(plot,SIGNAL(destroyed(QObject*)),this,SLOT(removePlot(QObject*)));
}

void PlotTab::newPlotSlot(QString seq)
{
  PlotForm* plot = new PlotForm;
  // What we really need to pass is a sequence
  //  We can associate a treeWidgetItem
  //  with any sequence.
  //  Then, if the widget item is changed
  //  or deleted, we can be connected to that signal.


  plot->setLabel(seq);

  int cnt = layout->count();
  layout->insertWidget(cnt-1,plot);

}

//void PlotTab::removePlot(QObject* plot)
//{
//  plots.removeOne(plot);
//}
