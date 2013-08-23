#-------------------------------------------------
#
# Project created by QtCreator 2011-07-13T11:16:41
#
#-------------------------------------------------

QT       += core gui opengl sql

TARGET = QtVR
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    myglwidget.cpp \
    quatcamera.cpp \
    markerwidget.cpp \
    simworld.cpp \
    CapBody.cpp \
    jointwidget.cpp \
    bodysizeform.cpp \
    markerdata.cpp \
    c3ddata.cpp \
    jointdataframe.cpp \
    markerdataframe.cpp \
    mygraphicsscene.cpp \
    sequence.cpp \
    colorlabel.cpp \
    plotform.cpp \
    markertab.cpp \
    plottab.cpp \
    sequencetab.cpp \
    sequenceform.cpp \
    boarddata.cpp \
    PokeDataFrame.cpp \
    pokemarkerdata.cpp \
    boardmarkerdata.cpp \
    dsboxwithparams.cpp \
    swingdata.cpp \
    livemarkerdata.cpp \
    experimentscript.cpp \
    scriptbase.cpp \
    testscript.cpp

HEADERS  += mainwindow.h \
    myglwidget.h \
    quatcamera.h \
    markerwidget.h \
    simworld.h \
    CapBody.h \
    jointwidget.h \
    bodysizeform.h \
    markerdata.h \
    c3ddata.h \
    jointdataframe.h \
    markerdataframe.h \
    DataCValues.h \
    mygraphicsscene.h \
    sequence.h \
    colorlabel.h \
    plotform.h \
    markertab.h \
    plottab.h \
    sequencetab.h \
    sequenceform.h \
    boarddata.h \
    PokeDataFrame.h \
    pokemarkerdata.h \
    boardmarkerdata.h \
    dsboxwithparams.h \
    swingdata.h \
    livemarkerdata.h \
    experimentscript.h \
    scriptbase.h \
    testscript.h

FORMS    += mainwindow.ui \
    markerwidget.ui \
    jointwidget.ui \
    bodysizeform.ui \
    plotform.ui \
    sequenceform.ui \
    newSequenceDialog.ui

RESOURCES += \
    resources.qrc

# We use ODE with double precision
DEFINES += dDOUBLE

# In MSVC, we want to turn off some annoying warnings
win32:DEFINES += _CRT_SECURE_NO_WARNINGS

# If we've captured marker data and balance board data
# simultaneously, we'll need BoardData instead of MarkerData
#DEFINES += BOARD_DATA

#Set how many markers to expect
DEFINES += MARKER_COUNT=50

# Telling QT where the ODE code source is allows the creator
# to provide more code completion and source help
win32:INCLUDEPATH += C:/Code/ode_lab/include

# Point to Boost library for the random number generator
win32:INCLUDEPATH += C:/Code/boost_1_46_1

# Link the lab-specific copy of ODE
win32:LIBS += C:/Code/ode_lab/lib/ReleaseDoubleLib/ode_double.lib

# This linker flag is for MSVC.  It makes the stack larger.
# This might not be necessary any more, but old versions of
# ODE were prone to overflow the stack.
QMAKE_LFLAGS += /STACK:8000000
