#-------------------------------------------------
#
# Project created by QtCreator 2011-07-13T11:16:41
#
#-------------------------------------------------

QT += core gui opengl sql widgets

TARGET = QtVR
TEMPLATE = app

SOURCES += \
    src/main.cpp\
    src/mainwindow.cpp \
    src/myglwidget.cpp \
    src/quatcamera.cpp \
    src/markerwidget.cpp \
    src/simworld.cpp \
    src/CapBody.cpp \
    src/jointwidget.cpp \
    src/bodysizeform.cpp \
    src/markerdata.cpp \
    src/c3ddata.cpp \
    src/jointdataframe.cpp \
    src/markerdataframe.cpp \
    src/mygraphicsscene.cpp \
    src/sequence.cpp \
    src/colorlabel.cpp \
    src/plotform.cpp \
    src/markertab.cpp \
    src/plottab.cpp \
    src/sequencetab.cpp \
    src/sequenceform.cpp \
    src/boarddata.cpp \
    src/PokeDataFrame.cpp \
    src/pokemarkerdata.cpp \
    src/boardmarkerdata.cpp \
    src/dsboxwithparams.cpp \
    src/swingdata.cpp \
    src/livemarkerdata.cpp \
    src/experimentscript.cpp \
    src/scriptbase.cpp \
    src/testscript.cpp

HEADERS += \
    include/mainwindow.h \
    include/myglwidget.h \
    include/quatcamera.h \
    include/markerwidget.h \
    include/simworld.h \
    include/CapBody.h \
    include/jointwidget.h \
    include/bodysizeform.h \
    include/markerdata.h \
    include/c3ddata.h \
    include/jointdataframe.h \
    include/markerdataframe.h \
    include/DataCValues.h \
    include/mygraphicsscene.h \
    include/sequence.h \
    include/colorlabel.h \
    include/plotform.h \
    include/markertab.h \
    include/plottab.h \
    include/sequencetab.h \
    include/sequenceform.h \
    include/boarddata.h \
    include/PokeDataFrame.h \
    include/pokemarkerdata.h \
    include/boardmarkerdata.h \
    include/dsboxwithparams.h \
    include/swingdata.h \
    include/livemarkerdata.h \
    include/experimentscript.h \
    include/scriptbase.h \
    include/testscript.h

FORMS += \
    ui/mainwindow.ui \
    ui/markerwidget.ui \
    ui/jointwidget.ui \
    ui/bodysizeform.ui \
    ui/plotform.ui \
    ui/sequenceform.ui \
    ui/newSequenceDialog.ui

RESOURCES += \
    resources.qrc

INCLUDEPATH += ./include

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
win32:QMAKE_LFLAGS += /STACK:8000000

# On Unix, add specific compiler flags for dependent libraries.
unix:LIBS += -lode -lGL -lglut -lGLU
unix:LIBS += -L./ode/lib
unix:INCLUDES += -I./ode/include
