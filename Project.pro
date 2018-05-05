QT += widgets
CONFIG += warn_off
SOURCES += main.cpp
HEADERS += QModel.h
HEADERS += QDefine.h
HEADERS += QViewer.h 
HEADERS += QEnergy.h
HEADERS += QSketch.h
HEADERS += QCanvas.h 
HEADERS += QWindow.h
HEADERS += QProblem.h
HEADERS += QAnalyzer.h
HEADERS += QOptimizer.h
HEADERS += QStanMath.h
HEADERS += QPolygons.hpp
SOURCES += QModel.cpp
SOURCES += QViewer.cpp 
SOURCES += QEnergy.cpp
SOURCES += QSketch.cpp
SOURCES += QCanvas.cpp 
SOURCES += QWindow.cpp 
SOURCES += QProblem.cpp
SOURCES += QAnalyzer.cpp
SOURCES += QOptimizer.cpp
INCLUDEPATH += QOpenGL
INCLUDEPATH += C:\stan-math
INCLUDEPATH += C:\stan-math\include
SOURCES += QOpenGL/QOpenGLMesh.cpp
SOURCES += QOpenGL/QOpenGLVertex.cpp
SOURCES += QOpenGL/QOpenGLObject.cpp
SOURCES += QOpenGL/QOpenGLPolygon.cpp
SOURCES += QOpenGL/QOpenGLWindow.cpp
INCLUDEPATH += C:\stan-math\lib\eigen_3.2.9
INCLUDEPATH += C:\stan-math\lib\boost_1.62.0
INCLUDEPATH += C:\stan-math\include\cppoptlib
INCLUDEPATH += C:\stan-math\lib\cvodes_2.9.0\include
