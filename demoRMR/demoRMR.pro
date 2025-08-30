#-------------------------------------------------
#
# Project created by QtCreator 2018-02-11T14:35:38
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
include ($$PWD/../QJoysticks-master/QJoysticks.pri)
TARGET = demoRMR
TEMPLATE = app
win32 {
LIBS += -lws2_32
LIBS += -lWinmm
}
INCLUDEPATH += ../robot
LIBS += -L../bin -lrobot
    INCLUDEPATH += C:/opencv/include/

# QMAKE_CXXFLAGS += -fsanitize=address
# QMAKE_LFLAGS += -fsanitize=address
INCLUDEPATH += "C:/Program Files/PCL 1.14.1/3rdParty/Eigen3/include/eigen3"
win32 {
    INCLUDEPATH += C:/opencv_vc17/include/

    LIBS +=-LC:/opencv_vc17/x64/vc17/bin
    LIBS +=-LC:/opencv_vc17/x64/vc17/lib
    CONFIG(debug, debug|release) {
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_core4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_highgui4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_imgcodecs4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_imgproc4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_features2d4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_calib3d4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_videoio4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_ml4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_dnn4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_flann4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_objdetect4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_photo4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_shape4100d
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_video4100d
    }
    else {
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_core4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_highgui4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_imgcodecs4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_imgproc4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_features2d4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_calib3d4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_videoio4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_ml4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_dnn4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_flann4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_objdetect4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_photo4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_shape4100
        LIBS +=-LC:/opencv_vc17/x64/vc17/lib -lopencv_video4100
    }
}
#DEFINES += QT_FORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += main.cpp\
        mainwindow.cpp \
        visualizer.cpp

HEADERS  += mainwindow.h \
    visualizer.h

FORMS    += mainwindow.ui
