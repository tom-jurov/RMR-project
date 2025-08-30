CONFIG -= qt

TEMPLATE = lib
DEFINES += ROBOT_LIBRARY
win32 {
LIBS += -lws2_32
LIBS += -lWinmm
}
CONFIG += c++17
DESTDIR = ../bin
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
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
unix {
    PKGCONFIG += opencv4

INCLUDEPATH += /usr/local/include/opencv4/

    LIBS += -L/usr/local/lib/        \
        -l:libopencv_core.so       \
        -l:libopencv_highgui.so    \
        -l:libopencv_imgcodecs.so  \
        -l:libopencv_imgproc.so    \
        -l:libopencv_features2d.so\
        -l:libopencv_calib3d.so    \
        -l:libopencv_videoio.so    \
        -l:libopencv_ml.so         \
        -l:libopencv_dnn.so        \
        -l:libopencv_flann.so      \
        -l:libopencv_objdetect.so \
        -l:libopencv_photo.so      \
        -l:libopencv_video.so
}
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    CKobuki.cpp \
    lidar_2d_utils.cpp \
    likelihood_filed.cpp \
    mapping_2d.cpp \
    occupancy_map.cpp \
    odometry.cpp \
    robot.cpp \
    rplidar.cpp \
    submap.cpp

HEADERS += \
    CKobuki.h \
    frame.h \
    lidar_2d_utils.h \
    likelihood_filed.h \
    mapping_2d.h \
    math_utils.h \
    occupancy_map.h \
    odometry.h \
    robot_global.h \
    robot.h \
    rplidar.h \
    submap.h \
    szevent.h \
    utility.h

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    priestor.txt
