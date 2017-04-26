#-------------------------------------------------
#
# Project created by QtCreator 2017-04-13T09:57:47
#
#-------------------------------------------------

QT += gui

HEADERS += include/transmem.h \
    include/transformationbuffer.h \
    include/frameandlink.h \
    include/typedefs.h \
    include/stampedtransformation.h \
    include/gmlwriter.h

SOURCES += src/transmem.cpp \
    src/transformationbuffer.cpp \
    src/frameandlink.cpp \
    src/stampedtransformation.cpp \
    src/gmlwriter.cpp

INCLUDEPATH += include

 target.path = /usr/lib
 INSTALLS += target


test{

    QT += testlib

    HEADERS += test/include/transmemTest.h
    SOURCES += test/src/transmemTest.cpp


    TEMPLATE = app
    TARGET = myapptests

}
else{

    TEMPLATE = lib
    TARGET = transmem
    CONFIG += staticlib
}



# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
