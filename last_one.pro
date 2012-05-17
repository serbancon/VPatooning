#-------------------------------------------------
#
# Project created by QtCreator 2011-08-25T00:05:16
#
#-------------------------------------------------

QT       += core network

QT       -= gui

TARGET = last_one
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp



LIBS     +=-lopencv_core -lopencv_highgui
