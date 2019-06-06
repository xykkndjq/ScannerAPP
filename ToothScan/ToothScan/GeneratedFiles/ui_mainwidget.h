/********************************************************************************
** Form generated from reading UI file 'mainwidget.ui'
**
** Created by: Qt User Interface Compiler version 5.9.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWIDGET_H
#define UI_MAINWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWidgetClass
{
public:

    void setupUi(QWidget *MainWidgetClass)
    {
        if (MainWidgetClass->objectName().isEmpty())
            MainWidgetClass->setObjectName(QStringLiteral("MainWidgetClass"));
        MainWidgetClass->resize(600, 400);

        retranslateUi(MainWidgetClass);

        QMetaObject::connectSlotsByName(MainWidgetClass);
    } // setupUi

    void retranslateUi(QWidget *MainWidgetClass)
    {
        MainWidgetClass->setWindowTitle(QApplication::translate("MainWidgetClass", "MainWidget", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWidgetClass: public Ui_MainWidgetClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWIDGET_H
