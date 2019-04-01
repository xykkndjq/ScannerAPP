/********************************************************************************
** Form generated from reading UI file 'ScanMainGUI.ui'
**
** Created by: Qt User Interface Compiler version 5.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SCANMAINGUI_H
#define UI_SCANMAINGUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ScanMainGUI
{
public:

    void setupUi(QWidget *ScanMainGUI)
    {
        if (ScanMainGUI->objectName().isEmpty())
            ScanMainGUI->setObjectName(QStringLiteral("ScanMainGUI"));
        ScanMainGUI->resize(400, 300);

        retranslateUi(ScanMainGUI);

        QMetaObject::connectSlotsByName(ScanMainGUI);
    } // setupUi

    void retranslateUi(QWidget *ScanMainGUI)
    {
        ScanMainGUI->setWindowTitle(QApplication::translate("ScanMainGUI", "ScanMainGUI", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class ScanMainGUI: public Ui_ScanMainGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SCANMAINGUI_H
