/********************************************************************************
** Form generated from reading UI file 'TabMainGUI.ui'
**
** Created by: Qt User Interface Compiler version 5.9.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_TABMAINGUI_H
#define UI_TABMAINGUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TabMainGUI
{
public:

    void setupUi(QWidget *TabMainGUI)
    {
        if (TabMainGUI->objectName().isEmpty())
            TabMainGUI->setObjectName(QStringLiteral("TabMainGUI"));
        TabMainGUI->resize(400, 300);

        retranslateUi(TabMainGUI);

        QMetaObject::connectSlotsByName(TabMainGUI);
    } // setupUi

    void retranslateUi(QWidget *TabMainGUI)
    {
        TabMainGUI->setWindowTitle(QApplication::translate("TabMainGUI", "TabMainGUI", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class TabMainGUI: public Ui_TabMainGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TABMAINGUI_H
