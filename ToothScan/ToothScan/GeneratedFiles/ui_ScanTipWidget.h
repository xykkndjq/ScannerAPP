/********************************************************************************
** Form generated from reading UI file 'ScanTipWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.9.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SCANTIPWIDGET_H
#define UI_SCANTIPWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ScanTipWidget
{
public:

    void setupUi(QWidget *ScanTipWidget)
    {
        if (ScanTipWidget->objectName().isEmpty())
            ScanTipWidget->setObjectName(QStringLiteral("ScanTipWidget"));
        ScanTipWidget->resize(400, 300);

        retranslateUi(ScanTipWidget);

        QMetaObject::connectSlotsByName(ScanTipWidget);
    } // setupUi

    void retranslateUi(QWidget *ScanTipWidget)
    {
        ScanTipWidget->setWindowTitle(QApplication::translate("ScanTipWidget", "ScanTipWidget", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class ScanTipWidget: public Ui_ScanTipWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SCANTIPWIDGET_H
