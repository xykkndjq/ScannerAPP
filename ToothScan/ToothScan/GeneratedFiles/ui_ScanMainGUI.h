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
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ScanMainGUI
{
public:
    QTextEdit *textEditR;
    QTextEdit *textEditG;
    QTextEdit *textEditB;
    QPushButton *changeBgColorBtn;
    QTextEdit *textEditA;

    void setupUi(QWidget *ScanMainGUI)
    {
        if (ScanMainGUI->objectName().isEmpty())
            ScanMainGUI->setObjectName(QStringLiteral("ScanMainGUI"));
        ScanMainGUI->resize(1920, 1080);
        ScanMainGUI->setMaximumSize(QSize(1920, 1080));
        textEditR = new QTextEdit(ScanMainGUI);
        textEditR->setObjectName(QStringLiteral("textEditR"));
        textEditR->setGeometry(QRect(1330, 80, 104, 71));
        textEditG = new QTextEdit(ScanMainGUI);
        textEditG->setObjectName(QStringLiteral("textEditG"));
        textEditG->setGeometry(QRect(1440, 80, 104, 71));
        textEditB = new QTextEdit(ScanMainGUI);
        textEditB->setObjectName(QStringLiteral("textEditB"));
        textEditB->setGeometry(QRect(1550, 80, 104, 71));
        changeBgColorBtn = new QPushButton(ScanMainGUI);
        changeBgColorBtn->setObjectName(QStringLiteral("changeBgColorBtn"));
        changeBgColorBtn->setGeometry(QRect(1770, 100, 211, 23));
        changeBgColorBtn->setCheckable(false);
        textEditA = new QTextEdit(ScanMainGUI);
        textEditA->setObjectName(QStringLiteral("textEditA"));
        textEditA->setGeometry(QRect(1660, 80, 104, 71));

        retranslateUi(ScanMainGUI);

        QMetaObject::connectSlotsByName(ScanMainGUI);
    } // setupUi

    void retranslateUi(QWidget *ScanMainGUI)
    {
        ScanMainGUI->setWindowTitle(QApplication::translate("ScanMainGUI", "ScanMainGUI", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        ScanMainGUI->setToolTip(QApplication::translate("ScanMainGUI", "\347\241\256\350\256\244\351\200\211\346\213\251", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_TOOLTIP
        changeBgColorBtn->setToolTip(QApplication::translate("ScanMainGUI", "\345\267\246\350\247\206", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        changeBgColorBtn->setText(QApplication::translate("ScanMainGUI", "changeBgColorBtn", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class ScanMainGUI: public Ui_ScanMainGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SCANMAINGUI_H
