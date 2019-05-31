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
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_TabMainGUI
{
public:
    QGroupBox *orderInforGroupBox;
    QGroupBox *orderGroupBox;
    QPushButton *newPushButton;
    QPushButton *openPushButton;
    QPushButton *watchPushButton;
    QGroupBox *orderDateGroupBox;
    QLineEdit *orderDateLineEdit;
    QLabel *orderDateLabel;
    QLabel *orderNumLabel;
    QGroupBox *orderNumGroupBox;
    QLineEdit *orderNumLineEdit;
    QLabel *patientNameLabel;
    QGroupBox *patientNameGroupBox;
    QLineEdit *patientNameLineEdit;
    QLabel *doctorNameLabel;
    QGroupBox *doctorNameGroupBox;
    QLineEdit *doctorNameLineEdit;
    QLabel *operatorNameLabel;
    QGroupBox *operatorNameGroupBox;
    QLineEdit *operatorNameLineEdit;
    QLabel *additionLabel;
    QTextEdit *additionTextEdit;
    QPushButton *savePushButton;
    QGroupBox *orderScanGroupBox;
    QPushButton *spliteModelPushButton;
    QPushButton *moulagePushButton;
    QGroupBox *unModelGroupBox;
    QPushButton *upperJawPushButton;
    QPushButton *lowerJawPushButton;
    QPushButton *unModelPushButton;
    QPushButton *scanPushButton;
    QGroupBox *moulageGroupBox;
    QPushButton *moulagePushButton_2;
    QPushButton *moulagePushButton_1;
    QPushButton *moulagePushButton_3;
    QGroupBox *spliteModelGroupBox;
    QPushButton *clearAllButton;
    QGroupBox *mainGroupBox;
    QLabel *logoLabel;
    QPushButton *btnClose;
    QGroupBox *settingGroupBox;
    QLabel *textureLabel;
    QLabel *calibrationPointLabel;
    QLabel *jawFrameLabel;
    QLabel *saveSpliteModelLabel;
    QLabel *scanpathLabel;
    QGroupBox *scanPathGroupBox;
    QLineEdit *scanPathLineEdit;
    QPushButton *choosePathPushButton;
    QPushButton *saveSpliteModelCheckBox;
    QPushButton *jawFrameCheckBox;
    QPushButton *calibrationPointCheckBox;
    QPushButton *textureCheckBox;
    QGroupBox *calibrationGroupBox;
    QPushButton *calibrationPushButton;
    QLabel *leftCameraLable;
    QLabel *rightCameraLable;
    QGroupBox *aboutGroupBox;
    QLabel *aboutImageLabel;
    QLabel *aboutTextLabel;

    void setupUi(QWidget *TabMainGUI)
    {
        if (TabMainGUI->objectName().isEmpty())
            TabMainGUI->setObjectName(QStringLiteral("TabMainGUI"));
        TabMainGUI->resize(2222, 1011);
        orderInforGroupBox = new QGroupBox(TabMainGUI);
        orderInforGroupBox->setObjectName(QStringLiteral("orderInforGroupBox"));
        orderInforGroupBox->setEnabled(true);
        orderInforGroupBox->setGeometry(QRect(0, 100, 1920, 1180));
        orderInforGroupBox->setStyleSheet(QStringLiteral("background-color: rgba(255, 255, 255, 255);"));
        orderGroupBox = new QGroupBox(orderInforGroupBox);
        orderGroupBox->setObjectName(QStringLiteral("orderGroupBox"));
        orderGroupBox->setGeometry(QRect(0, 0, 470, 980));
        orderGroupBox->setStyleSheet(QStringLiteral("background-color: rgba(255, 255, 255, 255);"));
        newPushButton = new QPushButton(orderGroupBox);
        newPushButton->setObjectName(QStringLiteral("newPushButton"));
        newPushButton->setGeometry(QRect(25, 45, 97, 48));
        newPushButton->setStyleSheet(QLatin1String("background-color: rgba(40, 138, 237, 255);\n"
"color: rgb(255, 255, 255);\n"
"border:0px groove gray;border-radius:5px;"));
        QIcon icon;
        icon.addFile(QStringLiteral(":/MainWidget/Resources/images/new.png"), QSize(), QIcon::Normal, QIcon::Off);
        newPushButton->setIcon(icon);
        newPushButton->setIconSize(QSize(22, 22));
        openPushButton = new QPushButton(orderGroupBox);
        openPushButton->setObjectName(QStringLiteral("openPushButton"));
        openPushButton->setGeometry(QRect(132, 45, 97, 48));
        openPushButton->setStyleSheet(QLatin1String("background-color: rgba(40, 138, 237, 255);\n"
"color: rgb(255, 255, 255);\n"
"border:0px groove gray;border-radius:5px;"));
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/MainWidget/Resources/images/open.png"), QSize(), QIcon::Normal, QIcon::Off);
        openPushButton->setIcon(icon1);
        openPushButton->setIconSize(QSize(22, 22));
        watchPushButton = new QPushButton(orderGroupBox);
        watchPushButton->setObjectName(QStringLiteral("watchPushButton"));
        watchPushButton->setGeometry(QRect(239, 45, 97, 48));
        watchPushButton->setStyleSheet(QLatin1String("background-color: rgba(40, 138, 237, 255);\n"
"color: rgb(255, 255, 255);\n"
"border:0px groove gray;border-radius:5px;"));
        QIcon icon2;
        icon2.addFile(QStringLiteral(":/MainWidget/Resources/images/preview.png"), QSize(), QIcon::Normal, QIcon::Off);
        watchPushButton->setIcon(icon2);
        watchPushButton->setIconSize(QSize(22, 22));
        orderDateGroupBox = new QGroupBox(orderGroupBox);
        orderDateGroupBox->setObjectName(QStringLiteral("orderDateGroupBox"));
        orderDateGroupBox->setGeometry(QRect(43, 177, 389, 55));
        orderDateGroupBox->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 255);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        orderDateLineEdit = new QLineEdit(orderDateGroupBox);
        orderDateLineEdit->setObjectName(QStringLiteral("orderDateLineEdit"));
        orderDateLineEdit->setGeometry(QRect(20, 12, 291, 30));
        orderDateLineEdit->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(160, 160, 160);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        orderDateLabel = new QLabel(orderGroupBox);
        orderDateLabel->setObjectName(QStringLiteral("orderDateLabel"));
        orderDateLabel->setGeometry(QRect(43, 146, 71, 21));
        orderDateLabel->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(102, 105, 120);"));
        orderNumLabel = new QLabel(orderGroupBox);
        orderNumLabel->setObjectName(QStringLiteral("orderNumLabel"));
        orderNumLabel->setGeometry(QRect(43, 258, 71, 16));
        orderNumLabel->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(102, 105, 120);"));
        orderNumGroupBox = new QGroupBox(orderGroupBox);
        orderNumGroupBox->setObjectName(QStringLiteral("orderNumGroupBox"));
        orderNumGroupBox->setGeometry(QRect(43, 289, 389, 55));
        orderNumGroupBox->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 255);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        orderNumLineEdit = new QLineEdit(orderNumGroupBox);
        orderNumLineEdit->setObjectName(QStringLiteral("orderNumLineEdit"));
        orderNumLineEdit->setGeometry(QRect(20, 12, 291, 30));
        orderNumLineEdit->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(160, 160, 160);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        patientNameLabel = new QLabel(orderGroupBox);
        patientNameLabel->setObjectName(QStringLiteral("patientNameLabel"));
        patientNameLabel->setGeometry(QRect(43, 370, 71, 16));
        patientNameLabel->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(102, 105, 120);"));
        patientNameGroupBox = new QGroupBox(orderGroupBox);
        patientNameGroupBox->setObjectName(QStringLiteral("patientNameGroupBox"));
        patientNameGroupBox->setGeometry(QRect(40, 401, 389, 55));
        patientNameGroupBox->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 255);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        patientNameLineEdit = new QLineEdit(patientNameGroupBox);
        patientNameLineEdit->setObjectName(QStringLiteral("patientNameLineEdit"));
        patientNameLineEdit->setGeometry(QRect(20, 12, 291, 30));
        patientNameLineEdit->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(160, 160, 160);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        doctorNameLabel = new QLabel(orderGroupBox);
        doctorNameLabel->setObjectName(QStringLiteral("doctorNameLabel"));
        doctorNameLabel->setGeometry(QRect(43, 482, 71, 16));
        doctorNameLabel->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(102, 105, 120);"));
        doctorNameGroupBox = new QGroupBox(orderGroupBox);
        doctorNameGroupBox->setObjectName(QStringLiteral("doctorNameGroupBox"));
        doctorNameGroupBox->setGeometry(QRect(37, 513, 389, 55));
        doctorNameGroupBox->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 255);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        doctorNameLineEdit = new QLineEdit(doctorNameGroupBox);
        doctorNameLineEdit->setObjectName(QStringLiteral("doctorNameLineEdit"));
        doctorNameLineEdit->setGeometry(QRect(20, 12, 291, 30));
        doctorNameLineEdit->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(160, 160, 160);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        operatorNameLabel = new QLabel(orderGroupBox);
        operatorNameLabel->setObjectName(QStringLiteral("operatorNameLabel"));
        operatorNameLabel->setGeometry(QRect(43, 594, 71, 16));
        operatorNameLabel->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(102, 105, 120);"));
        operatorNameGroupBox = new QGroupBox(orderGroupBox);
        operatorNameGroupBox->setObjectName(QStringLiteral("operatorNameGroupBox"));
        operatorNameGroupBox->setGeometry(QRect(40, 625, 389, 55));
        operatorNameGroupBox->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 255);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        operatorNameLineEdit = new QLineEdit(operatorNameGroupBox);
        operatorNameLineEdit->setObjectName(QStringLiteral("operatorNameLineEdit"));
        operatorNameLineEdit->setGeometry(QRect(20, 12, 291, 30));
        operatorNameLineEdit->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(160, 160, 160);\n"
"border:0px groove gray;border-radius:5px;"));
        additionLabel = new QLabel(orderGroupBox);
        additionLabel->setObjectName(QStringLiteral("additionLabel"));
        additionLabel->setGeometry(QRect(43, 706, 71, 16));
        additionLabel->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(102, 105, 120);"));
        additionTextEdit = new QTextEdit(orderGroupBox);
        additionTextEdit->setObjectName(QStringLiteral("additionTextEdit"));
        additionTextEdit->setGeometry(QRect(43, 740, 389, 171));
        additionTextEdit->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 255);\n"
"border:0px groove gray;border-radius:5px;\n"
"color: rgb(160, 160, 160);\n"
""));
        savePushButton = new QPushButton(orderGroupBox);
        savePushButton->setObjectName(QStringLiteral("savePushButton"));
        savePushButton->setGeometry(QRect(346, 45, 97, 48));
        savePushButton->setStyleSheet(QLatin1String("background-color: rgba(40, 138, 237, 255);\n"
"color: rgb(255, 255, 255);\n"
"border:0px groove gray;border-radius:5px;"));
        QIcon icon3;
        icon3.addFile(QStringLiteral(":/MainWidget/Resources/images/save.png"), QSize(), QIcon::Normal, QIcon::Off);
        savePushButton->setIcon(icon3);
        savePushButton->setIconSize(QSize(22, 22));
        orderScanGroupBox = new QGroupBox(orderInforGroupBox);
        orderScanGroupBox->setObjectName(QStringLiteral("orderScanGroupBox"));
        orderScanGroupBox->setGeometry(QRect(470, 0, 1450, 980));
        orderScanGroupBox->setStyleSheet(QStringLiteral("background-color: rgba(250, 250, 252,255);"));
        spliteModelPushButton = new QPushButton(orderScanGroupBox);
        spliteModelPushButton->setObjectName(QStringLiteral("spliteModelPushButton"));
        spliteModelPushButton->setGeometry(QRect(626, 43, 200, 50));
        spliteModelPushButton->setStyleSheet(QLatin1String("QPushButton{background-color: rgba(255, 255, 255, 255); color: rgba(40, 138, 237, 255); \n"
"border: 0px;border-style: outset;}\n"
"QPushButton:hover{background-color: rgba(40, 138, 237, 255); color:rgba(255, 255, 255, 255);}\n"
"QPushButton:checked{background-color: rgba(40, 138, 237, 255); color:rgba(255, 255, 255, 255); border-style: inset; }"));
        spliteModelPushButton->setIconSize(QSize(22, 22));
        spliteModelPushButton->setCheckable(true);
        moulagePushButton = new QPushButton(orderScanGroupBox);
        moulagePushButton->setObjectName(QStringLiteral("moulagePushButton"));
        moulagePushButton->setGeometry(QRect(827, 43, 200, 50));
        moulagePushButton->setStyleSheet(QLatin1String("QPushButton{background-color: rgba(255, 255, 255, 255); color: rgba(40, 138, 237, 255); \n"
"border: 0px;border-style: outset;}\n"
"QPushButton:hover{background-color: rgba(40, 138, 237, 255); color:rgba(255, 255, 255, 255);}\n"
"QPushButton:checked{background-color: rgba(40, 138, 237, 255); color:rgba(255, 255, 255, 255); border-style: inset; }"));
        moulagePushButton->setIconSize(QSize(22, 22));
        moulagePushButton->setCheckable(true);
        unModelGroupBox = new QGroupBox(orderScanGroupBox);
        unModelGroupBox->setObjectName(QStringLiteral("unModelGroupBox"));
        unModelGroupBox->setGeometry(QRect(425, 100, 600, 660));
        unModelGroupBox->setStyleSheet(QStringLiteral("border: none;  "));
        upperJawPushButton = new QPushButton(unModelGroupBox);
        upperJawPushButton->setObjectName(QStringLiteral("upperJawPushButton"));
        upperJawPushButton->setGeometry(QRect(60, 160, 500, 185));
        upperJawPushButton->setAutoFillBackground(false);
        upperJawPushButton->setStyleSheet(QLatin1String("QPushButton{border-image: url(:/MainWidget/Resources/images/upperjaw_no.png);}\n"
"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/upperjaw_yes.png);}\n"
"QPushButton:checked{border-image: url(:/MainWidget/Resources/images/upperjaw_yes.png);}"));
        upperJawPushButton->setCheckable(true);
        upperJawPushButton->setFlat(true);
        lowerJawPushButton = new QPushButton(unModelGroupBox);
        lowerJawPushButton->setObjectName(QStringLiteral("lowerJawPushButton"));
        lowerJawPushButton->setGeometry(QRect(90, 330, 448, 153));
        lowerJawPushButton->setAutoFillBackground(false);
        lowerJawPushButton->setStyleSheet(QLatin1String("QPushButton{border-image: url(:/MainWidget/Resources/images/lowerjaw_no.png);}\n"
"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/lowerjaw_yes.png);}\n"
"QPushButton:checked{border-image: url(:/MainWidget/Resources/images/lowerjaw_yes.png);}"));
        lowerJawPushButton->setCheckable(true);
        lowerJawPushButton->raise();
        upperJawPushButton->raise();
        unModelPushButton = new QPushButton(orderScanGroupBox);
        unModelPushButton->setObjectName(QStringLiteral("unModelPushButton"));
        unModelPushButton->setGeometry(QRect(425, 43, 200, 50));
        unModelPushButton->setStyleSheet(QLatin1String("QPushButton{background-color: rgba(255, 255, 255, 255); color: rgba(40, 138, 237, 255); \n"
"border: 0px;border-style: outset;}\n"
"QPushButton:hover{background-color: rgba(40, 138, 237, 255); color:rgba(255, 255, 255, 255);}\n"
"QPushButton:checked{background-color: rgba(40, 138, 237, 255); color:rgba(255, 255, 255, 255); border-style: inset; }"));
        unModelPushButton->setIconSize(QSize(22, 22));
        unModelPushButton->setCheckable(true);
        unModelPushButton->setFlat(false);
        scanPushButton = new QPushButton(orderScanGroupBox);
        scanPushButton->setObjectName(QStringLiteral("scanPushButton"));
        scanPushButton->setGeometry(QRect(598, 810, 255, 48));
        scanPushButton->setStyleSheet(QLatin1String("background-color: rgba(40, 138, 237, 255);\n"
"color: rgb(255, 255, 255);\n"
"border:0px groove gray;border-radius:5px;"));
        QIcon icon4;
        icon4.addFile(QStringLiteral("../../../../../QTProject/qtwidgettest11/qtwidgettest/qtwidgettest/Resources/images/scan.png"), QSize(), QIcon::Normal, QIcon::Off);
        scanPushButton->setIcon(icon4);
        scanPushButton->setIconSize(QSize(22, 22));
        moulageGroupBox = new QGroupBox(orderScanGroupBox);
        moulageGroupBox->setObjectName(QStringLiteral("moulageGroupBox"));
        moulageGroupBox->setGeometry(QRect(425, 100, 600, 660));
        moulageGroupBox->setStyleSheet(QStringLiteral("border: none;  "));
        moulagePushButton_2 = new QPushButton(moulageGroupBox);
        moulagePushButton_2->setObjectName(QStringLiteral("moulagePushButton_2"));
        moulagePushButton_2->setGeometry(QRect(125, 220, 359, 182));
        moulagePushButton_2->setAutoFillBackground(false);
        moulagePushButton_2->setStyleSheet(QLatin1String("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage2_no.png);}\n"
"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/moulage2_yes.png);}\n"
"QPushButton:checked{border-image: url(:/MainWidget/Resources/images/moulage2_yes.png);}"));
        moulagePushButton_2->setCheckable(true);
        moulagePushButton_2->setFlat(true);
        moulagePushButton_1 = new QPushButton(moulageGroupBox);
        moulagePushButton_1->setObjectName(QStringLiteral("moulagePushButton_1"));
        moulagePushButton_1->setGeometry(QRect(125, 10, 359, 182));
        moulagePushButton_1->setAutoFillBackground(false);
        moulagePushButton_1->setStyleSheet(QLatin1String("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage1_no.png);}\n"
"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/moulage1_yes.png);}\n"
"QPushButton:checked{border-image: url(:/MainWidget/Resources/images/moulage1_yes.png);}"));
        moulagePushButton_1->setCheckable(true);
        moulagePushButton_1->setFlat(true);
        moulagePushButton_3 = new QPushButton(moulageGroupBox);
        moulagePushButton_3->setObjectName(QStringLiteral("moulagePushButton_3"));
        moulagePushButton_3->setGeometry(QRect(125, 430, 359, 182));
        moulagePushButton_3->setAutoFillBackground(false);
        moulagePushButton_3->setStyleSheet(QLatin1String("QPushButton{border-image: url(:/MainWidget/Resources/images/moulage3_no.png);}\n"
"QPushButton:hover{border-image: url(:/MainWidget/Resources/images/moulage3_yes.png);}\n"
"QPushButton:checked{border-image: url(:/MainWidget/Resources/images/moulage3_yes.png);}"));
        moulagePushButton_3->setCheckable(true);
        moulagePushButton_3->setFlat(true);
        moulagePushButton_2->raise();
        moulagePushButton_3->raise();
        moulagePushButton_1->raise();
        spliteModelGroupBox = new QGroupBox(orderScanGroupBox);
        spliteModelGroupBox->setObjectName(QStringLiteral("spliteModelGroupBox"));
        spliteModelGroupBox->setGeometry(QRect(400, 100, 700, 660));
        spliteModelGroupBox->setStyleSheet(QStringLiteral("border: none;  "));
        clearAllButton = new QPushButton(spliteModelGroupBox);
        clearAllButton->setObjectName(QStringLiteral("clearAllButton"));
        clearAllButton->setGeometry(QRect(140, 280, 100, 38));
        clearAllButton->setStyleSheet(QLatin1String("background-color: rgba(40, 138, 237, 255);\n"
"color: rgb(255, 255, 255);\n"
"border:0px groove gray;border-radius:5px;"));
        clearAllButton->setIconSize(QSize(22, 22));
        spliteModelPushButton->raise();
        moulagePushButton->raise();
        unModelPushButton->raise();
        scanPushButton->raise();
        spliteModelGroupBox->raise();
        moulageGroupBox->raise();
        unModelGroupBox->raise();
        mainGroupBox = new QGroupBox(TabMainGUI);
        mainGroupBox->setObjectName(QStringLiteral("mainGroupBox"));
        mainGroupBox->setGeometry(QRect(0, 0, 1920, 100));
        mainGroupBox->setStyleSheet(QLatin1String("background-color: rgba(255, 255, 255, 255);\n"
"border:0px;"));
        logoLabel = new QLabel(mainGroupBox);
        logoLabel->setObjectName(QStringLiteral("logoLabel"));
        logoLabel->setGeometry(QRect(38, 38, 172, 39));
        logoLabel->setPixmap(QPixmap(QString::fromUtf8(":/MainWidget/Resources/images/logo.png")));
        btnClose = new QPushButton(mainGroupBox);
        btnClose->setObjectName(QStringLiteral("btnClose"));
        btnClose->setGeometry(QRect(1870, 40, 14, 14));
        btnClose->setStyleSheet(QStringLiteral("border-image: url(:/MainWidget/Resources/images/btnClose.png);"));
        settingGroupBox = new QGroupBox(TabMainGUI);
        settingGroupBox->setObjectName(QStringLiteral("settingGroupBox"));
        settingGroupBox->setGeometry(QRect(530, 99, 775, 500));
        settingGroupBox->setStyleSheet(QLatin1String("background-color: rgba(255, 255, 255, 255);\n"
"border-radius:5px;"));
        textureLabel = new QLabel(settingGroupBox);
        textureLabel->setObjectName(QStringLiteral("textureLabel"));
        textureLabel->setGeometry(QRect(150, 53, 54, 34));
        calibrationPointLabel = new QLabel(settingGroupBox);
        calibrationPointLabel->setObjectName(QStringLiteral("calibrationPointLabel"));
        calibrationPointLabel->setGeometry(QRect(150, 125, 54, 34));
        jawFrameLabel = new QLabel(settingGroupBox);
        jawFrameLabel->setObjectName(QStringLiteral("jawFrameLabel"));
        jawFrameLabel->setGeometry(QRect(150, 197, 54, 34));
        saveSpliteModelLabel = new QLabel(settingGroupBox);
        saveSpliteModelLabel->setObjectName(QStringLiteral("saveSpliteModelLabel"));
        saveSpliteModelLabel->setGeometry(QRect(150, 269, 81, 34));
        scanpathLabel = new QLabel(settingGroupBox);
        scanpathLabel->setObjectName(QStringLiteral("scanpathLabel"));
        scanpathLabel->setGeometry(QRect(53, 406, 81, 34));
        scanPathGroupBox = new QGroupBox(settingGroupBox);
        scanPathGroupBox->setObjectName(QStringLiteral("scanPathGroupBox"));
        scanPathGroupBox->setGeometry(QRect(151, 395, 389, 55));
        scanPathGroupBox->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 255);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        scanPathLineEdit = new QLineEdit(scanPathGroupBox);
        scanPathLineEdit->setObjectName(QStringLiteral("scanPathLineEdit"));
        scanPathLineEdit->setGeometry(QRect(20, 12, 291, 30));
        scanPathLineEdit->setStyleSheet(QLatin1String("background-color:rgba(246, 246, 248, 0);\n"
"color: rgb(160, 160, 160);\n"
"border:0px groove gray;border-radius:5px;\n"
""));
        choosePathPushButton = new QPushButton(settingGroupBox);
        choosePathPushButton->setObjectName(QStringLiteral("choosePathPushButton"));
        choosePathPushButton->setGeometry(QRect(570, 397, 115, 50));
        choosePathPushButton->setStyleSheet(QLatin1String("background-color: rgba(40, 138, 237, 255);\n"
"color: rgb(255, 255, 255);\n"
"border:0px groove gray;border-radius:5px;"));
        choosePathPushButton->setIconSize(QSize(22, 22));
        saveSpliteModelCheckBox = new QPushButton(settingGroupBox);
        saveSpliteModelCheckBox->setObjectName(QStringLiteral("saveSpliteModelCheckBox"));
        saveSpliteModelCheckBox->setGeometry(QRect(510, 260, 76, 34));
        saveSpliteModelCheckBox->setStyleSheet(QLatin1String("QPushButton:checked{border-image: url(:/MainWidget/Resources/images/checked.png); }\n"
"QPushButton{border-image: url(:/MainWidget/Resources/images/uncheck.png); }"));
        saveSpliteModelCheckBox->setCheckable(true);
        saveSpliteModelCheckBox->setChecked(false);
        jawFrameCheckBox = new QPushButton(settingGroupBox);
        jawFrameCheckBox->setObjectName(QStringLiteral("jawFrameCheckBox"));
        jawFrameCheckBox->setGeometry(QRect(510, 188, 76, 34));
        jawFrameCheckBox->setStyleSheet(QLatin1String("QPushButton:checked{border-image: url(:/MainWidget/Resources/images/checked.png); }\n"
"QPushButton{border-image: url(:/MainWidget/Resources/images/uncheck.png); }"));
        jawFrameCheckBox->setCheckable(true);
        jawFrameCheckBox->setChecked(false);
        calibrationPointCheckBox = new QPushButton(settingGroupBox);
        calibrationPointCheckBox->setObjectName(QStringLiteral("calibrationPointCheckBox"));
        calibrationPointCheckBox->setGeometry(QRect(510, 116, 76, 34));
        calibrationPointCheckBox->setStyleSheet(QLatin1String("QPushButton:checked{border-image: url(:/MainWidget/Resources/images/checked.png); }\n"
"QPushButton{border-image: url(:/MainWidget/Resources/images/uncheck.png); }"));
        calibrationPointCheckBox->setCheckable(true);
        calibrationPointCheckBox->setChecked(false);
        textureCheckBox = new QPushButton(settingGroupBox);
        textureCheckBox->setObjectName(QStringLiteral("textureCheckBox"));
        textureCheckBox->setGeometry(QRect(510, 51, 76, 34));
        textureCheckBox->setStyleSheet(QLatin1String("QPushButton:checked{border-image: url(:/MainWidget/Resources/images/checked.png); }\n"
"QPushButton{border-image: url(:/MainWidget/Resources/images/uncheck.png); }"));
        textureCheckBox->setCheckable(true);
        textureCheckBox->setChecked(false);
        calibrationGroupBox = new QGroupBox(TabMainGUI);
        calibrationGroupBox->setObjectName(QStringLiteral("calibrationGroupBox"));
        calibrationGroupBox->setGeometry(QRect(0, 100, 1920, 1080));
        calibrationGroupBox->setStyleSheet(QStringLiteral("background-color: rgba(250, 250, 252,255);"));
        calibrationPushButton = new QPushButton(calibrationGroupBox);
        calibrationPushButton->setObjectName(QStringLiteral("calibrationPushButton"));
        calibrationPushButton->setGeometry(QRect(831, 82, 255, 48));
        calibrationPushButton->setStyleSheet(QLatin1String("background-color: rgba(40, 138, 237, 255);\n"
"color: rgb(255, 255, 255);\n"
"border:0px groove gray;border-radius:5px;"));
        leftCameraLable = new QLabel(calibrationGroupBox);
        leftCameraLable->setObjectName(QStringLiteral("leftCameraLable"));
        leftCameraLable->setGeometry(QRect(110, 180, 806, 668));
        leftCameraLable->setStyleSheet(QStringLiteral("background-color: rgb(217, 217, 217);"));
        rightCameraLable = new QLabel(calibrationGroupBox);
        rightCameraLable->setObjectName(QStringLiteral("rightCameraLable"));
        rightCameraLable->setGeometry(QRect(1016, 180, 806, 668));
        rightCameraLable->setStyleSheet(QStringLiteral("background-color: rgb(217, 217, 217);"));
        aboutGroupBox = new QGroupBox(TabMainGUI);
        aboutGroupBox->setObjectName(QStringLiteral("aboutGroupBox"));
        aboutGroupBox->setGeometry(QRect(0, 100, 1920, 1180));
        aboutGroupBox->setStyleSheet(QStringLiteral("background-color: rgba(250, 250, 252,255);"));
        aboutImageLabel = new QLabel(aboutGroupBox);
        aboutImageLabel->setObjectName(QStringLiteral("aboutImageLabel"));
        aboutImageLabel->setGeometry(QRect(850, 335, 234, 51));
        aboutImageLabel->setPixmap(QPixmap(QString::fromUtf8(":/MainWidget/Resources/images/logo.png")));
        aboutImageLabel->setScaledContents(true);
        aboutTextLabel = new QLabel(aboutGroupBox);
        aboutTextLabel->setObjectName(QStringLiteral("aboutTextLabel"));
        aboutTextLabel->setGeometry(QRect(560, 432, 671, 16));
        settingGroupBox->raise();
        aboutGroupBox->raise();
        calibrationGroupBox->raise();
        mainGroupBox->raise();
        orderInforGroupBox->raise();

        retranslateUi(TabMainGUI);

        QMetaObject::connectSlotsByName(TabMainGUI);
    } // setupUi

    void retranslateUi(QWidget *TabMainGUI)
    {
        TabMainGUI->setWindowTitle(QApplication::translate("TabMainGUI", "TabMainGUI", Q_NULLPTR));
        orderInforGroupBox->setTitle(QString());
        orderGroupBox->setTitle(QString());
        newPushButton->setText(QApplication::translate("TabMainGUI", " \346\226\260\345\273\272", Q_NULLPTR));
        openPushButton->setText(QApplication::translate("TabMainGUI", "\345\257\274\345\205\245", Q_NULLPTR));
        watchPushButton->setText(QApplication::translate("TabMainGUI", "\351\242\204\350\247\210", Q_NULLPTR));
        orderDateGroupBox->setTitle(QString());
        orderDateLineEdit->setText(QString());
        orderDateLabel->setText(QApplication::translate("TabMainGUI", "\350\256\242\345\215\225\346\227\245\346\234\237", Q_NULLPTR));
        orderNumLabel->setText(QApplication::translate("TabMainGUI", "\350\256\242\345\215\225\347\274\226\345\217\267", Q_NULLPTR));
        orderNumGroupBox->setTitle(QString());
        orderNumLineEdit->setText(QString());
        patientNameLabel->setText(QApplication::translate("TabMainGUI", "\346\202\243\350\200\205\345\247\223\345\220\215", Q_NULLPTR));
        patientNameGroupBox->setTitle(QString());
        patientNameLineEdit->setInputMask(QString());
        patientNameLineEdit->setText(QApplication::translate("TabMainGUI", "\346\202\243\350\200\205", Q_NULLPTR));
        doctorNameLabel->setText(QApplication::translate("TabMainGUI", "\345\214\273\347\224\237\345\247\223\345\220\215", Q_NULLPTR));
        doctorNameGroupBox->setTitle(QString());
        doctorNameLineEdit->setInputMask(QString());
        doctorNameLineEdit->setText(QApplication::translate("TabMainGUI", "\345\214\273\347\224\237", Q_NULLPTR));
        operatorNameLabel->setText(QApplication::translate("TabMainGUI", "\346\212\200\345\270\210\345\247\223\345\220\215", Q_NULLPTR));
        operatorNameGroupBox->setTitle(QString());
        operatorNameLineEdit->setText(QApplication::translate("TabMainGUI", "\346\212\200\345\270\210", Q_NULLPTR));
        additionLabel->setText(QApplication::translate("TabMainGUI", "\345\244\207\346\263\250", Q_NULLPTR));
        savePushButton->setText(QApplication::translate("TabMainGUI", "\344\277\235\345\255\230", Q_NULLPTR));
        orderScanGroupBox->setTitle(QString());
        spliteModelPushButton->setText(QApplication::translate("TabMainGUI", "\345\210\206\346\250\241", Q_NULLPTR));
        moulagePushButton->setText(QApplication::translate("TabMainGUI", "\345\215\260\346\250\241", Q_NULLPTR));
        unModelGroupBox->setTitle(QString());
        upperJawPushButton->setText(QString());
        lowerJawPushButton->setText(QString());
        unModelPushButton->setText(QApplication::translate("TabMainGUI", "\346\234\252\345\210\206\346\250\241", Q_NULLPTR));
        scanPushButton->setText(QApplication::translate("TabMainGUI", "\346\211\253\346\217\217", Q_NULLPTR));
        moulageGroupBox->setTitle(QString());
        moulagePushButton_2->setText(QString());
        moulagePushButton_1->setText(QString());
        moulagePushButton_3->setText(QString());
        spliteModelGroupBox->setTitle(QString());
        clearAllButton->setText(QApplication::translate("TabMainGUI", "\346\270\205\351\231\244\351\200\211\346\213\251", Q_NULLPTR));
        mainGroupBox->setTitle(QString());
        logoLabel->setText(QString());
        btnClose->setText(QString());
        settingGroupBox->setTitle(QString());
        textureLabel->setText(QApplication::translate("TabMainGUI", "\347\272\271\347\220\206", Q_NULLPTR));
        calibrationPointLabel->setText(QApplication::translate("TabMainGUI", "\346\240\207\345\277\227\347\202\271", Q_NULLPTR));
        jawFrameLabel->setText(QApplication::translate("TabMainGUI", "\351\242\214\346\236\266", Q_NULLPTR));
        saveSpliteModelLabel->setText(QApplication::translate("TabMainGUI", "\344\277\235\345\255\230\345\215\225\345\270\247\346\250\241\345\236\213", Q_NULLPTR));
        scanpathLabel->setText(QApplication::translate("TabMainGUI", "\346\211\253\346\217\217\350\267\257\345\276\204\357\274\232", Q_NULLPTR));
        scanPathGroupBox->setTitle(QString());
        scanPathLineEdit->setInputMask(QString());
        scanPathLineEdit->setText(QApplication::translate("TabMainGUI", "./ScanData", Q_NULLPTR));
        choosePathPushButton->setText(QApplication::translate("TabMainGUI", "\351\200\211\346\213\251\350\267\257\345\276\204", Q_NULLPTR));
        saveSpliteModelCheckBox->setText(QString());
        jawFrameCheckBox->setText(QString());
        calibrationPointCheckBox->setText(QString());
        textureCheckBox->setText(QString());
        calibrationGroupBox->setTitle(QString());
        calibrationPushButton->setText(QApplication::translate("TabMainGUI", "\345\274\200\345\247\213\346\240\207\345\256\232", Q_NULLPTR));
        leftCameraLable->setText(QString());
        rightCameraLable->setText(QString());
        aboutGroupBox->setTitle(QString());
        aboutImageLabel->setText(QString());
        aboutTextLabel->setText(QApplication::translate("TabMainGUI", "\347\211\231\351\275\277\346\211\253\346\217\217\344\273\252\350\275\257\344\273\266\346\230\257\344\270\200\346\254\276\345\257\271\347\211\231\351\275\277\346\250\241\345\236\213\350\277\233\350\241\214\346\211\253\346\217\217\357\274\214\345\217\226\345\276\227\347\233\270\345\272\224\344\270\211\347\273\264\346\250\241\345\236\213\347\232\204\350\275\257\344\273\266\357\274\214\350\257\245\350\275\257\344\273\266\351\205\215\345\245\227\347\211\231\351\275\277\346\211\253\346\217\217\344\273\252\345\231\250\344\275\277\347\224\250\343\200\202", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class TabMainGUI: public Ui_TabMainGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_TABMAINGUI_H
