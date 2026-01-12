/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStackedWidget>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qcameraviewfinder.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionExit;
    QAction *actionStartCamera;
    QAction *actionStopCamera;
    QAction *actionSettings;
    QAction *actionSavePath;
    QAction *actionSavePlan;
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QStackedWidget *stackedWidget;
    QWidget *viewfinderPage;
    QGridLayout *gridLayout_5;
    QCameraViewfinder *viewfinder;
    QWidget *previewPage;
    QGridLayout *gridLayout_4;
    QLabel *lastImagePreviewLabel;
    QPushButton *lockButton;
    QTabWidget *captureWidget;
    QWidget *tab_2;
    QGridLayout *gridLayout;
    QPushButton *takeImageButton;
    QLabel *label;
    QSlider *exposureCompensation;
    QSpacerItem *verticalSpacer_2;
    QWidget *tab;
    QVBoxLayout *verticalLayout;
    QPushButton *recordButton;
    QPushButton *pauseButton;
    QPushButton *stopButton;
    QPushButton *autoRecordButton;
    QPushButton *autoStopButton;
    QSpacerItem *verticalSpacer;
    QPushButton *muteButton;
    QLabel *pathLabel;
    QLabel *planLabel;
    QMenuBar *menubar;
    QMenu *menuFile;
    QMenu *menuDevices;
    QMenu *menuFile_2;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(803, 525);
        actionExit = new QAction(MainWindow);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionStartCamera = new QAction(MainWindow);
        actionStartCamera->setObjectName(QString::fromUtf8("actionStartCamera"));
        actionStopCamera = new QAction(MainWindow);
        actionStopCamera->setObjectName(QString::fromUtf8("actionStopCamera"));
        actionSettings = new QAction(MainWindow);
        actionSettings->setObjectName(QString::fromUtf8("actionSettings"));
        actionSavePath = new QAction(MainWindow);
        actionSavePath->setObjectName(QString::fromUtf8("actionSavePath"));
        actionSavePlan = new QAction(MainWindow);
        actionSavePlan->setObjectName(QString::fromUtf8("actionSavePlan"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        stackedWidget = new QStackedWidget(centralwidget);
        stackedWidget->setObjectName(QString::fromUtf8("stackedWidget"));
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(stackedWidget->sizePolicy().hasHeightForWidth());
        stackedWidget->setSizePolicy(sizePolicy);
        QPalette palette;
        QBrush brush(QColor(255, 255, 255, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Base, brush);
        QBrush brush1(QColor(145, 145, 145, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        stackedWidget->setPalette(palette);
        viewfinderPage = new QWidget();
        viewfinderPage->setObjectName(QString::fromUtf8("viewfinderPage"));
        gridLayout_5 = new QGridLayout(viewfinderPage);
        gridLayout_5->setObjectName(QString::fromUtf8("gridLayout_5"));
        viewfinder = new QCameraViewfinder(viewfinderPage);
        viewfinder->setObjectName(QString::fromUtf8("viewfinder"));

        gridLayout_5->addWidget(viewfinder, 0, 0, 1, 1);

        stackedWidget->addWidget(viewfinderPage);
        previewPage = new QWidget();
        previewPage->setObjectName(QString::fromUtf8("previewPage"));
        gridLayout_4 = new QGridLayout(previewPage);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        lastImagePreviewLabel = new QLabel(previewPage);
        lastImagePreviewLabel->setObjectName(QString::fromUtf8("lastImagePreviewLabel"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::MinimumExpanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(lastImagePreviewLabel->sizePolicy().hasHeightForWidth());
        lastImagePreviewLabel->setSizePolicy(sizePolicy1);
        lastImagePreviewLabel->setFrameShape(QFrame::Box);

        gridLayout_4->addWidget(lastImagePreviewLabel, 0, 0, 1, 1);

        stackedWidget->addWidget(previewPage);

        gridLayout_2->addWidget(stackedWidget, 0, 0, 2, 1);

        lockButton = new QPushButton(centralwidget);
        lockButton->setObjectName(QString::fromUtf8("lockButton"));

        gridLayout_2->addWidget(lockButton, 0, 1, 1, 1);

        captureWidget = new QTabWidget(centralwidget);
        captureWidget->setObjectName(QString::fromUtf8("captureWidget"));
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        gridLayout = new QGridLayout(tab_2);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        takeImageButton = new QPushButton(tab_2);
        takeImageButton->setObjectName(QString::fromUtf8("takeImageButton"));
        takeImageButton->setEnabled(false);

        gridLayout->addWidget(takeImageButton, 0, 0, 1, 1);

        label = new QLabel(tab_2);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 2, 0, 1, 1);

        exposureCompensation = new QSlider(tab_2);
        exposureCompensation->setObjectName(QString::fromUtf8("exposureCompensation"));
        exposureCompensation->setMinimum(-4);
        exposureCompensation->setMaximum(4);
        exposureCompensation->setPageStep(2);
        exposureCompensation->setOrientation(Qt::Horizontal);
        exposureCompensation->setTickPosition(QSlider::TicksAbove);

        gridLayout->addWidget(exposureCompensation, 3, 0, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 161, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer_2, 1, 0, 1, 1);

        captureWidget->addTab(tab_2, QString());
        tab = new QWidget();
        tab->setObjectName(QString::fromUtf8("tab"));
        verticalLayout = new QVBoxLayout(tab);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        recordButton = new QPushButton(tab);
        recordButton->setObjectName(QString::fromUtf8("recordButton"));
        recordButton->setMinimumSize(QSize(141, 0));

        verticalLayout->addWidget(recordButton);

        pauseButton = new QPushButton(tab);
        pauseButton->setObjectName(QString::fromUtf8("pauseButton"));

        verticalLayout->addWidget(pauseButton);

        stopButton = new QPushButton(tab);
        stopButton->setObjectName(QString::fromUtf8("stopButton"));

        verticalLayout->addWidget(stopButton);

        autoRecordButton = new QPushButton(tab);
        autoRecordButton->setObjectName(QString::fromUtf8("autoRecordButton"));
        autoRecordButton->setMinimumSize(QSize(141, 0));

        verticalLayout->addWidget(autoRecordButton);

        autoStopButton = new QPushButton(tab);
        autoStopButton->setObjectName(QString::fromUtf8("autoStopButton"));
        autoStopButton->setMinimumSize(QSize(141, 0));

        verticalLayout->addWidget(autoStopButton);

        verticalSpacer = new QSpacerItem(20, 76, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer);

        muteButton = new QPushButton(tab);
        muteButton->setObjectName(QString::fromUtf8("muteButton"));
        muteButton->setCheckable(true);

        verticalLayout->addWidget(muteButton);

        captureWidget->addTab(tab, QString());

        gridLayout_2->addWidget(captureWidget, 1, 1, 1, 1);

        pathLabel = new QLabel(centralwidget);
        pathLabel->setObjectName(QString::fromUtf8("pathLabel"));

        gridLayout_2->addWidget(pathLabel, 2, 0, 1, 1);

        planLabel = new QLabel(centralwidget);
        planLabel->setObjectName(QString::fromUtf8("planLabel"));

        gridLayout_2->addWidget(planLabel, 3, 0, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 803, 29));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuDevices = new QMenu(menubar);
        menuDevices->setObjectName(QString::fromUtf8("menuDevices"));
        menuFile_2 = new QMenu(menubar);
        menuFile_2->setObjectName(QString::fromUtf8("menuFile_2"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuFile_2->menuAction());
        menubar->addAction(menuDevices->menuAction());
        menuFile->addAction(actionStartCamera);
        menuFile->addAction(actionStopCamera);
        menuFile->addSeparator();
        menuFile->addAction(actionSettings);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuFile_2->addAction(actionSavePath);
        menuFile_2->addAction(actionSavePlan);

        retranslateUi(MainWindow);
        QObject::connect(recordButton, SIGNAL(clicked()), MainWindow, SLOT(startAllRecording()));
        QObject::connect(autoRecordButton, SIGNAL(clicked()), MainWindow, SLOT(startAllAutoRecording()));
        QObject::connect(autoStopButton, SIGNAL(clicked()), MainWindow, SLOT(stopAllAutoRecording()));
        QObject::connect(stopButton, SIGNAL(clicked()), MainWindow, SLOT(stopAllRecording()));
        QObject::connect(pauseButton, SIGNAL(clicked()), MainWindow, SLOT(pauseAllRecording()));
        QObject::connect(actionExit, SIGNAL(triggered()), MainWindow, SLOT(close()));
        QObject::connect(takeImageButton, SIGNAL(clicked()), MainWindow, SLOT(takeAllCapture()));
        QObject::connect(lockButton, SIGNAL(clicked()), MainWindow, SLOT(toggleLock()));
        QObject::connect(muteButton, SIGNAL(toggled(bool)), MainWindow, SLOT(setAllMuted(bool)));
        QObject::connect(exposureCompensation, SIGNAL(valueChanged(int)), MainWindow, SLOT(setExposureCompensation(int)));
        QObject::connect(actionSettings, SIGNAL(triggered()), MainWindow, SLOT(configureCameraSettings()));
        QObject::connect(actionStartCamera, SIGNAL(triggered()), MainWindow, SLOT(startAllCamera()));
        QObject::connect(actionStopCamera, SIGNAL(triggered()), MainWindow, SLOT(stopAllCamera()));

        stackedWidget->setCurrentIndex(1);
        captureWidget->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        actionExit->setText(QApplication::translate("MainWindow", "Exit", nullptr));
        actionStartCamera->setText(QApplication::translate("MainWindow", "Start Camera", nullptr));
        actionStopCamera->setText(QApplication::translate("MainWindow", "Stop Camera", nullptr));
        actionSettings->setText(QApplication::translate("MainWindow", "Settings", nullptr));
        actionSavePath->setText(QApplication::translate("MainWindow", "Save File Path", nullptr));
        actionSavePlan->setText(QApplication::translate("MainWindow", "Save Plan", nullptr));
        lastImagePreviewLabel->setText(QString());
        lockButton->setText(QApplication::translate("MainWindow", "Focus", nullptr));
        takeImageButton->setText(QApplication::translate("MainWindow", "Capture Photo", nullptr));
        label->setText(QApplication::translate("MainWindow", "Exposure Compensation:", nullptr));
        captureWidget->setTabText(captureWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Image", nullptr));
        recordButton->setText(QApplication::translate("MainWindow", "Record", nullptr));
        pauseButton->setText(QApplication::translate("MainWindow", "Pause", nullptr));
        stopButton->setText(QApplication::translate("MainWindow", "Stop", nullptr));
        autoRecordButton->setText(QApplication::translate("MainWindow", "Auto Record", nullptr));
        autoStopButton->setText(QApplication::translate("MainWindow", "Auto Stop", nullptr));
        muteButton->setText(QApplication::translate("MainWindow", "Mute", nullptr));
        captureWidget->setTabText(captureWidget->indexOf(tab), QApplication::translate("MainWindow", "Video", nullptr));
        pathLabel->setText(QApplication::translate("MainWindow", "SavePath", nullptr));
        planLabel->setText(QApplication::translate("MainWindow", "SavePlan", nullptr));
        menuFile->setTitle(QApplication::translate("MainWindow", "Session", nullptr));
        menuDevices->setTitle(QApplication::translate("MainWindow", "Devices", nullptr));
        menuFile_2->setTitle(QApplication::translate("MainWindow", "File", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
