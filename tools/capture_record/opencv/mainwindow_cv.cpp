#include "tools/capture_record/opencv/mainwindow_cv.h"

#include <QAction>
#include <QDebug>
#include <QGridLayout>
#include <QMediaMetaData>
#include <QMediaService>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPalette>
#include <QtMath>  // qCeil
#include <QtWidgets>

#include "tools/capture_record/pro/ui/ui_mainwindow.h"
#include "tools/capture_record/imagesettings.h"
#include "tools/capture_record/videosettings.h"

Q_DECLARE_METATYPE(QCameraInfo)

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  // 让 Qt Designer 里设计的界面生效
  ui->setupUi(this);

  // update save path
  this->filePath = "/media/jojo/WorkStation/test/jojo";
  // this->filePath = "/home/neousys/workspace/result";

  this->recordPlan = "08:00 - 20:00 (间隔: 01:00)";

  customSettings.filePath = filePath;

  // 显示初始路径到状态栏
  updateStatusBar();

  // 直接使用 designer 的一个 widget 占位
  viewLayout = ui->gridLayout_5;
  // 设置布局中各个小窗口之间的间距
  viewLayout->setSpacing(4);

  // 重新构建菜单（比如摄像头选择菜单）
  rebuildDeviceMenu();

  // clang-format off
  // 连接菜单动作
  connect(ui->actionSavePath, &QAction::triggered, this, &MainWindow::onActionSetSavePath);
  connect(ui->actionSavePlan, &QAction::triggered, this, &MainWindow::onActionSetSavePlan);

  // 全局 启动 和 结束 按钮的绑定
  connect(ui->actionStartCamera, &QAction::triggered, this, &MainWindow::startAllCamera);
  connect(ui->actionStopCamera, &QAction::triggered, this, &MainWindow::stopAllCamera);

  // 原来的 单个摄像头界面 被替换成 多个摄像头界面；并且在勾选时自动触发启动摄像头
  connect(ui->captureWidget, SIGNAL(currentChanged(int)), this, SLOT(updateCaptureMode(int)));
  // connect(ui->exposureCompensation, SIGNAL(valueChanged(int)), SLOT(setExposureCompensation(int)));
  connect(ui->exposureCompensation, &QSlider::valueChanged, this, &MainWindow::setExposureCompensation);

  dailyRecordTimer = new QTimer(this);
  connect(dailyRecordTimer, &QTimer::timeout, this, &MainWindow::checkDailyRecordStatus);
  // clang-format on

  // 正常显示 主界面
  ui->stackedWidget->setCurrentIndex(0);

  // init_device_service();
}

MainWindow::~MainWindow() {
  qDeleteAll(activeCameras);
  delete dailyRecordTimer;
  delete ui;
}

// ########## 响应 ##########
void MainWindow::rebuildDeviceMenu() {
  // qDebug() << "rebuildDeviceMenu() enter";

  QMenu* devicesMenu = ui->menuDevices;
  devicesMenu->clear();
  deviceActions.clear();
  discoveredCameras.clear();
  // 非标准摄像头，很难被发现，但其实可用，因此不再使用该函数
  // QCameraInfo::availableCameras()

  // 固定显示 video100 - video103（无论是否检测到）
  for (int num : fixedDevices) {
    QString key   = QString("video%1").arg(num);
    QString title = QString("/dev/video%1").arg(num);
    QAction* a    = new QAction(title, devicesMenu);
    a->setCheckable(true);

    // qDebug() << "fixed device:" << key << title;

    // way 1 这里是 "video100"
    a->setData(key);
    deviceActions.insert(key, a);

    // way 2 这里是 "/dev/video100"
    // a->setData(title);
    // deviceActions.insert(title, a);

    // bool available = checkDevicePermission(title);
    bool available = isRealVideoDevice(title);
    a->setEnabled(available);

    // 勾选 启动摄像头
    // 绑定切换：根据 action 的 data 来决定启动/停止
    connect(
        a, &QAction::toggled, this,
        [this, a](bool checked) {
          QString dev = a->data().toString();
          if (checked) {
            startCamera(dev);
          } else {
            stopCamera(dev);
          }
        },
        Qt::UniqueConnection);

    devicesMenu->addAction(a);
  }

  devicesMenu->addSeparator();

  QAction* refresh = devicesMenu->addAction("Refresh devices");
  connect(refresh, &QAction::triggered, this, &MainWindow::rebuildDeviceMenu,
          Qt::UniqueConnection);

  // startDefualtCamera();
}

void MainWindow::startDefualtCamera() {
  // 自动勾选系统默认摄像头（如果它属于我们菜单项并且可用）
  QString defaultKey =
      normalizeDevName(QCameraInfo::defaultCamera().deviceName());
  /* way 1 导致 UI 控件卡顿 偶尔响应
  if (deviceActions.contains(defaultKey) &&
      deviceActions[defaultKey]->isEnabled()) {
    // 会触发 startCamera
    deviceActions[defaultKey]->setChecked(true);
  }
  */

  // /*
  QTimer::singleShot(0, this, [this, defaultKey]() {
    // qDebug() << "singleShot: setting checked for" << defaultKey;
    if (deviceActions.contains(defaultKey) &&
        deviceActions[defaultKey]->isEnabled()) {
      deviceActions[defaultKey]->setChecked(true);
    }
  });
  // */
}

void MainWindow::startAllCamera() {
  qDebug() << "startAllCamera";

  for (CameraDevice* cam : activeCameras.values()) {
    cam->open();
  }
}

void MainWindow::stopAllCamera() {
  qDebug() << "stopAllCamera";

  for (CameraDevice* cam : activeCameras.values()) {
    cam->close();
  }
}

void MainWindow::updateCaptureMode(int index) {
  // QCamera::CaptureStillImage : QCamera::CaptureVideo;
  int mode = index == 0 ? 1 : 2;

  for (CameraDevice* cam : activeCameras.values()) {
    cam->setCaptureMode(mode);
  }

  // qDebug() << "updateCaptureMode: " << index;
}

void MainWindow::setExposureCompensation(int value) {
  for (CameraDevice* cam : activeCameras.values()) {
    cam->setExposureCompensation(value);
  }
}
// ########## 响应 ##########

// 启动一个摄像头
void MainWindow::startCamera(const QString& devKey, int mode) {
  if (activeCameras.contains(devKey)) return;
  // if (!discoveredCameras.contains(devKey)) return;

  // QCameraInfo info      = discoveredCameras[devKey];
  QString device_name   = "/dev/" + devKey;
  CameraDevice* cam     = new CameraDevice(device_name, this);
  activeCameras[devKey] = cam;

  cam->updateCustomSettings(this->customSettings);
  // qDebug() << "[MainWindow]" << "savePath:" << filePath;

  // clang-format off
  // camera 信号连接到 MainWindow
  connect(cam, &CameraDevice::cameraState, this, &MainWindow::updateCameraState);

  // capture 信号连接到 MainWindow
  connect(cam, &CameraDevice::readyCapture, this, &MainWindow::readyForCapture);
  connect(cam, &CameraDevice::previewAvailable, this, &MainWindow::processCapturedImage);
  connect(cam, &CameraDevice::saveImageSuc, this, &MainWindow::saveCapturedImage);

  // recorder 信号连接到 MainWindow
  connect(cam, &CameraDevice::recorderState, this, &MainWindow::updateRecorderState);
  connect(cam, &CameraDevice::recordTime, this, &MainWindow::updateRecordTime);
  // clang-format on

  if (mode == 1) {
    rearrangeViewLabels();
  }

  // 先启动相机，再对相机进行设置；20250907 默认进入了拍照模式
  cam->open();

  cam->initExposureSlider(ui->exposureCompensation);

  // 初始化 相机状态 页面； 但虽然先绑定信号，再启动相机，所以会直接触发一部分；
  // 但有些信号，是没有自动发布信号的，所以需要手动初始化
  updateCameraState(cam->deviceKey(), cam->cameraStatus());
  updateRecorderState(cam->deviceKey(), cam->recordStatus());

  // 调用 update_ui_tab(1)，返回 true/false → 用来决定第 0 个 tab 是否可用。
  // 调用 update_ui_tab(2)，返回 true/false → 用来决定第 1 个 tab 是否可用。
  // 像推迟 setChecked(true) 一样，把 tab 更新也延后到相机真正启动后
  QTimer::singleShot(0, this, [this]() {
    ui->captureWidget->setTabEnabled(0, update_ui_tab(1));
    ui->captureWidget->setTabEnabled(1, update_ui_tab(2));
  });
}

// 停止一个摄像头
void MainWindow::stopCamera(const QString& devKey, int mode) {
  if (!activeCameras.contains(devKey)) return;

  CameraDevice* cam = activeCameras.take(devKey);
  cam->close();
  delete cam;

  if (mode == 1) {
    rearrangeViewLabels();
  }
}

// 布局排列 viewfinder
void MainWindow::rearrangeViewLabels() {
  // 清空布局
  QLayoutItem* item = nullptr;
  while ((item = viewLayout->takeAt(0)) != nullptr) {
    if (item->widget()) {
      item->widget()->setParent(nullptr);
    }
    delete item;
  }

  QList<CameraDevice*> cams = activeCameras.values();
  int count                 = cams.size();

  if (count == 1) {
    viewLayout->addWidget(cams[0]->viewQLabel(), 0, 0);
  } else if (count == 2) {
    viewLayout->addWidget(cams[0]->viewQLabel(), 0, 0);
    viewLayout->addWidget(cams[1]->viewQLabel(), 0, 1);
  } else if (count == 3) {
    viewLayout->addWidget(cams[0]->viewQLabel(), 0, 0);
    viewLayout->addWidget(cams[1]->viewQLabel(), 0, 1);
    viewLayout->addWidget(cams[2]->viewQLabel(), 1, 0);  // 放到第二行左边
    viewLayout->setColumnStretch(0, 1);
    viewLayout->setColumnStretch(1, 1);
  } else if (count >= 4) {
    viewLayout->addWidget(cams[0]->viewQLabel(), 0, 0);
    viewLayout->addWidget(cams[1]->viewQLabel(), 0, 1);
    viewLayout->addWidget(cams[2]->viewQLabel(), 1, 0);
    viewLayout->addWidget(cams[3]->viewQLabel(), 1, 1);
    viewLayout->setColumnStretch(0, 1);
    viewLayout->setColumnStretch(1, 1);
    viewLayout->setRowStretch(0, 1);
    viewLayout->setRowStretch(1, 1);
  }
}

// ########## handler camera signals ##########
void MainWindow::updateCameraState(const QString& devKey, bool state) {
  // qDebug() << "updateCameraState: " << devKey << state;
  cameraStates[devKey] = state;
  refreshCameraControls();
}
// ########## handler camera signals ##########

bool MainWindow::update_ui_tab(int mode) {
  // qDebug() << "update_ui_tab() start ==> " << mode;

  return true;
}

void MainWindow::refreshCameraControls() {
  bool anyActive           = false;
  bool anyLoadedOrUnloaded = false;

  for (auto state : cameraStates.values()) {
    if (state) {
      anyActive = true;
    }
  }

  // 启动按钮：当**没有相机在运行**时可以启动
  ui->actionStartCamera->setEnabled(!anyActive);

  // 停止按钮：当**至少一个相机在运行**时可用
  ui->actionStopCamera->setEnabled(anyActive);

  // 拍照/录像的页面：至少一个相机在运行时可用
  ui->captureWidget->setEnabled(anyActive);

  // 设置按钮：至少一个相机在运行时可用
  ui->actionSettings->setEnabled(anyActive);
}

void MainWindow::takeAllCapture() {
  isCapturingImage = true;
  for (CameraDevice* cam : activeCameras.values()) {
    if (cam->isReadyForCapture()) {
      cam->takeImage();
    }
  }
}

// ########## handler capture signals ##########
void MainWindow::readyForCapture(const QString& devKey, bool ready) {
  // 遍历所有活跃相机状态
  bool anyReady = false;
  for (auto cam : activeCameras.values()) {
    if (cam->isReadyForCapture()) {
      anyReady = true;
      break;
    }
  }
  ui->takeImageButton->setEnabled(anyReady);
}

void MainWindow::processCapturedImage(const QString& devKey,
                                      const QImage& img) {
  // 根据 devKey 选择对应的 viewfinder 或 label 显示
  QString key = normalizeDevName(devKey);

  // qDebug() << "processCapturedImage key: " << key;

  if (!activeCameras.contains(key)) return;

  CameraDevice* cam = activeCameras[key];
  if (!cam || !cam->viewQLabel()) return;

  QImage scaled = img.scaled(ui->viewfinder->size(), Qt::KeepAspectRatio,
                             Qt::SmoothTransformation);

  // 这里目前不适合 多相机 模式
  // 会导致 ui->stackedWidget->setCurrentIndex(1); 进入"预览"页面
  ui->lastImagePreviewLabel->setPixmap(QPixmap::fromImage(scaled));

  displayCapturedImage();

  QTimer::singleShot(3000, this, SLOT(displayViewfinder()));
}

void MainWindow::saveCapturedImage(const QString& devKey,
                                   const QString& fileName) {
  Q_UNUSED(devKey);
  Q_UNUSED(fileName);

  isCapturingImage = false;
  if (applicationExiting) {
    close();
  }
}
// ########## handler capture signals ##########

// 录制所有勾选的相机
void MainWindow::startAllRecording() {
  // qDebug() << "startAllRecording()";

  isAutoRecording = false;
  dailyRecordTimer->stop();

  // 类似 _isRunning
  // isCapturingImage = true;
  // 不能设置，record
  // 是一个持续性动作，一旦开始，除非停止，否则不会结束，回导致无法退出程序
  for (CameraDevice* cam : activeCameras.values()) {
    // qDebug() << cam->deviceKey();
    // cam->updateCustomSettings(customSettings);
    cam->setCaptureMode(2);
    cam->startRecording();
  }
}

void MainWindow::pauseAllRecording() {
  dailyRecordTimer->stop();

  for (CameraDevice* cam : activeCameras.values()) {
    cam->pauseRecording();
  }
}

// 停止录制
void MainWindow::stopAllRecording(int mode) {
  if (mode == 1) {
    isAutoRecording = false;
    dailyRecordTimer->stop();
  }

  for (CameraDevice* cam : activeCameras.values()) {
    cam->stopRecording();
  }
}

void MainWindow::startAllAutoRecording() {
  // qDebug() << "startAllAutoRecording()";

  isAutoRecording = true;
  dailyRecordTimer->start(60000);  // 每分钟检查一次
  checkDailyRecordStatus();
  refreshRecorderControls();

  // dailyRecordTimer->start(1000);  // 每秒检查一次

  // /*  // 不再能手动控制
  // for (CameraDevice *cam : activeCameras.values()) {
  // qDebug() << cam->deviceKey();
  // cam->updateCustomSettings(customSettings);
  // cam->setCaptureMode(2);
  // cam->startAutoRecording();
  // }
  // */
}

void MainWindow::stopAllAutoRecording() {
  qDebug() << "stopAllAutoRecording()";

  stopAllRecording(1);

  startAllCamera();
  refreshRecorderControls();
}

void MainWindow::setAllMuted(bool muted) {}

void MainWindow::updateRecordTime(const QString& devKey,
                                  const qint64& timeSec) {
  qint64 hours   = timeSec / 3600;
  qint64 minutes = (timeSec % 3600) / 60;
  qint64 seconds = timeSec % 60;

  QString timeStr = QString("%1:%2:%3")
                        .arg(hours, 2, 10, QChar('0'))
                        .arg(minutes, 2, 10, QChar('0'))
                        .arg(seconds, 2, 10, QChar('0'));

  QStringList messages;
  messages << QString("%1: %2").arg(devKey).arg(timeStr);
  ui->statusbar->showMessage(messages.join(" | "));
}

// ########## handler recorder signals ##########
void MainWindow::updateRecorderState(const QString& devKey,
                                     const RecordState& state) {
  recorderStates[devKey] = state;  // 保存状态
  refreshRecorderControls();  // 刷新按钮状态
  // qDebug() << "updateRecorderState: " << devKey << state;
}

void MainWindow::refreshRecorderControls() {
  bool anyRecording     = false;
  bool anyAutoRecording = false;
  bool anyPaused        = false;
  bool anyStopped       = false;

  for (auto state : recorderStates.values()) {
    switch (state) {
      case RecordState::Recording:
        anyRecording = true;
        break;
      case RecordState::AutoRecording:
        anyAutoRecording = true;
        break;
      case RecordState::Paused:
        anyPaused = true;
        break;
      case RecordState::Stopped:
        anyStopped = true;
        break;
    }
  }

  // qDebug() << "refreshRecorderControls: isAutoRecording: " <<
  // isAutoRecording;

  // clang-format off
  // 统一控制按钮
  // - 录制按钮可用条件：没有手动录制正在进行，并且自动录制没有在跑，或者当前是暂停状态
  ui->recordButton->setEnabled(((!anyRecording && !anyAutoRecording) || anyPaused) && !isAutoRecording);

  // - 自动录制按钮可用条件：没有自动录制正在进行，并且手动录制没有在跑，或者当前是暂停状态
  ui->autoRecordButton->setEnabled(((!anyAutoRecording && !anyRecording) || anyPaused) && !isAutoRecording);

  // pauseButton：如果至少一个相机正在录制，则可用
  ui->pauseButton->setEnabled(anyRecording || anyAutoRecording);

  // stopButton：如果至少有一个相机正在录制或暂停，则可用
  ui->stopButton->setEnabled(((anyRecording && !anyAutoRecording) || anyPaused) && !isAutoRecording);

  ui->autoStopButton->setEnabled((anyAutoRecording && !anyRecording) || anyPaused || isAutoRecording);
  // clang-format on

  // qDebug() << "refreshRecorderControls: isAutoRecording: " <<
  // ((anyAutoRecording && !anyRecording) || anyPaused || isAutoRecording);
}
// ########## handler recorder signals ##########

void MainWindow::keyPressEvent(QKeyEvent* event) {
  // 忽略自动重复按键
  if (event->isAutoRepeat()) return;

  // 根据按键执行操作
  switch (event->key()) {
    case Qt::Key_CameraFocus:
      // 遍历所有激活的相机，执行对焦
      for (CameraDevice* cam : activeCameras.values()) {
        // cam->searchAndLock();
      }
      event->accept();
      break;

    case Qt::Key_Camera:
      // 遍历所有激活的相机，根据当前模式拍照或录像
      for (CameraDevice* cam : activeCameras.values()) {
        if (cam->isCapturing()) {
          cam->takeImage();
        } else {
          if (cam->isRecording())  // CameraDevice 内部维护状态
            cam->stopRecording();
          else
            cam->startRecording();
        }
      }
      event->accept();
      break;

    default:
      QMainWindow::keyPressEvent(event);
  }
}

void MainWindow::keyReleaseEvent(QKeyEvent* event) {
  if (event->isAutoRepeat()) return;

  switch (event->key()) {
    case Qt::Key_CameraFocus:
      for (CameraDevice* cam : activeCameras.values()) {
        // cam->unlock();
      }
      break;
    default:
      QMainWindow::keyReleaseEvent(event);
  }
}

void MainWindow::toggleLock() {
  for (CameraDevice* cam : activeCameras.values()) {
  }
}

void MainWindow::displayViewfinder() {
  // 静止图像的显示结束后，切回到“实时”页面
  // 把当前显示的“页面”切换到索引为 0 的那个 widget
  ui->stackedWidget->setCurrentIndex(0);
}

void MainWindow::displayCapturedImage() {
  // 切换到"预览"页面
  ui->stackedWidget->setCurrentIndex(1);
}

void MainWindow::configureCameraSettings() {
  // qDebug() << "configureCameraSettings";

  if (activeCameras.isEmpty()) return;

  // 选择第一个相机作为设置窗口的参考
  CameraDevice* firstCam = activeCameras.first();
  if (!firstCam) return;

  if (firstCam->isCapturing()) {
    configureImageSettings();  // 里面已经批量应用到所有相机
  } else if (firstCam->isRecording()) {
    configureVideoSettings();  // 里面已经批量应用到所有相机
  }
}

void MainWindow::configureVideoSettings() {
  // 弹出一次设置对话框，基于第一个相机初始化
  CameraDevice* firstCam = nullptr;
  if (!activeCameras.isEmpty()) firstCam = activeCameras.first();

  if (!firstCam) return;

  // qDebug() << "configureVideoSettings";

  // VideoSettings settingsDialog(firstCam->mediaRecorder());
  // settingsDialog.setAudioSettings(audioSettings);
  // settingsDialog.setVideoSettings(videoSettings);
  // settingsDialog.setFormat(videoContainerFormat);

  // if (settingsDialog.exec()) {
  //   audioSettings        = settingsDialog.audioSettings();
  //   videoSettings        = settingsDialog.videoSettings();
  //   videoContainerFormat = settingsDialog.format();

  //   for (CameraDevice *cam : activeCameras.values()) {
  //     cam->mediaRecorder()->setEncodingSettings(audioSettings, videoSettings,
  //                                               videoContainerFormat);
  //   }
  // }
}

void MainWindow::configureImageSettings() {
  CameraDevice* firstCam = nullptr;
  if (!activeCameras.isEmpty()) firstCam = activeCameras.first();

  if (!firstCam) return;

  // qDebug() << "configureImageSettings";

  // ImageSettings settingsDialog(firstCam->imageCapturer());
  // settingsDialog.setImageSettings(imageSettings);

  // if (settingsDialog.exec()) {
  //   imageSettings = settingsDialog.imageSettings();

  //   for (CameraDevice *cam : activeCameras.values()) {
  //     cam->imageCapturer()->setEncodingSettings(imageSettings);
  //   }
  // }
}

void MainWindow::closeEvent(QCloseEvent* event) {
  if (isCapturingImage) {
    // 禁用整个窗口（或者这个窗口部件），防止用户在拍照过程中操作其他按钮。
    setEnabled(false);
    applicationExiting = true;
    event->ignore();
  } else {
    event->accept();
  }
}

QString MainWindow::normalizeDevName(const QString& dev) {
  if (dev.startsWith("/dev/"))
    return dev.mid(5);  // "/dev/video101" -> "video101"
  return dev;
}

// 菜单栏触发槽
void MainWindow::onActionSetSavePath() {
  QString dir =
      QFileDialog::getExistingDirectory(this, tr("选择保存路径"), filePath);
  if (!dir.isEmpty()) {
    filePath                      = dir;
    this->customSettings.filePath = filePath;
    updateAllCamera();
    updateStatusBar();  // 更新状态栏显示
    QMessageBox::information(this, tr("路径已设置"),
                             QString("当前保存路径:\n%1").arg(dir));
  }
}

void MainWindow::onActionSetSavePlan() {
  QDialog dlg(this);
  dlg.setWindowTitle("设置每日录制计划");

  QTime lastStartTime = QTime(customSettings.recordStartSeconds / 3600,
                              (customSettings.recordStartSeconds % 3600) / 60);

  QTime lastIntervalTime = QTime(customSettings.segmentSeconds / 3600,
                                 (customSettings.segmentSeconds % 3600) / 60);

  QTime lastEndTime = QTime(customSettings.recordEndSeconds / 3600,
                            (customSettings.recordEndSeconds % 3600) / 60);

  // 在 MainWindow 构造函数或初始化函数中
  QLabel* startLabel = new QLabel("开始录制时间:", this);
  // QTimeEdit *startTimeEdit = new QTimeEdit(QTime::fromString("08:00",
  // "HH:mm"), this);
  QTimeEdit* startTimeEdit = new QTimeEdit(lastStartTime, this);
  startTimeEdit->setDisplayFormat("HH:mm");  // 只显示小时和分钟

  QLabel* intervalLabel = new QLabel("录制间隔:", this);
  // QTimeEdit *intervalTimeEdit = new QTimeEdit(QTime::fromString("00:30",
  // "HH:mm"), this);
  QTimeEdit* intervalTimeEdit = new QTimeEdit(lastIntervalTime, this);
  intervalTimeEdit->setDisplayFormat("HH:mm");

  QLabel* endLabel = new QLabel("结束录制时间:", this);
  // QTimeEdit *endTimeEdit = new QTimeEdit(QTime::fromString("20:00", "HH:mm"),
  // this);
  QTimeEdit* endTimeEdit = new QTimeEdit(lastEndTime, this);
  endTimeEdit->setDisplayFormat("HH:mm");

  QPushButton* okBtn     = new QPushButton("确定");
  QPushButton* cancelBtn = new QPushButton("取消");

  connect(okBtn, &QPushButton::clicked, &dlg, &QDialog::accept);
  connect(cancelBtn, &QPushButton::clicked, &dlg, &QDialog::reject);

  // 布局
  QGridLayout* layout = new QGridLayout;
  layout->addWidget(startLabel, 0, 0);
  layout->addWidget(startTimeEdit, 0, 1);
  layout->addWidget(intervalLabel, 1, 0);
  layout->addWidget(intervalTimeEdit, 1, 1);
  layout->addWidget(endLabel, 2, 0);
  layout->addWidget(endTimeEdit, 2, 1);
  layout->addWidget(okBtn, 3, 0);
  layout->addWidget(cancelBtn, 3, 1);
  dlg.setLayout(layout);

  if (dlg.exec() == QDialog::Accepted) {
    QTime startTime    = startTimeEdit->time();
    QTime intervalTime = intervalTimeEdit->time();
    QTime endTime      = endTimeEdit->time();

    int startSec    = startTime.hour() * 3600 + startTime.minute() * 60;
    int intervalSec = intervalTime.hour() * 3600 + intervalTime.minute() * 60;
    int endSec      = endTime.hour() * 3600 + endTime.minute() * 60;
    int crossEndSec = endSec;

    bool isCrossDay = (endSec < startSec);

    // 更新 recordPlan 字符串
    if (isCrossDay) {
      recordPlan = QString("%1 - 次日%2 (间隔: %3)")
                       .arg(startTime.toString("HH:mm"))
                       .arg(endTime.toString("HH:mm"))
                       .arg(intervalTime.toString("HH:mm"));
      crossEndSec = endSec + 24 * 3600;
    } else {
      recordPlan = QString("%1 - %2 (间隔: %3)")
                       .arg(startTime.toString("HH:mm"))
                       .arg(endTime.toString("HH:mm"))
                       .arg(intervalTime.toString("HH:mm"));
    }

    // qDebug() << "每日录制时间设置: " << recordPlan;

    // 保存到成员变量或 QSettings
    customSettings.recordStartSeconds = startSec;
    customSettings.segmentSeconds     = intervalSec;
    customSettings.recordEndSeconds   = endSec;
    customSettings.isCrossDay         = isCrossDay;
    customSettings.crossEndSec        = crossEndSec;

    updateAllCamera();
    updateStatusBar();
  }
}

void MainWindow::updateStatusBar() {
  // 临时显示保存路径
  // statusBar()->showMessage(QString("当前保存路径: %1").arg(filePath));

  // 永久显示
  ui->pathLabel->setText(QString("当前保存路径: %1").arg(filePath));
  ui->planLabel->setText(QString("当前录制计划: HH:mm %1").arg(recordPlan));
}

void MainWindow::updateAllCamera() {
  for (CameraDevice* cam : activeCameras.values()) {
    cam->updateCustomSettings(this->customSettings);
  }
}

void MainWindow::checkDailyRecordStatus() {
  int nowSec =
      QTime::currentTime().hour() * 3600 + QTime::currentTime().minute() * 60;

  bool shouldAutoRecord = false;
  if (!customSettings.isCrossDay) {
    shouldAutoRecord = (nowSec >= customSettings.recordStartSeconds &&
                        nowSec <= customSettings.recordEndSeconds);
  } else {
    // 跨夜情况  把结束时间视为第二天
    shouldAutoRecord = (nowSec >= customSettings.recordStartSeconds ||
                        nowSec <= customSettings.crossEndSec);
  }

  // qDebug() << "检查每日录制状态: " << shouldAutoRecord;

  if (shouldAutoRecord) {
    // qDebug() << "开始自动录制";

    this->startAllCamera();

    // 遍历所有摄像头实例
    for (auto it = activeCameras.begin(); it != activeCameras.end(); ++it) {
      CameraDevice* cam = it.value();
      if (!cam) continue;

      if (!cam->isRecording()) {
        cam->setCaptureMode(2);
        cam->startAutoRecording();
      }
    }
  } else {
    qDebug() << "停止录制，并关闭所有摄像头，等待下一次录制";

    // 终止所有录制，但不关闭定时器，不退出自动录制状态
    this->stopAllRecording(2);
    this->stopAllCamera();
    // 更新状态栏，使得 停止自动录制 按钮可用
    refreshRecorderControls();
    ui->captureWidget->setEnabled(true);
  }
}
