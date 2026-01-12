#include "tools/capture_record/qt5/mainwindow_qt5.h"

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

  // 直接使用 designer 的一个 widget 占位
  viewLayout = ui->gridLayout_5;
  // 设置布局中各个小窗口之间的间距
  viewLayout->setSpacing(4);

  // 重新构建菜单（比如摄像头选择菜单）
  rebuildDeviceMenu();

  // clang-format off
  // 全局 启动 和 结束 按钮的绑定
  connect(ui->actionStartCamera, &QAction::triggered, this, &MainWindow::startAllCamera);
  connect(ui->actionStopCamera, &QAction::triggered, this, &MainWindow::stopAllCamera);

  // 原来的 单个摄像头界面 被替换成 多个摄像头界面；并且在勾选时自动触发启动摄像头
  connect(ui->captureWidget, SIGNAL(currentChanged(int)), this, SLOT(updateCaptureMode(int)));
  // connect(ui->exposureCompensation, SIGNAL(valueChanged(int)), SLOT(setExposureCompensation(int)));
  connect(ui->exposureCompensation, &QSlider::valueChanged, this, &MainWindow::setExposureCompensation);
  // clang-format on

  // 正常显示 主界面
  ui->stackedWidget->setCurrentIndex(0);

  // init_device_service();
}

MainWindow::~MainWindow() {
  qDeleteAll(activeCameras);
  delete ui;
}

// ########## 响应 ##########
void MainWindow::rebuildDeviceMenu() {
  qDebug() << "rebuildDeviceMenu() enter";

  QMenu* devicesMenu = ui->menuDevices;
  devicesMenu->clear();
  deviceActions.clear();
  discoveredCameras.clear();

  // 扫描系统可用摄像头 /dev/video 存成 /video
  for (const QCameraInfo& info : QCameraInfo::availableCameras()) {
    QString key = normalizeDevName(info.deviceName());
    discoveredCameras.insert(key, info);
  }

  // 固定显示 video100 - video103（无论是否检测到）
  for (int num : fixedDevices) {
    QString key   = QString("video%1").arg(num);
    QString title = QString("/dev/video%1").arg(num);
    QAction* a    = new QAction(title, devicesMenu);
    a->setCheckable(true);
    a->setData(key);
    deviceActions.insert(key, a);

    if (discoveredCameras.contains(key)) {
      a->setEnabled(true);
    } else {
      a->setEnabled(false);  // 灰色显示
    }

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

  // 添加系统检测到的其它设备（排除已固定显示的）
  for (auto it = discoveredCameras.constBegin();
       it != discoveredCameras.constEnd(); ++it) {
    QString key = it.key();
    if (deviceActions.contains(key)) continue;  // 已被固定项占用

    QString title = it.value().description();
    if (title.isEmpty()) title = key;

    QAction* a = new QAction(title, devicesMenu);
    a->setCheckable(true);
    a->setData(key);
    deviceActions.insert(key, a);

    /* // way 1
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
    */

    // way 2
    connect(
        a, &QAction::toggled, this,
        [this](bool checked) {
          QAction* action = qobject_cast<QAction*>(sender());
          if (!action) return;
          QString dev = action->data().toString();
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

  startDefualtCamera();
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
    qDebug() << "singleShot: setting checked for" << defaultKey;
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
    cam->start();
  }
}

void MainWindow::stopAllCamera() {
  qDebug() << "stopAllCamera";

  for (CameraDevice* cam : activeCameras.values()) {
    cam->stop();
  }
}

void MainWindow::updateCaptureMode(int index) {
  QCamera::CaptureModes mode =
      index == 0 ? QCamera::CaptureStillImage : QCamera::CaptureVideo;

  for (CameraDevice* cam : activeCameras.values()) {
    cam->setCaptureMode(mode);
  }

  qDebug() << "updateCaptureMode: " << mode;
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
  if (!discoveredCameras.contains(devKey)) return;

  QCameraInfo info      = discoveredCameras[devKey];
  CameraDevice* cam     = new CameraDevice(info, this);
  activeCameras[devKey] = cam;

  cam->setSavePath(this->filePath);
  // qDebug() << "[MainWindow]" << "savePath:" << filePath;

  // clang-format off
  // camera 信号连接到 MainWindow
  connect(cam, &CameraDevice::cameraState, this, &MainWindow::updateCameraState);
  connect(cam, &CameraDevice::cameraError, this, &MainWindow::displayCameraError);
  connect(cam, &CameraDevice::cameraLockState, this, &MainWindow::updateLockStatus);

  // capture 信号连接到 MainWindow
  connect(cam, &CameraDevice::readyCapture, this, &MainWindow::readyForCapture);
  // connect(cam, &CameraDevice::previewAvailable, this, &MainWindow::processCapturedImage);
  connect(cam, &CameraDevice::saveImageSuc, this, &MainWindow::saveCapturedImage);
  connect(cam, &CameraDevice::captureError, this, &MainWindow::displayCaptureError);

  // recorder 信号连接到 MainWindow
  connect(cam, &CameraDevice::recorderState, this, &MainWindow::updateRecorderState);
  connect(cam, &CameraDevice::recordTime, this, &MainWindow::updateRecordTime);
  connect(cam, &CameraDevice::recorderError, this, &MainWindow::displayRecorderError);
  // clang-format on

  if (mode == 1) {
    rearrangeViewfinders();
  }

  // 先启动相机，再对相机进行设置；20250907 默认进入了拍照模式
  cam->start();

  cam->initExposureSlider(ui->exposureCompensation);

  // 初始化 相机状态 页面； 但虽然先绑定信号，再启动相机，所以会直接触发一部分；
  // 但有些信号，是没有自动发布信号的，所以需要手动初始化
  updateCameraState(cam->deviceKey(), cam->cameraStatus());
  updateLockStatus(cam->deviceKey(), cam->lockStatus(), QCamera::UserRequest);
  updateRecorderState(cam->deviceKey(), cam->recorderStatus());

  // 调用 update_ui_tab(1)，返回 true/false → 用来决定第 0 个 tab 是否可用。
  // 调用 update_ui_tab(2)，返回 true/false → 用来决定第 1 个 tab 是否可用。
  // 像推迟 setChecked(true) 一样，把 tab 更新也延后到相机真正启动后
  QTimer::singleShot(0, this, [this]() {
    ui->captureWidget->setTabEnabled(1, update_ui_tab(2));
    ui->captureWidget->setTabEnabled(0, update_ui_tab(1));
  });
}

// 停止一个摄像头
void MainWindow::stopCamera(const QString& devKey, int mode) {
  if (!activeCameras.contains(devKey)) return;

  CameraDevice* cam = activeCameras.take(devKey);
  cam->stop();
  delete cam;

  if (mode == 1) {
    rearrangeViewfinders();
  }
}

// 布局排列 viewfinder
void MainWindow::rearrangeViewfinders() {
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
    viewLayout->addWidget(cams[0]->viewfinderWidget(), 0, 0);
  } else if (count == 2) {
    viewLayout->addWidget(cams[0]->viewfinderWidget(), 0, 0);
    viewLayout->addWidget(cams[1]->viewfinderWidget(), 0, 1);
    viewLayout->setColumnStretch(0, 1);
    viewLayout->setRowStretch(0, 1);
  } else if (count == 3) {
    viewLayout->addWidget(cams[0]->viewfinderWidget(), 0, 0);
    viewLayout->addWidget(cams[1]->viewfinderWidget(), 0, 1);
    viewLayout->addWidget(cams[2]->viewfinderWidget(), 1, 0);  // 放到第二行左边
    viewLayout->setColumnStretch(0, 1);
    viewLayout->setColumnStretch(1, 1);
  } else if (count >= 4) {
    viewLayout->addWidget(cams[0]->viewfinderWidget(), 0, 0);
    viewLayout->addWidget(cams[1]->viewfinderWidget(), 0, 1);
    viewLayout->addWidget(cams[2]->viewfinderWidget(), 1, 0);
    viewLayout->addWidget(cams[3]->viewfinderWidget(), 1, 1);
    viewLayout->setColumnStretch(0, 1);
    viewLayout->setColumnStretch(1, 1);
    viewLayout->setRowStretch(0, 1);
    viewLayout->setRowStretch(1, 1);
  }
}

// ########## handler camera signals ##########
void MainWindow::updateCameraState(const QString& devKey,
                                   const QCamera::State& state) {
  cameraStates[devKey] = state;
  refreshCameraControls();
  // qDebug() << "updateCameraState: " << devKey << state;
}

void MainWindow::refreshCameraControls() {
  bool anyActive           = false;
  bool anyLoadedOrUnloaded = false;

  for (auto state : cameraStates.values()) {
    if (state == QCamera::ActiveState) {
      anyActive = true;
    } else {
      anyLoadedOrUnloaded = true;
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

void MainWindow::displayCameraError(const QString& devKey,
                                    const QString& errorString) {
  QMessageBox::warning(this, tr("%1 camera error").arg(devKey), errorString);
  // ui->statusLog->append(devKey + " error: " + errorString);
}

void MainWindow::updateLockStatus(const QString& devKey,
                                  const QCamera::LockStatus& status,
                                  const QCamera::LockChangeReason& reason) {
  QColor indicationColor = Qt::black;
  QString buttonText;
  QString statusMsg;

  switch (status) {
    case QCamera::Searching:
      indicationColor = Qt::yellow;
      buttonText      = tr("Focusing...");
      statusMsg       = tr("Focusing...");
      break;
    case QCamera::Locked:
      indicationColor = Qt::darkGreen;
      buttonText      = tr("Unlock");
      statusMsg       = tr("Focused");
      break;
    case QCamera::Unlocked:
      indicationColor = (reason == QCamera::LockFailed ? Qt::red : Qt::black);
      buttonText      = tr("Focus");
      if (reason == QCamera::LockFailed) {
        statusMsg = tr("Focus Failed");
      }
      break;
  }

  // 更新对应相机的 UI
  // if (CameraDevice *cam = activeCameras.value(devKey, nullptr)) {
  ui->lockButton->setText(buttonText);
  QPalette palette = ui->lockButton->palette();
  palette.setColor(QPalette::ButtonText, indicationColor);
  ui->lockButton->setPalette(palette);
  // }

  ui->statusbar->showMessage(statusMsg, 2000);
}
// ########## handler camera signals ##########

void MainWindow::init_device_service() {
  for (CameraDevice* cam : activeCameras.values()) {
    updateCameraState(cam->deviceKey(), cam->cameraStatus());
    updateLockStatus(cam->deviceKey(), cam->lockStatus(), QCamera::UserRequest);
    updateRecorderState(cam->deviceKey(), cam->recorderStatus());

    // 获取当前选项卡的索引
    int tabIndex = ui->captureWidget->currentIndex();
    QCamera::CaptureModes mode =
        tabIndex == 0 ? QCamera::CaptureStillImage : QCamera::CaptureVideo;
    cam->setCaptureMode(mode);
  }
}

bool MainWindow::update_ui_tab(int mode) {
  // qDebug() << "update_ui_tab() start ==> " << mode;

  bool anyCanCaptureImage = false;
  bool anyCanCaptureVideo = false;

  for (CameraDevice* cam : activeCameras.values()) {
    if (mode == 1) {
      if (cam->isStillImageCaptureSupported()) {
        anyCanCaptureImage = true;
      }
    } else if (mode == 2) {
      if (cam->isVideoCaptureSupported()) {
        anyCanCaptureVideo = true;
      }
    }
  }

  // qDebug() << "anyCanCaptureImage ==> " << anyCanCaptureImage;
  // qDebug() << "anyCanCaptureVideo ==> " << anyCanCaptureVideo;

  if (mode == 1) {
    return anyCanCaptureImage;
  } else if (mode == 2) {
    return anyCanCaptureVideo;
  }

  return false;
}

void MainWindow::takeAllCapture() {
  qDebug() << "takeAllCapture()";

  updateCaptureMode(0);

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
  QImage scaled = img.scaled(ui->viewfinder->size(), Qt::KeepAspectRatio,
                             Qt::SmoothTransformation);

  // 会导致 ui->stackedWidget->setCurrentIndex(1); 进入"预览"页面
  ui->lastImagePreviewLabel->setPixmap(QPixmap::fromImage(scaled));

  displayCapturedImage();

  QTimer::singleShot(3000, this, SLOT(displayViewfinder()));
}

void MainWindow::saveCapturedImage(const QString& devKey,
                                   const QString& fileName) {
  Q_UNUSED(fileName);

  isCapturingImage = false;
  if (applicationExiting) {
    close();
  }
}

void MainWindow::displayCaptureError(const QString& devKey,
                                     const QString& errorString) {
  QMessageBox::warning(this, tr("%1 image capture error").arg(devKey),
                       errorString);
  isCapturingImage = false;
}
// ########## handler capture signals ##########

// 录制所有勾选的相机
void MainWindow::startAllRecording() {
  // qDebug() << "startAllRecording()";

  updateCaptureMode(1);

  // 类似 _isRunning
  // isCapturingImage = true;
  // 不能设置，record
  // 是一个持续性动作，一旦开始，除非停止，否则不会结束，回导致无法退出程序
  for (CameraDevice* cam : activeCameras.values()) {
    // qDebug() << cam->deviceKey();
    cam->startRecording();
  }
}

void MainWindow::pauseAllRecording() {
  for (CameraDevice* cam : activeCameras.values()) cam->pauseRecording();

  this->updateRecordTime();
}

// 停止录制
void MainWindow::stopAllRecording() {
  for (CameraDevice* cam : activeCameras.values()) cam->stopRecording();

  this->updateRecordTime();
}

void MainWindow::setAllMuted(bool muted) {
  for (CameraDevice* cam : activeCameras.values()) cam->setMuted(muted);
}

void MainWindow::updateRecordTime() {
  QStringList messages;
  for (auto it = activeCameras.begin(); it != activeCameras.end(); ++it) {
    qint64 dur = it.value()->recordDuration();
    messages << QString("%1: %2 sec").arg(it.key()).arg(dur / 1000);
  }
  ui->statusbar->showMessage(messages.join(" | "));
}

// ########## handler recorder signals ##########
void MainWindow::updateRecorderState(const QString& devKey,
                                     const QMediaRecorder::State& state) {
  recorderStates[devKey] = state;  // 保存状态
  refreshRecorderControls();  // 刷新按钮状态
  qDebug() << "updateRecorderState: " << devKey << state;
}

void MainWindow::refreshRecorderControls() {
  bool anyRecording = false;
  bool anyPaused    = false;
  bool anyStopped   = false;

  for (auto state : recorderStates.values()) {
    switch (state) {
      case QMediaRecorder::RecordingState:
        anyRecording = true;
        break;
      case QMediaRecorder::PausedState:
        anyPaused = true;
        break;
      case QMediaRecorder::StoppedState:
        anyStopped = true;
        break;
    }
  }

  // 统一控制按钮
  // recordButton：如果没有任何相机在录制，或者有相机暂停，都可用
  ui->recordButton->setEnabled(!anyRecording || anyPaused);

  // pauseButton：如果至少一个相机正在录制，则可用
  ui->pauseButton->setEnabled(anyRecording);

  // stopButton：如果至少有一个相机正在录制或暂停，则可用
  ui->stopButton->setEnabled(anyRecording || anyPaused);
}

void MainWindow::displayRecorderError(const QString& devKey,
                                      const QString& errorString) {
  QMessageBox::warning(this, tr("%1 recorder error").arg(devKey), errorString);
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
        cam->searchAndLock();
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
        cam->unlock();
      }
      break;
    default:
      QMainWindow::keyReleaseEvent(event);
  }
}

void MainWindow::toggleLock() {
  for (CameraDevice* cam : activeCameras.values()) {
    switch (cam->lockStatus()) {
      case QCamera::Searching:
      case QCamera::Locked:
        cam->unlock();
        break;
      case QCamera::Unlocked:
        cam->searchAndLock();
    }
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
  qDebug() << "configureCameraSettings";

  if (activeCameras.isEmpty()) return;

  // 选择第一个相机作为设置窗口的参考
  CameraDevice* firstCam = activeCameras.first();
  if (!firstCam) return;

  switch (firstCam->captureMode()) {
    case QCamera::CaptureStillImage:
      configureImageSettings();  // 里面已经批量应用到所有相机
      break;

    case QCamera::CaptureVideo:
      configureVideoSettings();  // 里面已经批量应用到所有相机
      break;

    default:
      break;
  }
}

void MainWindow::configureVideoSettings() {
  // 弹出一次设置对话框，基于第一个相机初始化
  CameraDevice* firstCam = nullptr;
  if (!activeCameras.isEmpty()) firstCam = activeCameras.first();

  if (!firstCam) return;

  qDebug() << "configureVideoSettings";

  VideoSettings settingsDialog(firstCam->mediaRecorder());
  settingsDialog.setAudioSettings(audioSettings);
  settingsDialog.setVideoSettings(videoSettings);
  settingsDialog.setFormat(videoContainerFormat);

  if (settingsDialog.exec()) {
    audioSettings        = settingsDialog.audioSettings();
    videoSettings        = settingsDialog.videoSettings();
    videoContainerFormat = settingsDialog.format();

    for (CameraDevice* cam : activeCameras.values()) {
      cam->mediaRecorder()->setEncodingSettings(audioSettings, videoSettings,
                                                videoContainerFormat);
    }
  }
}

void MainWindow::configureImageSettings() {
  CameraDevice* firstCam = nullptr;
  if (!activeCameras.isEmpty()) firstCam = activeCameras.first();

  if (!firstCam) return;

  qDebug() << "configureImageSettings";

  ImageSettings settingsDialog(firstCam->imageCapturer());
  settingsDialog.setImageSettings(imageSettings);

  if (settingsDialog.exec()) {
    imageSettings = settingsDialog.imageSettings();

    for (CameraDevice* cam : activeCameras.values()) {
      cam->imageCapturer()->setEncodingSettings(imageSettings);
    }
  }
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
