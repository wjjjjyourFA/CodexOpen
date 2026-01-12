#include "tools/capture_record/qt5/camera_device.h"

// #include <QMediaCaptureSession>
#include <QDebug>
#include <QDir>
#include <QMediaMetaData>
#include <QUrl>

CameraDevice::CameraDevice(const QCameraInfo& info, QWidget* viewfinderParent,
                           QWidget* parent)
    : QObject(parent), cameraInfo(info), camera(new QCamera(info, this)) {
  devKey = info.deviceName();

  // 创建父容器里的子容器
  containerWidget = new QWidget(viewfinderParent);
  containerLayout = new QVBoxLayout(containerWidget);
  containerLayout->setContentsMargins(0, 0, 0, 0);

  viewfinder = new QCameraViewfinder(containerWidget);
  viewfinder->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  viewfinder->setMinimumSize(200, 150);

  containerLayout->addWidget(viewfinder);

  // clang-format off
  // 相机绑定 viewfinder
  camera->setViewfinder(viewfinder);
  connect(camera, SIGNAL(stateChanged(QCamera::State)), this, SLOT(updateCameraState(QCamera::State)));
  connect(camera, SIGNAL(error(QCamera::Error)), this, SLOT(displayCameraError()));
  connect(camera, SIGNAL(lockStatusChanged(QCamera::LockStatus, QCamera::LockChangeReason)), this, SLOT(updateLockStatus(QCamera::LockStatus, QCamera::LockChangeReason)));

  capturer = new QCameraImageCapture(camera, this);
  connect(capturer, SIGNAL(readyForCaptureChanged(bool)), this, SLOT(readyForCapture(bool)));
  connect(capturer, SIGNAL(imageCaptured(int, QImage)), this, SLOT(processCapturedImage(int, QImage)));
  // connect(capturer, SIGNAL(imageSaved(int, QString)), this, SLOT(saveCapturedImage(int, QString)));
  connect(capturer, SIGNAL(error(int, QCameraImageCapture::Error, QString)), this, SLOT(displayCaptureError(int,QCameraImageCapture::Error, QString)));

  // /* // 相机绑定 recorder
  recorder = new QMediaRecorder(camera, this);
  recorder->setMetaData(QMediaMetaData::Title, QVariant(QLatin1String("Test Video")));
  connect(recorder, SIGNAL(stateChanged(QMediaRecorder::State)), this, SLOT(updateRecorderState(QMediaRecorder::State)));
  connect(recorder, SIGNAL(durationChanged(qint64)), this, SLOT(updateRecordTime()));
  connect(recorder, SIGNAL(error(QMediaRecorder::Error)), this, SLOT(displayRecorderError()));
  // */

  qDebug() << "[CameraDevice] CameraDevice created for device:" << info.deviceName() << info.description();
  // clang-format on
}

CameraDevice::~CameraDevice() {
  stop();
  delete recorder;
  delete capturer;
  delete camera;
  delete viewfinder;
  delete containerWidget;
}

void CameraDevice::setSavePath(const QString& Path) {
  savePath = Path;
  // qDebug() << "[CameraDevice]" << "savePath:" << savePath;
}

void CameraDevice::setExposureCompensation(int index) {
  pendingExposureIndex = index;
  exposureDebounceTimer.start();  // 每次滑动都重新计时
  // qDebug() << "Exposure compensation set to: " << index;
}

void CameraDevice::initExposureSlider(QSlider* slider) {
  connect(slider, &QSlider::valueChanged, this,
          &CameraDevice::setExposureCompensation);

  exposureDebounceTimer.setSingleShot(true);
  exposureDebounceTimer.setInterval(150);  // 150ms 防抖
  connect(&exposureDebounceTimer, &QTimer::timeout, this,
          &CameraDevice::applyExposureCompensation);
}

void CameraDevice::applyExposureCompensation() {
  if (!camera) return;
  camera->exposure()->setExposureCompensation(pendingExposureIndex * 0.5);
  // qDebug() << "Exposure compensation set to:" << pendingExposureIndex;
}

void CameraDevice::start() {
  if (camera->state() == QCamera::ActiveState) return;  // Qt5 判断
  camera->start();

  qDebug() << "[CameraDevice] Camera started:" << cameraInfo.deviceName();
}

void CameraDevice::stop() {
  if (isRecording()) {
    stopRecording();
  }

  if (camera->state() == QCamera::ActiveState) {
    camera->stop();

    qDebug() << "[CameraDevice] Camera stopped:" << cameraInfo.description();
  }
}

void CameraDevice::startRecording() {
  if (camera->state() != QCamera::ActiveState) {
    camera->start();
  } else if (camera->state() == QCamera::ActiveState) {
    /*  暂时没有改变 笔记本 摄像头 不能录视频的问题
    camera->stop();
    camera->setCaptureMode(QCamera::CaptureVideo);
    camera->start();
    */
  }

  // 确保相机处于录像模式
  // /*
  if (camera->captureMode() != QCamera::CaptureVideo) return;
  // */

  // 具体的文件
  QString filePath = generateVideoFilePath();

  // /* // way 1
  // 先停止，确保 recorder 可用
  // recorder->stop();
  // qDebug() << "Recording stopped:";

  recorder->setContainerFormat("mp4");
  recorder->setOutputLocation(QUrl::fromLocalFile(filePath));
  recorder->record();
  // */

  qDebug() << "Recorder state:" << recorder->state();
  qDebug() << "Recorder error:" << recorder->error() << recorder->errorString();
  qDebug() << "[CameraDevice] Recording started:" << filePath;
}

void CameraDevice::pauseRecording() {
  if (recorder->state() == QMediaRecorder::RecordingState) {
    updateRecordTime();  // 更新录制时间
    recorder->pause();
  }
}

void CameraDevice::stopRecording() {
  // /* // way 1
  if (recorder->state() == QMediaRecorder::RecordingState ||
      recorder->state() == QMediaRecorder::PausedState) {
    updateRecordTime();  // 更新录制时间
    recorder->stop();

    qDebug() << "[CameraDevice] Recording stopped:" << cameraInfo.description();
  }
  // */
}

void CameraDevice::setMuted(bool muted) { recorder->setMuted(muted); }

void CameraDevice::setCaptureMode(QCamera::CaptureModes mode) {
  // 检查相机是否支持这个模式
  if (camera->isCaptureModeSupported(mode)) {
    // enum CaptureMode { CaptureStillImage = 0x1, CaptureVideo = 0x2 };
    qDebug() << "[CameraDevice] Setting capture mode to:" << mode;

    camera->setCaptureMode(mode);

    // QCamera::CaptureModes current = camera->captureMode();
    // qDebug() << "[CameraDevice] 设置后当前模式:" << current;
  } else {
    qWarning() << "[CameraDevice] Camera does not support capture mode:"
               << mode;
  }
}

void CameraDevice::takeImage() {
  if (camera->captureMode() != QCamera::CaptureStillImage) return;

  capturer->capture();
}

// ##########  发送信号给主窗口  ##########
void CameraDevice::updateCameraState(const QCamera::State& state) {
  qDebug() << "[CameraDevice]" << devKey << "updateCameraState:" << state;
  emit cameraState(devKey, state);
}

void CameraDevice::displayCameraError() {
  emit cameraError(devKey, camera->errorString());
}

// /*
void CameraDevice::updateLockStatus(const QCamera::LockStatus& status,
                                    const QCamera::LockChangeReason& reason) {
  qDebug() << "[CameraDevice]" << devKey << "updateLockStatus:" << status
           << reason;
  emit cameraLockState(devKey, status, reason);
}
// */

void CameraDevice::readyForCapture(bool ready) {
  emit readyCapture(devKey, ready);
}

void CameraDevice::processCapturedImage(int request_id, const QImage& img) {
  Q_UNUSED(request_id);

  // 自己实现文件命名逻辑
  QString filePath = generateImageFilePath();

  // 保存图片到磁盘
  if (img.save(filePath)) {
    emit saveImageSuc(devKey, filePath);  // 通知主窗口
  }

  // 将原始图像直接发给主窗口，由主窗口处理显示
  emit previewAvailable(devKey, img);

  qDebug() << "[CameraDevice]" << devKey
           << "captured image, saving to:" << filePath;
  // qDebug() << "[CameraDevice]" << "Image size:" << img.size();
}

/*  // 已经合并到 processCapturedImage
void CameraDevice::saveCapturedImage(int id, const QString &fileName) {
  Q_UNUSED(id);
  Q_UNUSED(fileName);

  // 自己实现文件命名逻辑
  QString filePath = generateImageFilePath();

  // 保存图片到磁盘
  if (img.save(filePath)) {
    emit saveImageSuc(devKey, filePath);  // 通知主窗口
  }
}
*/

void CameraDevice::displayCaptureError(int id,
                                       const QCameraImageCapture::Error error,
                                       const QString& errStr) {
  Q_UNUSED(id);
  Q_UNUSED(error);

  emit captureError(devKey, errStr);
}

void CameraDevice::updateRecorderState(const QMediaRecorder::State& state) {
  qDebug() << "[CameraDevice]" << devKey << "updateRecorderState:" << state;
  emit recorderState(devKey, state);
}

void CameraDevice::updateRecordTime() {
  emit recordTime(devKey, recorder->duration() / 1000);
}

void CameraDevice::displayRecorderError() {
  emit recorderError(devKey, recorder->errorString());
}
// ##########  发送信号给主窗口  ##########

QString CameraDevice::generateVideoFilePath() {
  // 确保 savePath 非空
  if (savePath.isEmpty()) {
    qWarning() << "savePath is empty!";
    savePath = QDir::homePath();  // 默认用用户主目录
  }

  // 只取设备名最后部分
  QString devName = QFileInfo(devKey).fileName();

  QString dateDir     = QDateTime::currentDateTime().toString("yyyy-MM-dd");
  QString fullDirPath = savePath + "/" + dateDir + "/" + devName + "/video";

  QDir dir;
  if (!dir.exists(fullDirPath)) {
    dir.mkpath(fullDirPath);
  }

  // 自动生成文件名：相机ID_年月日时分秒.mp4
  QString timestamp = QDateTime::currentDateTime().toString("hh-mm-ss");
  QString fileName  = QString("%2.mp4").arg(timestamp);
  QString filePath  = fullDirPath + "/" + fileName;

  return filePath;
}

QString CameraDevice::generateImageFilePath() {
  // 确保 savePath 非空
  if (savePath.isEmpty()) {
    qWarning() << "savePath is empty!";
    savePath = QDir::homePath();  // 默认用用户主目录
  }

  // 只取设备名最后部分
  QString devName = QFileInfo(devKey).fileName();

  QString dateDir     = QDateTime::currentDateTime().toString("yyyy-MM-dd");
  QString fullDirPath = savePath + "/" + dateDir + "/" + devName + "/image";

  QDir dir;
  if (!dir.exists(fullDirPath)) {
    dir.mkpath(fullDirPath);
  }

  // 自动生成文件名：相机ID_年月日时分秒.mp4
  // QString timestamp = QDateTime::currentDateTime().toString("hhmmss");
  QString timestamp =
      QString::number(QDateTime::currentDateTime().toMSecsSinceEpoch());
  QString fileName = QString("%2.jpg").arg(timestamp);
  QString filePath = fullDirPath + "/" + fileName;

  return filePath;
}

void CameraDevice::searchAndLock() {
  if (camera) {
    this->camera->searchAndLock();
  }
};

void CameraDevice::unlock() {
  if (camera) {
    this->camera->unlock();
  }
};
