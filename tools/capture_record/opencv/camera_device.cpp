#include "tools/capture_record/opencv/camera_device.h"

// #include <QMediaCaptureSession>
#include <QDebug>
#include <QDir>
#include <QMediaMetaData>
#include <QSizePolicy>
#include <QUrl>

#include "tools/capture_record/common/utils.h"

CameraDevice::CameraDevice(const QString& device, QWidget* viewfinderParent,
                           QWidget* parent)
    : QObject(parent) {
  devKey = device;
  // qDebug() << device;

  // 创建父容器里的子容器
  containerWidget = new QWidget(viewfinderParent);
  containerLayout = new QVBoxLayout(containerWidget);
  containerLayout->setContentsMargins(0, 0, 0, 0);

  viewLabel = new QLabel(containerWidget);
  viewLabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  viewLabel->setAlignment(Qt::AlignCenter);
  viewLabel->setStyleSheet("background-color: black; border: 1px solid gray;");
  viewLabel->setMinimumSize(200, 150);

  containerLayout->addWidget(viewLabel);

  connect(
      this, &CameraDevice::frameReady, viewLabel,
      [this](const QImage& frame) {
        previewUpdatePending.store(false, std::memory_order_release);
        if (!frame.isNull() && viewLabel) {
          viewLabel->setPixmap(QPixmap::fromImage(frame).scaled(
              viewLabel->size(), Qt::KeepAspectRatio,
              Qt::SmoothTransformation));
        }
      },
      Qt::QueuedConnection);

  lastImg = QImage(width_, height_, QImage::Format_RGB888);

  // 用于初始化，实际并没有在这里输出
  recordTimer.start();

  // qDebug() << "[CameraDevice] CameraDevice created for device:" << devKey;
}

CameraDevice::~CameraDevice() {
  close();
  delete viewLabel;
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
  // qDebug() << "Exposure compensation set to:" << pendingExposureIndex;
}

void CameraDevice::open() {
  if (ready.load(std::memory_order_acquire)) return;

  session_state_.Close();
  if (!session_state_.OpenStarted()) return;
  if (!cap.open(devKey.toStdString())) {
    qWarning() << "[CameraDevice] Failed to open device:" << devKey;
    ready = false;
    session_state_.OpenFailed();
    emit cameraState(devKey, false);
    return;
  }

  ready = true;

  // 默认模式
  recording = RecordState::Stopped;

  // 这个宽高是 摄像头驱动/解码后实际输出的尺寸。
  // 即使你手动 cap.set(cv::CAP_PROP_FRAME_WIDTH, xxx)，
  // 驱动可能不会完全遵守，只会选择它支持的分辨率（比如 640×480、1280×720、1920×1080）。
  device_width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
  device_height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  fps = cap.get(cv::CAP_PROP_FPS);
  if (fps <= 0) fps = 10;  // 有些相机会返回 0，兜底
  // qDebug() << "[CameraDevice] Camera fps:" << fps;

  if (!session_state_.OpenSucceeded()) {
    cap.release();
    ready = false;
    session_state_.OpenFailed();
    emit cameraState(devKey, false);
    return;
  }

  // OpenCV 采集线程  ==> camera->start();
  quit_flag = false;
  workerThread = QThread::create([this] { run(); });
  workerThread->start();
  emit cameraState(devKey, true);
}

void CameraDevice::close() {
  if (isRecording() || writer.isOpened()) stopRecording();

  // 关闭 采集线程; 会在下一次循环开始时停止
  quit_flag = true;
  if (workerThread) {
    workerThread->wait();
    delete workerThread;
    workerThread = nullptr;
  }

  {
    QMutexLocker writerLocker(&writerMutex);
    if (writer.isOpened()) writer.release();
  }

  // ==> camera->stop();
  if (cap.isOpened()) cap.release();
  ready = false;
  capturing = false;
  recording = RecordState::Stopped;
  session_state_.Close();
  emit cameraState(devKey, false);
}

void CameraDevice::startRecording() {
  if (!ready.load(std::memory_order_acquire) || isRecording()) return;
  if (recording.load(std::memory_order_acquire) != RecordState::Stopped ||
      writer.isOpened()) {
    stopRecording();
  }

  recording = RecordState::Starting;
  emit recorderState(devKey, RecordState::Starting);
  if (!session_state_.StartRecording(
          static_cast<std::size_t>(segmentMilliseconds / 1000)) ||
      !openNewWriter()) {
    session_state_.WriterFailed();
    recording = RecordState::Error;
    emit recordError(devKey, 4);
    emit recorderState(devKey, RecordState::Error);
    return;
  }
  if (!session_state_.WriterOpened()) {
    writer.release();
    session_state_.WriterFailed();
    recording = RecordState::Error;
    emit recordError(devKey, 4);
    emit recorderState(devKey, RecordState::Error);
    return;
  }

  recordElapsedSeconds = 0;
  encoded_frame_count_ = 0;
  first_record_loop = true;
  recording = RecordState::Recording;
  capturing = false;
  emit recorderState(devKey, RecordState::Recording);
}

void CameraDevice::startAutoRecording() {
  if (!ready.load(std::memory_order_acquire) || isRecording()) return;
  if (recording.load(std::memory_order_acquire) != RecordState::Stopped ||
      writer.isOpened()) {
    stopRecording();
  }

  recording = RecordState::Starting;
  emit recorderState(devKey, RecordState::Starting);
  if (!session_state_.StartRecording(
          static_cast<std::size_t>(segmentMilliseconds / 1000)) ||
      !openNewWriter()) {
    session_state_.WriterFailed();
    recording = RecordState::Error;
    emit recordError(devKey, 4);
    emit recorderState(devKey, RecordState::Error);
    return;
  }
  if (!session_state_.WriterOpened()) {
    writer.release();
    session_state_.WriterFailed();
    recording = RecordState::Error;
    emit recordError(devKey, 4);
    emit recorderState(devKey, RecordState::Error);
    return;
  }

  recordElapsedSeconds = 0;
  encoded_frame_count_ = 0;
  first_record_loop = true;
  recording = RecordState::AutoRecording;
  capturing = false;
  emit recorderState(devKey, RecordState::AutoRecording);
}

bool CameraDevice::openNewWriter() {
  /* // 先关闭旧的 writer
  if (writer.isOpened()) {
    writer.release();
  }
  */

  // 具体的文件
  QString filePath = generateVideoFilePath();
  if (filePath.isEmpty()) return false;

  // way 2
  // VideoWriter::open(...) 里传的 cv::Size(width, height) 是写入视频的帧大小。
  // 使用 MP4 H264 (需要系统支持)
  try {
    writer.open(filePath.toStdString(), cv::CAP_FFMPEG,
                cv::VideoWriter::fourcc('a', 'v', 'c', '1'), fps,
                cv::Size(width_, height_), true);
  } catch (const cv::Exception& e) {
    qWarning() << "[CameraDevice] Failed to open VideoWriter:" << e.what();
    return false;
  }

  if (!writer.isOpened()) {
    qWarning() << "[CameraDevice] Failed to open VideoWriter";
    return false;
  }

  // qDebug() << "[CameraDevice] openNewWriter:" << filePath;
  // qDebug() << "[CameraDevice] openNewWriter: fps: " << fps;
  return true;
}

void CameraDevice::pauseRecording() {
  // this->updateRecordTime();
  this->stopRecording();
}

void CameraDevice::stopRecording() {
  if (recording.load(std::memory_order_acquire) == RecordState::Stopped &&
      !writer.isOpened()) return;

  const qint64 finalDuration =
      recordElapsedSeconds.load(std::memory_order_acquire);
  recording = RecordState::Stopping;
  emit recorderState(devKey, RecordState::Stopping);
  {
    QMutexLocker writerLocker(&writerMutex);
    if (writer.isOpened()) writer.release();
  }
  first_record_loop = false;
  session_state_.StopRecording();
  recording = RecordState::Stopped;
  emit recordTime(devKey, finalDuration);
  emit recorderState(devKey, RecordState::Stopped);
}

void CameraDevice::updateCustomSettings(const CustomSettings& setings) {
  stopRecording();

  savePath = setings.filePath;
  fps      = setings.fps;
  width_   = setings.width;
  height_  = setings.height;
  // 自动切分时间间隔
  segmentMilliseconds = setings.segmentSeconds * 1000;

  // qDebug() << "[CameraDevice] updateCustomSettings path:" << setings.filePath;
  // qDebug() << "[CameraDevice] updateCustomSettings sseconds:"
  //          << setings.segmentSeconds;
}

void CameraDevice::setCaptureMode(int mode) {
  if (mode == 1) {
    capturing = true;
  } else if (mode == 2) {
    capturing = false;
  } else {
    capturing = false;
  }

  this->readyForCapture(ready.load(std::memory_order_acquire) &&
                        capturing.load(std::memory_order_acquire));
  emit recorderState(devKey, recording.load(std::memory_order_acquire));

  // qDebug() << "[CameraDevice] Setting capture mode to:" << mode;
}

void CameraDevice::takeImage() {
  if (!cap.isOpened()) return;

  if (!capturing.load(std::memory_order_acquire)) return;

  QMutexLocker locker(&frameMutex);
  if (lastFrameBgr.empty()) return;
  cv::Mat tmp_bgr = lastFrameBgr.clone();
  cv::Mat tmp_rgb = lastFrameRgb.clone();
  QImage tmp_img  = lastImg.copy();  // 强制深拷贝（立即复制内存）
  locker.unlock();

  processCapturedImage(tmp_img, tmp_bgr);  // BGR \ RGB

  // qDebug() << "[CameraDevice] Photo captured";
}

// ##########  发送信号给主窗口  ##########
void CameraDevice::readyForCapture(bool ready) {
  emit readyCapture(devKey, ready);
}

void CameraDevice::processCapturedImage(const QImage& img,
                                        const cv::Mat& frame) {
  // 自己实现文件命名逻辑
  QString filePath = generateImageFilePath();

  // 保存图片到磁盘
  cv::imwrite(filePath.toStdString(), frame);
  emit saveImageSuc(devKey, filePath);  // 通知主窗口

  // 将原始图像直接发给主窗口，由主窗口处理显示
  emit previewAvailable(devKey, img);

  // qDebug() << "[CameraDevice]" << devKey
  //          << "captured image, saving to:" << filePath;
  // qDebug() << "[CameraDevice]" << "Image size:" << img.size();
}

void CameraDevice::updateRecordTime() {
  qDebug() << "[CameraDevice] updateRecordTime:" << devKey << recordDuration();

  emit recordTime(devKey, recordDuration());
}
// ##########  发送信号给主窗口  ##########

// 采集循环
void CameraDevice::run() {
  cv::Mat frame, frame_resized, frame_rgb;
  QImage img(width_, height_, QImage::Format_RGB888);

  // int segmentFrames = static_cast<int>(std::round(fps * settings.segmentSeconds));
  // int frameIntervalMs = static_cast<int>(std::round(1000.0 / fps));
  double frameIntervalMs = 1000.0 / fps;  // 33.333... ms

  int frameCount = 0;
  auto lastTime  = std::chrono::steady_clock::now();

  // way 2 累积目标时间戳（基于 nextFrameTime） 全局对齐 基本平齐 fps
  auto nextStart = std::chrono::steady_clock::now();

  while (!quit_flag) {
    // way 1 逐帧自适应  始终比 fps 小一点点
    // auto loopStart = std::chrono::steady_clock::now();

    if (!ready.load(std::memory_order_acquire)) {
      QThread::msleep(300);
      continue;
    }

    // OpenCV VideoCapture 默认输出 BGR，不是 RGB。
    if (!cap.read(frame) || frame.empty()) {
      QThread::msleep(100);
      continue;
    }

    if (width_ != device_width || height_ != device_height) {
      cv::resize(frame, frame_resized, cv::Size(width_, height_));
    } else {
      frame_resized = frame;
    }

    // Qt 显示，因为 QImage::Format_RGB888 需要 RGB
    cv::cvtColor(frame_resized, frame_rgb, cv::COLOR_BGR2RGB);

    // way 1
    matToQImage(frame_rgb, img);

    // way 2 用 shared_ptr 包装 QImage，不拷贝
    // matToQImage(frame_rgb);

    // 保存最新一帧
    {
      QMutexLocker locker(&frameMutex);
      lastFrameBgr = frame_resized;  // BGR
      lastFrameRgb = frame_rgb;  // RGB
      lastImg      = img;  // QImage
    }

    // 计时器的初始化；写入视频
    if (this->isRecording()) {
      QMutexLocker writerLocker(&writerMutex);
      if (!writer.isOpened()) continue;
      if (first_record_loop == true) {
        // segmentFrames = static_cast<int>(std::round(fps * segmentSeconds));
        frameIntervalMs = 1000.0 / fps;
        recordTimer.restart();
        recordElapsedSeconds = 0;
        first_record_loop    = false;
      }

      if (recording.load(std::memory_order_acquire) ==
          RecordState::AutoRecording) {
        // 检查是否需要切分文件
        if (recordTimer.isValid() &&
            recordTimer.elapsed() >= segmentMilliseconds + 1) {
          // qDebug() << "实际切分帧数:" << frameCount;
          // 结束上一段数据录制，重新启动 视频写入器
          writer.release();

          if (!openNewWriter()) {
            session_state_.WriterFailed();
            recording = RecordState::Error;
            emit recordError(devKey, 4);
            emit recorderState(devKey, RecordState::Error);
            continue;
          }

          // 重新启动定时器
          recordTimer.restart();
          recordElapsedSeconds = 0;
          frameCount           = 0;
        }
      }

      // 写入当前帧
      auto writeStart = std::chrono::steady_clock::now();
      try {
        writer.write(frame_resized);
      } catch (const cv::Exception& e) {
        qWarning() << "[CameraDevice] VideoWriter write failed:" << e.what();
        writer.release();
        session_state_.WriterFailed();
        recording = RecordState::Error;
        emit recordError(devKey, 4);
        emit recorderState(devKey, RecordState::Error);
        continue;
      }
      encoded_frame_count_.fetch_add(1, std::memory_order_relaxed);
      recordElapsedSeconds = recordTimer.elapsed() / 1000;
      auto writeEnd        = std::chrono::steady_clock::now();

      // 可选：打印写入耗时（ms）与当前段的帧数，用来诊断性能瓶颈
      // qDebug() << "[CameraDevice] write_ms:"
      //          << std::chrono::duration_cast<std::chrono::milliseconds>(
      //                 writeEnd - writeStart)
      //                 .count()
      //          << "frameCount:" << frameCount;
    }
    frameCount++;

    // 当正在录制时，线程睡眠的时间 = 每帧间隔 = 1000 / fps 毫秒（fps 是帧率）
    // 例如 fps = 30 → 1000 / 30 ≈ 33 ms，也就是每 33 毫秒处理一帧
    // 当没有录制时，线程睡眠 30 ms（休眠更长一点，减少 CPU 占用）

    /* // way 1 
    auto loopEnd = std::chrono::steady_clock::now();
    int elapsedMs =
        static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                             loopEnd - loopStart)
                             .count());
    int toSleep = frameIntervalMs - elapsedMs;
    if (toSleep > 0) {
      QThread::msleep(toSleep);
    } else {
      // 如果处理已经超时（toSleep <= 0），不要再睡了，直接下一循环。
      // 可选：短暂让出，避免 100% CPU：
      QThread::yieldCurrentThread();
    }
    */

    // way 2 累积目标时间
    nextStart +=
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(frameIntervalMs));
    auto loopEnd = std::chrono::steady_clock::now();
    if (loopEnd < nextStart) {
      // 还没到下一帧目标时间 → 等一下
      auto toSleep = std::chrono::duration_cast<std::chrono::milliseconds>(
                         nextStart - loopEnd)
                         .count();
      QThread::msleep(toSleep);
    } else {
      // 已经超时 → 跟进节奏，避免越拖越慢
      nextStart = loopEnd;
      QThread::yieldCurrentThread();
    }

    // /* // Debug
    auto now = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration<double>(now - lastTime).count();  // 精确到小数秒
    if (elapsed >= 1.0) {
      qDebug() << "实际采集帧率:" << frameCount / elapsed << "Hz";
      frameCount = 0;
      lastTime   = now;
    }
    // */
  }
}

QString CameraDevice::generateVideoFilePath() {
  // 确保 savePath 非空
  if (savePath.isEmpty()) savePath = QDir::homePath();

  // 只取设备名最后部分
  const QString devName = QFileInfo(devKey).fileName();
  const QString dateDir = QDateTime::currentDateTime().toString("yyyy-MM-dd");
  const QString fullDirPath = savePath + "/" + dateDir + "/" + devName + "/video";
  const QString filePath = generateUniqueMediaFilePath(fullDirPath, "mp4");
  if (filePath.isEmpty()) {
    qWarning() << "[CameraDevice] Failed to create unique video path in"
               << fullDirPath;
  }
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
  // QString timestamp = QDateTime::currentDateTime().toString("hh-mm-ss");
  QString timestamp =
      QString::number(QDateTime::currentDateTime().toMSecsSinceEpoch());
  QString fileName = QString("%2.jpg").arg(timestamp);
  QString filePath = fullDirPath + "/" + fileName;

  return filePath;
}

void CameraDevice::matToQImage(const cv::Mat& frame, QImage& img) {
  if (frame.empty()) return;

  cv::Mat rgb = frame;
  // cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

  // 保证 img 和 frame 尺寸、通道匹配
  if (img.size() != QSize(frame.cols, frame.rows) ||
      img.format() != QImage::Format_RGB888) {
    img = QImage(frame.cols, frame.rows, QImage::Format_RGB888);
  }

  // 直接 memcpy 到 QImage 内存
  memcpy(img.bits(), rgb.data, rgb.total() * rgb.elemSize());

  showImageOnLabel(img);
}

void CameraDevice::matToQImage(const cv::Mat& frame) {
  if (frame.empty()) return;

  cv::Mat rgb = frame;
  // cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);

  // way 2 用 shared_ptr 包装 QImage，不拷贝
  auto img = std::make_shared<QImage>(rgb.data, rgb.cols, rgb.rows,
                                      static_cast<int>(rgb.step),
                                      QImage::Format_RGB888);

  showImageOnLabel(*img);
}

// 在 CameraDevice 中可以加 resizeEvent 处理：
// 在 Qt 窗口大小变化时（ResizeEvent）更新显示内容
void CameraDevice::resizeEvent(QResizeEvent* event) {
  // showImageOnLabel();
}

void CameraDevice::showImageOnLabel(QImage& qImg) {
  if (qImg.isNull() || !viewLabel) return;

  bool expected = false;
  if (!previewUpdatePending.compare_exchange_strong(
          expected, true, std::memory_order_acq_rel)) {
    return;
  }
  emit frameReady(qImg.copy());
}

qint64 CameraDevice::recordDuration() const {
  if (!isRecording()) return 0;
  return recordElapsedSeconds.load(std::memory_order_acquire);
}
