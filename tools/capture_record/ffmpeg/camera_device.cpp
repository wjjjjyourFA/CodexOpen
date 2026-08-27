#include "tools/capture_record/ffmpeg/camera_device.h"

// #include <QMediaCaptureSession>
#include <QDebug>
#include <QDir>
#include <QMediaMetaData>
#include <QSizePolicy>
#include <QUrl>

#include <algorithm>
#include <cmath>

#include "tools/capture_record/common/utils.h"

CameraDevice::CameraDevice(const QString& device, const CustomSettings& setings,
                           QWidget* viewfinderParent, QWidget* parent)
    : QObject(parent) {
  devKey = device;
  // qDebug() << device;

  // 原来写在参数文件中，现在写在构造函数里，避免加载参数文件；
  config_ = std::make_shared<Config>();
  config_->set_camera_dev(devKey.toStdString());
  // config_->set_pixel_format("yuyv");
  // config_->set_pixel_format("uyvy");
  config_->set_pixel_format("mjpeg");
  config_->set_frame_rate(setings.fps);
  config_->set_width(setings.width);
  config_->set_height(setings.height);
  config_->set_bytes_per_pixel(2);
  config_->set_io_method(IO_METHOD_MMAP);
  config_->set_output_type(RGB);

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

  lastImg = QImage(config_->width(), config_->height(), QImage::Format_RGB888);

  // 用于初始化，实际并没有在这里输出
  recordTimer.start();

  // clang-format off
  CameraImagePtr raw_image_   = std::make_shared<CameraImage>();
  raw_image_->width           = config_->width();
  raw_image_->height          = config_->height();
  raw_image_->bytes_per_pixel = config_->bytes_per_pixel();
  raw_image_->yuv_image.create(raw_image_->height, raw_image_->width, CV_8UC2);
  raw_image_->image.create(raw_image_->height, raw_image_->width, CV_8UC3);
  raw_image = raw_image_;
  // clang-format on

  // Resize
  if (config_->width() != raw_image_->width) {
    // 预分配内存
    raw_image_for_compress =
        cv::Mat(config_->height(), config_->width(), CV_8UC3);
  }

  const auto queueCapacity = std::max<std::size_t>(
      2, static_cast<std::size_t>(std::ceil(config_->frame_rate() * 0.25)));
  queue_ = std::make_shared<FrameQueue<cv::Mat>>(queueCapacity);

  // qDebug() << "[CameraDevice] CameraDevice created for device:" << devKey;
  // clang-format on
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

  v4l2_device = std::make_shared<UsbCamCv>();
  if (!v4l2_device->init(config_) || !v4l2_device->wait_for_device()) {
    qWarning() << "[CameraDevice] Failed to open device:" << devKey;
    ready = false;
    session_state_.OpenFailed();
    emit cameraState(devKey, false);
    return;
  }

  ready = true;
  recording = RecordState::Stopped;
  if (!session_state_.OpenSucceeded()) {
    ready = false;
    session_state_.OpenFailed();
    v4l2_device->ReleaseDevice();
    v4l2_device.reset();
    emit cameraState(devKey, false);
    return;
  }

  quit_flag_run = false;
  workerThread = QThread::create([this] { run_multi(); });
  workerThread->start();
  emit cameraState(devKey, true);
}

void CameraDevice::close() {
  if (isRecording() || writerThread || writer) stopRecording();

  quit_flag_run = true;
  if (workerThread) {
    workerThread->wait();
    delete workerThread;
    workerThread = nullptr;
  }

  if (v4l2_device && v4l2_device->is_capturing()) {
    v4l2_device->ReleaseDevice();
  }
  v4l2_device.reset();
  ready = false;
  capturing = false;
  recording = RecordState::Stopped;
  session_state_.Close();
  emit cameraState(devKey, false);
}

void CameraDevice::startRecording() {
  if (!ready.load(std::memory_order_acquire) || isRecording()) return;
  if (recording.load(std::memory_order_acquire) != RecordState::Stopped ||
      writerThread || writer) {
    stopRecording();
  }

  recording = RecordState::Starting;
  emit recorderState(devKey, RecordState::Starting);
  if (!session_state_.StartRecording(static_cast<std::size_t>(segmentSeconds)) ||
      !openNewWriter(segmentSeconds)) {
    session_state_.WriterFailed();
    recording = RecordState::Error;
    emit recordError(devKey, 4);
    emit recorderState(devKey, RecordState::Error);
    return;
  }
  if (!session_state_.WriterOpened()) {
    writer->abort();
    writer.reset();
    session_state_.WriterFailed();
    recording = RecordState::Error;
    emit recordError(devKey, 4);
    emit recorderState(devKey, RecordState::Error);
    return;
  }

  recordElapsedSeconds = 0;
  encoded_frame_count_ = 0;
  recordTimer.restart();
  queue_->reset();
  quit_flag_encode = false;
  recording = RecordState::Recording;
  capturing = false;
  writerThread = QThread::create([this] { encode(); });
  writerThread->start();
  emit recorderState(devKey, RecordState::Recording);
}

void CameraDevice::startAutoRecording() {
  if (!ready.load(std::memory_order_acquire) || isRecording()) return;
  if (recording.load(std::memory_order_acquire) != RecordState::Stopped ||
      writerThread || writer) {
    stopRecording();
  }

  recording = RecordState::Starting;
  emit recorderState(devKey, RecordState::Starting);
  if (!session_state_.StartRecording(static_cast<std::size_t>(segmentSeconds)) ||
      !openNewWriter(segmentSeconds)) {
    session_state_.WriterFailed();
    recording = RecordState::Error;
    emit recordError(devKey, 4);
    emit recorderState(devKey, RecordState::Error);
    return;
  }
  if (!session_state_.WriterOpened()) {
    writer->abort();
    writer.reset();
    session_state_.WriterFailed();
    recording = RecordState::Error;
    emit recordError(devKey, 4);
    emit recorderState(devKey, RecordState::Error);
    return;
  }

  recordElapsedSeconds = 0;
  encoded_frame_count_ = 0;
  recordTimer.restart();
  queue_->reset();
  quit_flag_encode = false;
  recording = RecordState::AutoRecording;
  capturing = false;
  writerThread = QThread::create([this] { encode(); });
  writerThread->start();
  emit recorderState(devKey, RecordState::AutoRecording);
}

bool CameraDevice::openNewWriter(int segmentSeconds) {
  if (segmentSeconds <= 0) return false;

  // 具体的文件
  QString filePath = generateVideoFileRoot();

  std::shared_ptr<FFmpegWriter> candidate;
  try {
    candidate = std::make_shared<FFmpegWriter>(
        config_->width(), config_->height(), config_->frame_rate(),
        segmentSeconds, filePath.toStdString());
  } catch (const std::exception& e) {
    qWarning() << "[CameraDevice] Failed to start FFmpeg:" << e.what();
    return false;
  }

  if (!candidate->isInited()) {
    qWarning() << "[CameraDevice] Failed to open VideoWriter:"
               << QString::fromStdString(candidate->lastError());
    return false;
  }
  writer = std::move(candidate);

  // qDebug() << "[CameraDevice] openNewWriter:" << filePath;
  // qDebug() << "[CameraDevice] openNewWriter: fps: " << config_->frame_rate();
  return true;
}

void CameraDevice::pauseRecording() {
  // this->updateRecordTime();
  this->stopRecording();
}

void CameraDevice::stopRecording() {
  if (recording.load(std::memory_order_acquire) == RecordState::Stopped &&
      !writerThread && !writer) return;

  const qint64 finalDuration =
      recordElapsedSeconds.load(std::memory_order_acquire);
  recording = RecordState::Stopping;
  emit recorderState(devKey, RecordState::Stopping);

  quit_flag_encode = true;
  queue_->stop();
  if (writerThread) {
    writerThread->wait();
    delete writerThread;
    writerThread = nullptr;
  }

  bool publishOk = true;
  QString errorMessage;
  if (writer) {
    publishOk = writer->close();
    if (!publishOk) errorMessage = QString::fromStdString(writer->lastError());
    writer.reset();
  }
  queue_->clear();
  emit recordTime(devKey, finalDuration);

  if (!publishOk) {
    session_state_.WriterFailed();
    recording = RecordState::Error;
    qWarning() << "[CameraDevice] Failed to finalize recording:" << errorMessage;
    emit recordError(devKey, 4);
    emit recorderState(devKey, RecordState::Error);
    return;
  }

  session_state_.StopRecording();
  recording = RecordState::Stopped;
  emit recorderState(devKey, RecordState::Stopped);
}

void CameraDevice::updateCustomSettings(const CustomSettings& setings) {
  stopRecording();

  savePath = setings.filePath;
  // 无法修改 fps，和 分辨率
  // config_->set_frame_rate(setings.fps);
  // config_->set_width(setings.width);
  // config_->set_height(setings.height);
  // 自动切分时间间隔
  segmentSeconds = setings.segmentSeconds;

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
  if (!v4l2_device || !v4l2_device->is_capturing()) return;

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
  // qDebug() << "[CameraDevice] updateRecordTime:" << devKey << recordDuration();

  emit recordTime(devKey, recordDuration());
}
// ##########  发送信号给主窗口  ##########

// 多线程 采集循环
void CameraDevice::run_multi() {
  cv::Mat frame, frame_resized, frame_rgb;
  QImage img(config_->width(), config_->height(), QImage::Format_RGB888);

  // int segmentFrames = static_cast<int>(std::round(fps * settings.segmentSeconds));
  double frameIntervalMs = 1000.0 / config_->frame_rate();
  // qDebug() << "[CameraDevice] run() frame_rate:" << config_->frame_rate();
  // qDebug() << "[CameraDevice] run() frameIntervalMs:" << frameIntervalMs;

  int frameCount = 0;
  auto lastTime  = std::chrono::steady_clock::now();

  // way 2 累积目标时间戳（基于 nextFrameTime） 全局对齐 基本平齐 fps
  auto nextStart = std::chrono::steady_clock::now();

  while (!quit_flag_run) {
    if (!ready.load(std::memory_order_acquire)) {
      QThread::msleep(300);
      continue;
    }

    // OpenCV VideoCapture 默认输出 BGR，不是 RGB。
    if (!v4l2_device->poll(raw_image)) {
      AERROR << "camera device poll failed";
      QThread::msleep(100);
      continue;
    } else {
      frame = raw_image->image;
    }

    // clang-format off
    if (config_->width() != frame.cols || config_->height() != frame.rows) {
      cv::resize(frame, frame_resized, cv::Size(config_->width(), config_->height()));
    } else {
      frame_resized = frame;
    }
    // clang-format on

    // 全部交给 encode 线程，还是 run 也负责一部分？
    // 推入编码队列
    if (isRecording()) {
      if (!queue_->push(frame_resized.clone())) {
        session_state_.CountDroppedFrame();
      }
      // qDebug() << "queue size:" << queue_->size();
    }

    // Qt 显示，因为 QImage::Format_RGB888 需要 RGB
    cv::cvtColor(frame_resized, frame_rgb, cv::COLOR_BGR2RGB);

    matToQImage(frame_rgb, img);

    // 保存最新一帧
    {
      QMutexLocker locker(&frameMutex);
      lastFrameBgr = frame_resized;  // BGR
      lastFrameRgb = frame_rgb;  // RGB
      lastImg      = img;  // QImage
    }

    // encode();
    frameCount++;

    // 当正在录制时，线程睡眠的时间 = 每帧间隔 = 1000 / fps 毫秒（fps 是帧率）
    // 例如 fps = 30 → 1000 / 30 ≈ 33 ms，也就是每 33 毫秒处理一帧
    // 当没有录制时，线程睡眠 30 ms（休眠更长一点，减少 CPU 占用）
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

    // /* Debug
    auto now = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration<double>(now - lastTime).count();  // 精确到小数秒
    if (elapsed >= 1.0) {
      // qDebug() << "实际采集帧率:" << frameCount / elapsed << "Hz";
      frameCount = 0;
      lastTime   = now;
    }
    // */
  }
}

QString CameraDevice::generateVideoFileRoot() {
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

  return fullDirPath;
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

void CameraDevice::encode() {
  cv::Mat frame;
  auto failRecording = [this](int errorCode, const QString& message) {
    qWarning() << "[CameraDevice] FFmpeg recording failed:" << message;
    session_state_.WriterFailed();
    recording = RecordState::Error;
    quit_flag_encode = true;
    queue_->stop();
    if (writer) writer->abort();
    emit recordError(devKey, errorCode);
    emit recorderState(devKey, RecordState::Error);
  };

  while (!quit_flag_encode) {
    if (!isRecording()) break;
    if (!queue_->waitPop(frame)) break;

    const int ret = writer ? writer->write(frame) : -1;
    recordElapsedSeconds = recordTimer.elapsed() / 1000;
    if (ret != 0) {
      const QString message = writer
                                  ? QString::fromStdString(writer->lastError())
                                  : QStringLiteral("FFmpeg writer is unavailable");
      failRecording(ret < 0 ? 1 : ret, message);
      break;
    }
    encoded_frame_count_.fetch_add(1, std::memory_order_relaxed);

    try {
      if (writer && writer->rotateIfNeeded()) {
        recordTimer.restart();
        recordElapsedSeconds = 0;
      }
    } catch (const std::exception& e) {
      failRecording(4, QString::fromLocal8Bit(e.what()));
      break;
    }
  }
}
