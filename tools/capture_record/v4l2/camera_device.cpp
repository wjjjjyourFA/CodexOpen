#include "tools/capture_record/v4l2/camera_device.h"

// #include <QMediaCaptureSession>
#include <QUrl>
#include <QDir>
#include <QDebug>
#include <QSizePolicy>
#include <QMediaMetaData>

using namespace apollo::drivers::camera::config;

CameraDevice::CameraDevice(const QString& device, const CustomSettings& setings,
                           QWidget* viewfinderParent, QWidget* parent)
    : QObject(parent) {
  devKey = device;
  // qDebug() << device;

  // 原来写在参数文件中，现在写在构造函数里，避免加载参数文件；
  config_ = std::make_shared<Config>();
  config_->set_camera_dev(devKey.toStdString());
  // config_->set_pixel_format("yuyv");
  config_->set_pixel_format("uyvy");
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

  queue_ = std::make_shared<FrameQueue<cv::Mat>>();

  // qDebug() << "[CameraDevice] CameraDevice created for device:" << devKey;
  // clang-format on
}

CameraDevice::~CameraDevice() {
  close();
  if (writerThread) {
    writerThread->quit();
    writerThread->wait();
  }
  if (workerThread) {
    workerThread->quit();
    workerThread->wait();
  }
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
  if (ready) return;

  v4l2_device = std::make_shared<UsbCamCv>();
  if (!v4l2_device->init(config_)) {
    qWarning() << "[CameraDevice] Failed to reset device:" << devKey;
    ready = false;
    emit cameraState(devKey, ready);
    return;
  }
  // v4l2_device->DebugInfo();
  // qDebug() << "[CameraDevice] open() Config fps:" << config_->frame_rate();

  // ==> v4l2_device->start_capturing();
  if (!v4l2_device->wait_for_device()) {
    qWarning() << "[CameraDevice] Failed to open device:" << devKey;
    ready = false;
    emit cameraState(devKey, ready);
    return;
  }

  ready = true;

  // 默认模式
  recording = RecordState::Stopped;

  // OpenCV 采集线程  ==> camera->start();
  quit_flag_run = false;
  workerThread  = QThread::create([this] {
    // run_single();
    run_multi();
  });
  workerThread->start();

  emit cameraState(devKey, ready);

  // qDebug() << "[CameraDevice] Camera started:" << devKey;
}

void CameraDevice::close() {
  if (isRecording()) {
    stopRecording();
  }

  // 关闭 采集线程; 会在下一次循环开始时停止
  quit_flag_run = true;
  if (workerThread) {
    workerThread->quit();
    workerThread->wait();
  }

  // ==> camera->stop();
  if (v4l2_device->is_capturing()) {
    ready     = false;
    capturing = false;
    recording = RecordState::Stopped;

    v4l2_device->ReleaseDevice();

    emit cameraState(devKey, ready);

    // qDebug() << "[CameraDevice] Camera stopped:" << devKey;
  }
}

void CameraDevice::startRecording() {
  if (!ready) {
    if (!v4l2_device->is_capturing()) {
      // 这里要退出，因为设备并没有启动
      return;
    }
    ready = true;
    emit cameraState(devKey, ready);  // 重新发送状态
  }

  // setCaptureMode(2) ==> 设置为录制模式
  if (!isRecording()) return;

  // encode
  quit_flag_encode = false;
  writerThread     = QThread::create([this] { encode(); });
  writerThread->start();

  recording = RecordState::Recording;
  capturing = false;

  emit recorderState(devKey, recording);

  // qDebug() << "[CameraDevice] Recording started:" << devKey
  //          << segmentMilliseconds;
}

void CameraDevice::startAutoRecording() {
  if (!ready) {
    if (!v4l2_device->is_capturing()) return;
    ready = true;
    emit cameraState(devKey, ready);
  }

  if (!isRecording()) return;

  // encode
  quit_flag_encode = false;
  writerThread     = QThread::create([this] { encode(); });
  writerThread->start();

  recording = RecordState::AutoRecording;
  capturing = false;

  emit recorderState(devKey, recording);

  // qDebug() << "[CameraDevice] AutoRecording started:" << devKey;
}

void CameraDevice::openNewWriter() {
  /* // 先关闭旧的 writer
  if (writer.isOpened()) {
    writer.release();
  }
  */

  // 具体的文件
  // QString filePath = generateVideoFilePath("mp4");
  QString filePath = generateVideoFilePath("avi");

  // way 2
  // VideoWriter::open(...) 里传的 cv::Size(width, height) 是写入视频的帧大小。
  /* 使用 MP4 H264 (需要系统支持)
  writer.open(filePath.toStdString(), cv::CAP_FFMPEG,
              cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
              config_->frame_rate(),
              cv::Size(config_->width(), config_->height()), true);
  */
  // /* MJPG
  // 单帧独立压缩，对于视频来说，压缩效率低，但兼容性好
  writer.open(filePath.toStdString(), cv::CAP_FFMPEG,
              cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
              config_->frame_rate(),
              cv::Size(config_->width(), config_->height()), true);
  // */
  /* XVID
  writer.open(filePath.toStdString(), cv::CAP_FFMPEG,
              cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),
              config_->frame_rate(),
              cv::Size(config_->width(), config_->height()), true);
  */

  if (!writer.isOpened()) {
    qWarning() << "[CameraDevice] Failed to open VideoWriter";
    return;
  }

  // qDebug() << "[CameraDevice] openNewWriter:" << filePath;
  // qDebug() << "[CameraDevice] openNewWriter: fps: " << config_->frame_rate();
}

void CameraDevice::pauseRecording() {
  // this->updateRecordTime();
  this->stopRecording();
}

void CameraDevice::stopRecording() {
  if (isRecording()) {
    this->updateRecordTime();

    // 关闭 编码线程; 会在下一次循环开始时停止
    quit_flag_encode = true;
    if (writerThread) {
      writerThread->quit();
      writerThread->wait();

      // 由主线程统一关闭
      if (writer.isOpened()) {
        writer.release();
      }
    }

    // way 2
    recording         = RecordState::Stopped;
    first_record_loop = false;

    /* // 单线程
    if (writer.isOpened()) {
      writer.release();
    } 
    */

    emit recorderState(devKey, recording);

    // qDebug() << "[CameraDevice] Recording stopped";
  }
}

void CameraDevice::updateCustomSettings(const CustomSettings& setings) {
  stopRecording();

  savePath = setings.filePath;
  // 无法修改 fps，和 分辨率
  // config_->set_frame_rate(setings.fps);
  // config_->set_width(setings.width);
  // config_->set_height(setings.height);
  // 自动切分时间间隔
  segmentMilliseconds = setings.segmentSeconds * 1000;

  // qDebug() << "[CameraDevice] updateCustomSettings path:" << setings.filePath;
  // qDebug() << "[CameraDevice] updateCustomSettings sseconds:"
  //          << setings.segmentSeconds;
}

void CameraDevice::setCaptureMode(int mode) {
  if (mode == 1) {
    capturing = true;
    recording = RecordState::Stopped;
  } else if (mode == 2) {
    capturing = false;
    recording = RecordState::Recording;
  } else {
    capturing = false;
    recording = RecordState::Stopped;
  }

  this->readyForCapture(ready && capturing);
  emit recorderState(devKey, recording);

  // qDebug() << "[CameraDevice] Setting capture mode to:" << mode;
}

void CameraDevice::takeImage() {
  if (!v4l2_device->is_capturing()) return;

  if (!capturing) return;

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

// /* // 单线程 采集循环
void CameraDevice::run_single() {
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
    if (!ready) {
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

    // 计时器的初始化；写入视频
    if (this->isRecording() && writer.isOpened()) {
      if (first_record_loop == true) {
        // segmentFrames = static_cast<int>(std::round(fps * segmentSeconds));
        frameIntervalMs = 1000.0 / config_->frame_rate();
        recordTimer.restart();
        first_record_loop = false;
      }

      if (recording == RecordState::AutoRecording) {
        // 检查是否需要切分文件
        if (recordTimer.isValid() &&
            recordTimer.elapsed() >= segmentMilliseconds + 1) {
          // qDebug() << "实际切分帧数:" << frameCount;
          // 结束上一段数据录制，重新启动 视频写入器
          writer.release();

          this->openNewWriter();

          // 重新启动定时器
          recordTimer.restart();
          frameCount = 0;
        }
      }

      // 写入当前帧
      // auto writeStart = std::chrono::steady_clock::now();
      writer.write(frame_resized);
      frameCount++;
      // auto writeEnd = std::chrono::steady_clock::now();

      // 可选：打印写入耗时（ms）与当前段的帧数，用来诊断性能瓶颈
      // qDebug() << "[CameraDevice] write_ms:"
      //          << std::chrono::duration_cast<std::chrono::milliseconds>(
      //                 writeEnd - writeStart)
      //                 .count()
      //          << "frameCount:" << frameCount;
    }

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

    // Debug
    auto now = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration<double>(now - lastTime).count();  // 精确到小数秒
    if (elapsed >= 1.0) {
      // qDebug() << "实际采集帧率:" << frameCount / elapsed << "Hz";
      frameCount = 0;
      lastTime   = now;
    }
  }
}
// */

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
    if (!ready) {
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
      queue_->push(frame_resized.clone());
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

QString CameraDevice::generateVideoFilePath(const std::string& suffix) {
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
  QString fileName =
      QString("%1.%2").arg(timestamp).arg(QString::fromStdString(suffix));
  QString filePath = fullDirPath + "/" + fileName;

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
  if (!qImg.isNull() && viewLabel) {
    viewLabel->setPixmap(QPixmap::fromImage(qImg).scaled(
        viewLabel->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  }
}

qint64 CameraDevice::recordDuration() const {
  if (!isRecording()) return 0;
  // recordTimer.elapsed() 本身是线程安全的，不依赖外部数据
  // QMutexLocker locker(&frameMutex);
  qint64 time = recordTimer.elapsed() / 1000;
  return time;
}

void CameraDevice::encode() {
  cv::Mat frame;

  double frameIntervalMs = 1000.0 / config_->frame_rate();

  int frameCount = 0;
  auto lastTime  = std::chrono::steady_clock::now();

  this->openNewWriter();
  first_record_loop = true;

  while (!quit_flag_encode) {
    if (!this->isRecording()) {
      QThread::msleep(300);
      continue;
    }

    // 计时器的初始化；写入视频
    if (writer.isOpened()) {
      if (first_record_loop == true) {
        // segmentFrames = static_cast<int>(std::round(fps * segmentSeconds));
        frameIntervalMs = 1000.0 / config_->frame_rate();
        recordTimer.restart();
        first_record_loop = false;
      }

      if (recording == RecordState::AutoRecording) {
        // 检查是否需要切分文件
        if (recordTimer.isValid() &&
            recordTimer.elapsed() >= segmentMilliseconds + 1) {
          // qDebug() << "实际切分帧数:" << frameCount;
          // 结束上一段数据录制，重新启动 视频写入器
          writer.release();

          this->openNewWriter();

          // 重新启动定时器
          recordTimer.restart();
          frameCount = 0;
        }
      }

      // 写入当前帧
      if (queue_->pop(frame)) {
        // auto writeStart = std::chrono::steady_clock::now();
        writer.write(frame);
        frameCount++;
        // auto writeEnd = std::chrono::steady_clock::now();
      } else {
        QThread::msleep(5);  // 队列空，避免忙等
        continue;
      }

      // 可选：打印写入耗时（ms）与当前段的帧数，用来诊断性能瓶颈
      // qDebug() << "[CameraDevice] write_ms:"
      //          << std::chrono::duration_cast<std::chrono::milliseconds>(
      //                 writeEnd - writeStart)
      //                 .count()
      //          << "frameCount:" << frameCount;
    } else {
      QThread::msleep(200);
      continue;
    }

    /* Debug
    auto now = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration<double>(now - lastTime).count();  // 精确到小数秒
    if (elapsed >= 1.0) {
      // qDebug() << "实际编码帧率:" << frameCount / elapsed << "Hz";
      frameCount = 0;
      lastTime   = now;
    }
    */
  }

  /* 子线程自己关闭 writer
  if (writer.isOpened()) {
    writer.release();
  }
  */
}
