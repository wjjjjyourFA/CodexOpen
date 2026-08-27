#ifndef CAMERA_DEVICE_H
#define CAMERA_DEVICE_H

#pragma once

#include <QDateTime>
#include <QElapsedTimer>
#include <QImage>
#include <QLabel>
#include <QMutex>
#include <QObject>
#include <QSlider>
#include <QString>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <atomic>
#include <climits>
#include <cstdint>

#include <opencv2/opencv.hpp>

#include "tools/capture_record/common/common.h"
#include "tools/capture_record/core/capture_state.h"

/**
 * CameraDevice
 * 负责摄像头操作：
 * 1. 打开/关闭摄像头
 * 2. 捕获图像帧
 * 3. 视频录制（OpenCV VideoWriter）
 * 仅负责数据
 */
class CameraDevice : public QObject {
  Q_OBJECT
 public:
  explicit CameraDevice(const QString& device,
                        QWidget* viewfinderParent = nullptr,
                        QWidget* parent           = nullptr);
  ~CameraDevice();

  // 返回一个能嵌入 UI 的窗口
  QLabel* viewQLabel() const { return this->viewLabel; };
  void showImageOnLabel(QImage& qImg);

  void setSavePath(const QString& path);
  void setExposureCompensation(int index);
  void initExposureSlider(QSlider* slider);

  void open();  // 启动相机
  void close();  // 停止相机

  // 数据捕获线程
  void run();

  // 响应 UI 控制指令
  void startRecording();
  void pauseRecording();
  void stopRecording();

  void startAutoRecording();

  bool isRecording() const {
    const RecordState state = recording.load(std::memory_order_acquire);
    return state == RecordState::Recording ||
           state == RecordState::AutoRecording;
  }

  qint64 recordDuration() const;

  void setCaptureMode(int mode);
  void takeImage();

  bool isCapturing() const { return capturing.load(std::memory_order_acquire); }

  bool isReadyForCapture() const {
    return ready.load(std::memory_order_acquire) && isCapturing();
  }

  void updateCustomSettings(const CustomSettings& setings);

  // 返回设备标识
  QString deviceKey() const { return devKey; }
  bool cameraStatus() const { return ready.load(std::memory_order_acquire); }
  RecordState recordStatus() const {
    return recording.load(std::memory_order_acquire);
  }
  std::uint64_t encodedFrameCount() const {
    return encoded_frame_count_.load(std::memory_order_relaxed);
  }

 signals:
  void cameraState(const QString& devKey, bool state);
  void frameReady(const QImage& frame);

  void readyCapture(const QString& devKey, bool ready);
  void previewAvailable(const QString& devKey, const QImage& frame);
  // void previewAvailable(const QString &devKey, std::shared_ptr<QImage> img);
  void saveImageSuc(const QString& devKey, const QString& filePath);

  void recorderState(const QString& devKey, const RecordState& state);
  void recordTime(const QString& devKey, qint64 time);
  void recordError(const QString& devKey, int errorCode);

 protected:  // 原来被 camera 触发的事件，现在改为手动触发
  void readyForCapture(bool ready);
  void processCapturedImage(const QImage& img, const cv::Mat& frame);

  void updateRecordTime();

 protected:
  void applyExposureCompensation();

  void matToQImage(const cv::Mat& frame, QImage& img);
  void matToQImage(const cv::Mat& frame);
  void resizeEvent(QResizeEvent* event);
  bool openNewWriter();

 private:
  QLabel* viewLabel = nullptr;
  // 使用opencv进行视频录制
  cv::VideoCapture cap;
  cv::VideoWriter writer;
  std::atomic<bool> ready{false};
  std::atomic<bool> capturing{false};
  std::atomic<RecordState> recording{RecordState::Stopped};
  jojo::tools::capture_record::CaptureSessionState session_state_;
  std::atomic<std::uint64_t> encoded_frame_count_{0};
  std::atomic<bool> previewUpdatePending{false};

  // 子容器，用于嵌入 UI
  QWidget* containerWidget;
  QVBoxLayout* containerLayout;

  QString devKey;
  QString savePath;

  QString generateVideoFilePath();
  QString generateImageFilePath();

  QTimer exposureDebounceTimer;
  int pendingExposureIndex = 0;

  // OpenCV 采集线程
  std::atomic<bool> quit_flag{false};
  QThread* workerThread = nullptr;
  QMutex frameMutex;
  QMutex writerMutex;
  cv::Mat lastFrameBgr, lastFrameRgb;
  QImage lastImg;  // for preview

  // 相机固定设置
  int device_width;
  int device_height;

  // 相机custom设置等
  std::atomic<float> fps{10.0F};  // default
  std::atomic<int> width_{1920};
  std::atomic<int> height_{1080};

  // 自动录制切分功能
  // int segmentMilliseconds = 10 * 1000;
  int segmentMilliseconds = 3600 * 1000;  // 默认每段录像 1 小时
  // segmentSeconds 是以秒为单位，而 QElapsedTimer::elapsed() 或 QTimer 的时间参数是 毫秒（ms）。
  // 记录录像开始时间 并 用来判断是否该切分
  QElapsedTimer recordTimer;
  std::atomic<qint64> recordElapsedSeconds{0};
  bool first_record_loop = false;
};

#endif
