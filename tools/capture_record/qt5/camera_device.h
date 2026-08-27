#ifndef CAMERA_DEVICE_H
#define CAMERA_DEVICE_H

#pragma once

#include <QCamera>
#include <QCameraImageCapture>
#include <QCameraInfo>
#include <QCameraViewfinder>
#include <QDateTime>
#include <QMediaRecorder>
#include <QSlider>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

class CameraDevice : public QObject {
  Q_OBJECT
 public:
  explicit CameraDevice(const QCameraInfo& info,
                        QWidget* viewfinderParent = nullptr,
                        QWidget* parent           = nullptr);
  ~CameraDevice();

  // 返回一个能嵌入 UI 的窗口
  QWidget* viewfinderWidget() const { return this->viewfinder; };

  void setSavePath(const QString& path);
  void setExposureCompensation(int index);
  void initExposureSlider(QSlider* slider);

  void start();  // 启动相机
  void stop();  // 停止相机

  void startRecording();
  void pauseRecording();
  void stopRecording();

  bool isRecording() const {
    return this->recorder->state() == QMediaRecorder::RecordingState;
  }

  qint64 recordDuration() const { return this->recorder->duration() / 1000; }

  void setCaptureMode(QCamera::CaptureModes mode);
  void takeImage();

  bool isCapturing() const {
    return this->camera->captureMode() == QCamera::CaptureStillImage;
  }

  bool isReadyForCapture() const {
    if (!capturer) return false;
    return capturer->isReadyForCapture();  // QCameraImageCapture 提供此方法
  }

  void setMuted(bool muted);

  void searchAndLock();
  void unlock();

  // 返回设备标识
  QString deviceKey() const { return devKey; }
  QCamera::State cameraStatus() const { return camera->state(); }
  QCamera::LockStatus lockStatus() const { return camera->lockStatus(); }
  QMediaRecorder::State recorderStatus() const { return recorder->state(); }
  // 判断是否支持拍照
  bool isStillImageCaptureSupported() const {
    return camera->isCaptureModeSupported(QCamera::CaptureStillImage);
  }
  // 判断是否支持录像
  bool isVideoCaptureSupported() const {
    return camera->isCaptureModeSupported(QCamera::CaptureVideo);
  }
  QCamera::CaptureModes captureMode() const { return camera->captureMode(); }

  QMediaRecorder* mediaRecorder() const { return recorder; }
  QCameraImageCapture* imageCapturer() const { return capturer; }

 signals:
  void cameraState(const QString& devKey, const QCamera::State& state);
  void cameraError(const QString& devKey, const QString& errStr);
  void cameraLockState(const QString& devKey, const QCamera::LockStatus& status,
                       QCamera::LockChangeReason reason);

  void readyCapture(const QString& devKey, bool ready);
  void previewAvailable(const QString& devKey, const QImage& img);
  void saveImageSuc(const QString& devKey, const QString& filePath);
  void captureError(const QString& devKey, const QString& errStr);

  void recorderState(const QString& devKey, const QMediaRecorder::State& state);
  void recordTime(const QString& devKey, qint64 time);
  void recorderError(const QString& devKey, const QString& errStr);

 private slots:
  void updateCameraState(const QCamera::State& state);
  void displayCameraError();
  void updateLockStatus(const QCamera::LockStatus& status,
                        const QCamera::LockChangeReason& reason);

  void readyForCapture(bool ready);
  void processCapturedImage(int request_id, const QImage& img);
  // void saveCapturedImage(int id, const QString &fileName);
  void displayCaptureError(int id, const QCameraImageCapture::Error error,
                           const QString& errorString);

  void updateRecorderState(const QMediaRecorder::State& state);
  void updateRecordTime();
  void displayRecorderError();

 protected:
  void applyExposureCompensation();

 private:
  QCameraInfo cameraInfo;
  QCamera* camera;
  QCameraImageCapture* capturer = nullptr;
  QMediaRecorder* recorder      = nullptr;
  QCameraViewfinder* viewfinder = nullptr;

  // 子容器，用于嵌入 UI
  QWidget* containerWidget;
  QVBoxLayout* containerLayout;

  QString devKey;
  QString savePath;

  QString generateVideoFilePath();
  QString generateImageFilePath();

  QTimer exposureDebounceTimer;
  int pendingExposureIndex = 0;
};

#endif
