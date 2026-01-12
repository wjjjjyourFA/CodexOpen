#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCamera>
#include <QCameraInfo>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QMediaRecorder>
#include <QMap>
#include <QGridLayout>

#include "tools/capture_record/qt5/camera_device.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);
  ~MainWindow();

 private slots:
  void rebuildDeviceMenu();  // 扫描设备，更新菜单
  void updateCaptureMode(int index);
  void setExposureCompensation(int value);

  void startAllCamera();
  void stopAllCamera();

  void takeAllCapture();

  void startAllRecording();
  void pauseAllRecording();
  void stopAllRecording();

  void setAllMuted(bool muted = true);

  void updateCameraState(const QString& devKey, const QCamera::State& state);
  void displayCameraError(const QString& devKey, const QString& errorString);
  void updateLockStatus(const QString& devKey,
                        const QCamera::LockStatus& status,
                        const QCamera::LockChangeReason& reason);

  void readyForCapture(const QString& devKey, bool ready);
  void processCapturedImage(const QString& devKey, const QImage& img);
  void saveCapturedImage(const QString& devKey, const QString& filePath);
  void displayCaptureError(const QString& devKey, const QString& errorString);

  void updateRecorderState(const QString& devKey,
                           const QMediaRecorder::State& state);

  void displayRecorderError(const QString& devKey, const QString& errorString);

  void configureCameraSettings();
  void configureVideoSettings();
  void configureImageSettings();

  void toggleLock();
  void displayViewfinder();
  void displayCapturedImage();

 protected:
  void updateRecordTime();
  void refreshCameraControls();
  void refreshRecorderControls();

  void init_device_service();
  bool update_ui_tab(int mode);

  void keyPressEvent(QKeyEvent* event);
  void keyReleaseEvent(QKeyEvent* event);
  void closeEvent(QCloseEvent* event);

 private:
  Ui::MainWindow* ui;

  QGridLayout* viewLayout;  // 对应 designer 中 gridLayout_5

  QMap<QString, QCameraInfo> discoveredCameras;  // 系统检测到的摄像头
  QMap<QString, QAction*> deviceActions;  // 菜单 action
  QMap<QString, CameraDevice*> activeCameras;  // 当前勾选的摄像头实例

  const QList<int> fixedDevices = {100, 101, 102, 103};
  // /dev/video100 -> video100
  QString normalizeDevName(const QString& devName);

  void startDefualtCamera();
  void startCamera(const QString& devKey, int mode = 1);
  void stopCamera(const QString& devKey, int mode = 1);
  void rearrangeViewfinders();

  QImageEncoderSettings imageSettings;
  QAudioEncoderSettings audioSettings;
  QVideoEncoderSettings videoSettings;
  QString videoContainerFormat;

  QString filePath;
  bool isCapturingImage   = false;
  bool applicationExiting = false;

  QMap<QString, QCamera::State> cameraStates;
  QMap<QString, QMediaRecorder::State> recorderStates;
  // 预留给多相机画面 拍摄预览
  // QMap<QString, QLabel*> cameraPreviews;
};

#endif
