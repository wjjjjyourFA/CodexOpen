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

#include "tools/capture_record/v4l2/camera_device.h"

#include "tools/capture_record/common/utils.h"

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
  void startAllAutoRecording();
  void pauseAllRecording();
  void stopAllRecording(int mode = 1);
  void stopAllAutoRecording();

  void setAllMuted(bool muted = true);

  void updateCameraState(const QString& devKey, bool state);

  void readyForCapture(const QString& devKey, bool ready);
  void processCapturedImage(const QString& devKey, const QImage& img);
  // void processCapturedImage(const QString &devKey, std::shared_ptr<QImage> img);
  void saveCapturedImage(const QString& devKey, const QString& filePath);

  void updateRecorderState(const QString& devKey, const RecordState& state);
  void updateRecordTime(const QString& devKey, const qint64& timeSec);

  void configureCameraSettings();
  void configureVideoSettings();
  void configureImageSettings();

  void onActionSetSavePath();
  void onActionSetSavePlan();

  void toggleLock();
  void displayViewfinder();
  void displayCapturedImage();

 protected:
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

  const QList<int> fixedDevices = {0, 100, 101, 102, 103};
  // /dev/video100 -> video100
  QString normalizeDevName(const QString& devName);

  void startDefualtCamera();
  void startCamera(const QString& devKey, int mode = 1);
  void stopCamera(const QString& devKey, int mode = 1);
  void rearrangeViewLabels();

  void updateStatusBar();
  void updateAllCamera();

  QImageEncoderSettings imageSettings;
  QAudioEncoderSettings audioSettings;
  QVideoEncoderSettings videoSettings;
  QString videoContainerFormat;

  CustomSettings customSettings;

  QString filePath, recordPlan;
  bool isCapturingImage   = false;
  bool applicationExiting = false;

  QMap<QString, bool> cameraStates;
  QMap<QString, RecordState> recorderStates;
  // 预留给多相机画面 拍摄预览
  // QMap<QString, QLabel*> cameraPreviews;

  // 每天自动录制功能
  void checkDailyRecordStatus();
  QTimer* dailyRecordTimer = nullptr;
  bool isAutoRecording     = false;
};

#endif
