#ifndef VIDEOPLAYERWIDGET_H
#define VIDEOPLAYERWIDGET_H

#include <QWidget>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QComboBox>
#include <QPushButton>
#include <QSettings>
#include <QTimer>
#include <QMessageBox>

class VideoPlayerWidget : public QWidget {
  Q_OBJECT
 public:
  explicit VideoPlayerWidget(const QString& title, QWidget* parent = nullptr);

 signals:
  void sigRunScript();  // 运行脚本的回调信号

protected slots:
  void onMediaStatusChanged(QMediaPlayer::MediaStatus status);
  void onStateChanged(QMediaPlayer::State state);
  void onError(QMediaPlayer::Error error);
  void onRetryTimeout();
private slots:
  void onOpenFile();
  void onRunScript();
  void onPlay();
  void onPause();
  void onPositionChanged(qint64 pos);

 private:
  void initUI();
  void initConnections();

  bool preview_pending_ = false;

 private:
  QString title_;

  QMediaPlayer* player_;
  QVideoWidget* videoWidget_;

  QPushButton* btnOpenFile_;
  QPushButton* btnRunScript_;

  QComboBox* comboRtsp_;
  QPushButton* btnPlay_;
  QPushButton* btnPause_;

  QString config_path = "./../../config/MmposeInferenceGui";

  // rtsp
  QTimer*       retryTimer_;
  int           retryCount_ = 0;
  const int     maxRetry_   = 3;
  bool          pullOk_     = false;
  QString       rtspUrl_;

  void startPull(const QString &url);
  void tryPull();
  void onPullSuccess();
  void onPullFailed();
};

#endif  // VIDEOPLAYERWIDGET_H
