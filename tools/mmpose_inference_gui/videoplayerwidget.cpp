#include "tools/mmpose_inference_gui/videoplayerwidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QProcess>
#include <QUrl>
#include <QDebug>

VideoPlayerWidget::VideoPlayerWidget(const QString& title, QWidget* parent)
    : QWidget(parent), title_(title) {
  setWindowTitle(title_);
  resize(800, 600);

  player_      = new QMediaPlayer(this);
  videoWidget_ = new QVideoWidget(this);
  player_->setVideoOutput(videoWidget_);

  initUI();
  initConnections();
  /* ---------- 默认 RTSP ---------- */
  int index           = title_.right(1).toInt();  // video1 → 1
  QString defaultRtsp = QString("rtsp://127.0.0.1:8554/fszn_%1").arg(index);

  comboRtsp_->addItem(defaultRtsp);
  comboRtsp_->setCurrentText(defaultRtsp);

  //rtsp 设置
  connect(player_, SIGNAL(mediaStatusChanged(QMediaPlayer::MediaStatus)),this, SLOT(onMediaStatusChanged(QMediaPlayer::MediaStatus)));
  connect(player_, SIGNAL(stateChanged(QMediaPlayer::State)),this, SLOT(onStateChanged(QMediaPlayer::State)));
  connect(player_, SIGNAL(error(QMediaPlayer::Error)),this, SLOT(onError(QMediaPlayer::Error)));
  retryTimer_ = new QTimer(this);
  retryTimer_->setInterval(3000);
  retryTimer_->setSingleShot(true);
  connect(retryTimer_, &QTimer::timeout,
          this, &VideoPlayerWidget::onRetryTimeout);

}

void VideoPlayerWidget::initUI() {
  /* ---------- 顶部按钮 ---------- */
  btnOpenFile_  = new QPushButton("Open File", this);
  btnRunScript_ = new QPushButton("Run Script", this);

  QHBoxLayout* topLayout = new QHBoxLayout;
  topLayout->addWidget(btnOpenFile_);
  topLayout->addWidget(btnRunScript_);
  topLayout->addStretch();

  /* ---------- 底部控制区 ---------- */
  comboRtsp_ = new QComboBox(this);
  comboRtsp_->setEditable(true);
  comboRtsp_->setMinimumWidth(350);

  // 预置 RTSP 地址
  // comboRtsp_->addItems({"rtsp://192.168.1.100:8554/live"});

  btnPlay_  = new QPushButton("Play", this);
  btnPause_ = new QPushButton("Pause", this);

  QHBoxLayout* bottomLayout = new QHBoxLayout;
  bottomLayout->addWidget(comboRtsp_);
  bottomLayout->addWidget(btnPlay_);
  bottomLayout->addWidget(btnPause_);

  /* ---------- 主布局 ---------- */
  QVBoxLayout* mainLayout = new QVBoxLayout(this);
  mainLayout->addLayout(topLayout);
  mainLayout->addWidget(videoWidget_, 1);
  mainLayout->addLayout(bottomLayout);

  setLayout(mainLayout);
  btnPlay_->setFixedWidth(100);
  btnPause_->hide();
}

void VideoPlayerWidget::initConnections() {
  // clang-format off
  connect(btnOpenFile_, &QPushButton::clicked, this, &VideoPlayerWidget::onOpenFile);
  connect(btnRunScript_, &QPushButton::clicked, this, &VideoPlayerWidget::onRunScript);
  connect(btnPlay_, &QPushButton::clicked, this, &VideoPlayerWidget::onPlay);
  connect(btnPause_, &QPushButton::clicked, this, &VideoPlayerWidget::onPause);
  connect(player_, &QMediaPlayer::positionChanged, this, &VideoPlayerWidget::onPositionChanged);
  // clang-format on
}

/* ================= 槽函数 ================= */

void VideoPlayerWidget::onOpenFile() {
  QString file =
      QFileDialog::getOpenFileName(this, "Open Video File", QDir::homePath(),
                                   "Video Files (*.mp4 *.avi *.mkv)");

  if (file.isEmpty()) return;

  player_->setMedia(QUrl::fromLocalFile(file));
  // 标记：我们只是为了预览第一帧
  preview_pending_ = true;
  // 触发解码管线
  player_->play();
  int index = title_.right(1).toInt();
  QString ini_path = config_path+QString("/mmpose_%1.ini").arg(index);
  QSettings settings(ini_path, QSettings::IniFormat);
  settings.beginGroup("video");

  // settings.setValue(QString("video_path_%1").arg(index), file);
  settings.setValue("video_path", file);

  settings.endGroup();
}

void VideoPlayerWidget::onRunScript() {
  emit sigRunScript();

  int index = title_.right(1).toInt();
  // video1 → 1
  QString defaultRtsp = QString("rtsp://127.0.0.1:8554/fszn_%1").arg(index);
  QString ini_path = config_path+QString("/mmpose_%1.ini").arg(index);
  QSettings settings(ini_path, QSettings::IniFormat);
  settings.beginGroup("video");

  // settings.setValue(QString("rtsp_%1").arg(index), defaultRtsp);
  settings.setValue("rtsp_url", defaultRtsp);

  settings.endGroup();

  // 示例：直接调用脚本
  QString script = config_path + "/start_inference.sh";

  QProcess::startDetached("/bin/bash", QStringList() << script << title_);

  // 延迟设置默认视角
  QTimer::singleShot(3000, this, [this,defaultRtsp]() {
      startPull(defaultRtsp);
  });
}

void VideoPlayerWidget::onPlay() {
  QString url = comboRtsp_->currentText();
  if (url.isEmpty()) return;

  if (url.startsWith("rtsp://"))
    player_->setMedia(QUrl(url));
  else
    player_->setMedia(QUrl::fromLocalFile(url));

  player_->play();
}

void VideoPlayerWidget::onPositionChanged(qint64 pos) {
  if (!preview_pending_) return;

  if (pos > 0) {
    preview_pending_ = false;

    player_->pause();
    // 回到第一帧
    player_->setPosition(0);
    // 此时 QVideoWidget 上显示的就是第一帧
  }
}

void VideoPlayerWidget::onPause() { player_->pause(); }

void VideoPlayerWidget::startPull(const QString& url)
{
    rtspUrl_   = url;
    retryCount_ = 0;
    pullOk_     = false;

    tryPull();
}
void VideoPlayerWidget::tryPull()
{
    retryCount_++;

    qDebug() << "Try pull RTSP:" << retryCount_;

    pullOk_ = false;

    player_->stop();
    player_->setMedia(QUrl(rtspUrl_));
    player_->play();

    // 3 秒内如果没成功，就进入下一次
    retryTimer_->start();
}
void VideoPlayerWidget::onMediaStatusChanged(QMediaPlayer::MediaStatus status)
{
    if (status == QMediaPlayer::LoadedMedia ||
        status == QMediaPlayer::BufferedMedia)
    {
        onPullSuccess();
    }
}
void VideoPlayerWidget::onStateChanged(QMediaPlayer::State state)
{
    if (state == QMediaPlayer::PlayingState) {
        onPullSuccess();
    }
}

void VideoPlayerWidget::onError(QMediaPlayer::Error error)
{
    if (error == QMediaPlayer::NoError)
        return;

    qWarning() << "RTSP error:" << player_->errorString();

    pullOk_ = false;
}

void VideoPlayerWidget::onRetryTimeout()
{
    if (pullOk_)
        return;

    if (retryCount_ < maxRetry_) {
        tryPull();
    } else {
        onPullFailed();
    }
}
void VideoPlayerWidget::onPullSuccess()
{
    if (pullOk_)
        return;

    pullOk_ = true;
    retryTimer_->stop();

    qDebug() << "RTSP pull success";
}

void VideoPlayerWidget::onPullFailed()
{
    retryTimer_->stop();
    player_->stop();

    qWarning() << "RTSP pull failed after 3 retries";

    QMessageBox::warning(
        this,
        tr("视频连接失败"),
        tr("无法连接视频流地址：\n%1\n\n请检查推流服务是否启动。")
            .arg(rtspUrl_)
    );
}


