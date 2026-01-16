#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <QWidget>
#include <QComboBox>
#include <QGridLayout>
#include <QLabel>

#include "tools/mmpose_inference_gui/videoplayerwidget.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWidget;
}
QT_END_NAMESPACE

class MainWidget : public QWidget {
  Q_OBJECT

 public:
  explicit MainWidget(QWidget* parent = nullptr);
  ~MainWidget();

 private slots:
  void onVideoCountChanged(int index);
  void onModelChanged(const QString& model);

 private:
  void initVideoArea();
  void initCtrlArea();

  void loadModeConfig();

  void loadPose2dConfig();
  void savePose2dToConfigMMpose(int index);

  void loadDetectionConfig();
  void saveDetectionToConfigMMpose(int index);

 private:
  Ui::MainWidget* ui;

  // 左侧
  QGridLayout* videoGridLayout_;
  QList<VideoPlayerWidget*> videoWidgets_;

  // 右侧
  QComboBox* comboVideoCount_;
  QComboBox* comboModel_;

  // detection 配置
  QComboBox* comboDetConfig_;
  QComboBox* comboDetWeights_;

  // pose2d 配置
  QComboBox* comboPoseConfig_;
  QComboBox* comboPoseWeights_;

  // 内部缓存（index → 路径）
  QMap<int, QString> detConfigMap_;
  QMap<int, QString> detWeightsMap_;
  QMap<int, QString> poseConfigMap_;
  QMap<int, QString> poseWeightsMap_;

  QString config_path = "./../../config/MmposeInferenceGui";
};

#endif  // WIDGET_H
