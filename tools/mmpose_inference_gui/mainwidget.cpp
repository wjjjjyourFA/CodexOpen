#include "tools/mmpose_inference_gui/mainwidget.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSettings>
#include <QDebug>

#include "tools/mmpose_inference_gui/pro/ui/ui_mainwidget.h"

MainWidget::MainWidget(QWidget* parent)
    : QWidget(parent), ui(new Ui::MainWidget) {
  ui->setupUi(this);
  this->setWindowTitle("视频推理平台");

  /* ---------- 主横向布局比例 10:1 ---------- */
  auto* mainLayout = new QHBoxLayout(this);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setSpacing(5);

  QWidget* videoArea = new QWidget(this);
  QWidget* ctrlArea  = new QWidget(this);

  mainLayout->addWidget(videoArea, 10);
  mainLayout->addWidget(ctrlArea, 1);
  setLayout(mainLayout);

  /* ---------- 左侧视频区域 ---------- */
  videoGridLayout_ = new QGridLayout(videoArea);
  videoGridLayout_->setSpacing(5);
  videoArea->setLayout(videoGridLayout_);

  /* ---------- 右侧控制区域 ---------- */
  QVBoxLayout* ctrlLayout = new QVBoxLayout(ctrlArea);
  ctrlLayout->setSpacing(10);
  ctrlLayout->setContentsMargins(10, 20, 10, 20);

  /* ===== 窗口数量 ===== */
  QLabel* labelVideoCount = new QLabel("窗口数量：", ctrlArea);
  comboVideoCount_        = new QComboBox(ctrlArea);
  comboVideoCount_->addItems({"1", "2", "4"});

  ctrlLayout->addWidget(labelVideoCount);
  ctrlLayout->addWidget(comboVideoCount_);

  /* ===== 模型选择 ===== */
  QLabel* labelModel = new QLabel("模型选择：", ctrlArea);
  comboModel_        = new QComboBox(ctrlArea);

  QLabel* labelDetConfig = new QLabel("检测配置：", ctrlArea);
  comboDetConfig_        = new QComboBox(ctrlArea);

  QLabel* labelDetWeights = new QLabel("检测权重文件：", ctrlArea);
  comboDetWeights_        = new QComboBox(ctrlArea);

  QLabel* labelPoseConfig = new QLabel("关键点配置：", ctrlArea);
  comboPoseConfig_        = new QComboBox(ctrlArea);

  QLabel* labelPoseWeights = new QLabel("关键点权重文件：", ctrlArea);
  comboPoseWeights_        = new QComboBox(ctrlArea);

  ctrlLayout->addSpacing(15);
  ctrlLayout->addWidget(labelModel);
  ctrlLayout->addWidget(comboModel_);

  ctrlLayout->addSpacing(15);
  ctrlLayout->addWidget(labelDetConfig);
  ctrlLayout->addWidget(comboDetConfig_);
  ctrlLayout->addSpacing(10);
  ctrlLayout->addWidget(labelDetWeights);
  ctrlLayout->addWidget(comboDetWeights_);

  ctrlLayout->addSpacing(15);
  ctrlLayout->addWidget(labelPoseConfig);
  ctrlLayout->addWidget(comboPoseConfig_);
  ctrlLayout->addSpacing(10);
  ctrlLayout->addWidget(labelPoseWeights);
  ctrlLayout->addWidget(comboPoseWeights_);

  ctrlLayout->addStretch();
  ctrlArea->setLayout(ctrlLayout);

  QFont font;
  font.setBold(true);
  labelVideoCount->setFont(font);
  labelModel->setFont(font);

  /* ---------- 初始化 ---------- */
  initVideoArea();
  initCtrlArea();

  loadModeConfig();
  loadDetectionConfig();
  loadPose2dConfig();

  connect(comboModel_, &QComboBox::currentTextChanged, this,
          &MainWidget::onModelChanged);
  connect(comboDetConfig_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int index) {
            comboDetWeights_->setCurrentIndex(index);
            saveDetectionToConfigMMpose(index);
          });
  connect(comboPoseConfig_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int index) {
            comboPoseWeights_->setCurrentIndex(index);
            savePose2dToConfigMMpose(index);
          });
  
  // default show widget
  comboVideoCount_->setCurrentIndex(0);
}

MainWidget::~MainWidget() { delete ui; }

/* ================= 左侧视频 ================= */

void MainWidget::initVideoArea() {
  // 先创建 4 个 VideoPlayerWidget
  for (int i = 0; i < 4; ++i) {
    auto* player = new VideoPlayerWidget(QString("%1").arg(i + 1), this);

    videoWidgets_.append(player);
  }

  // 默认显示 4 个
  // onVideoCountChanged(2);
}

/* ================= 右侧控制 ================= */

void MainWidget::initCtrlArea() {
  connect(comboVideoCount_, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &MainWidget::onVideoCountChanged);
  // 0 ==> 1 | 1 ==> 2 | 2 ==> 4
  comboVideoCount_->setCurrentIndex(2);
}

void MainWidget::onVideoCountChanged(int index) {
  int count = comboVideoCount_->itemText(index).toInt();

  // 清空布局
  QLayoutItem* item;
  while ((item = videoGridLayout_->takeAt(0)) != nullptr) {
    if (item->widget()) item->widget()->setParent(nullptr);
    delete item;
  }

  // 按数量重新布局
  if (count == 1) {
    videoGridLayout_->addWidget(videoWidgets_[0], 0, 0);
    videoWidgets_[0]->show();
  } else if (count == 2) {
    videoGridLayout_->addWidget(videoWidgets_[0], 0, 0);
    videoGridLayout_->addWidget(videoWidgets_[1], 0, 1);
    videoWidgets_[0]->show();
    videoWidgets_[1]->show();
  } else if (count == 4) {
    int idx = 0;
    for (int r = 0; r < 2; ++r) {
      for (int c = 0; c < 2; ++c) {
        videoGridLayout_->addWidget(videoWidgets_[idx], r, c);
        videoWidgets_[idx]->show();
        idx++;
      }
    }
  }
}

void MainWidget::onModelChanged(const QString& model) {
  QSettings settings(config_path + "/mmpose.ini", QSettings::IniFormat);
  settings.beginGroup("model");
  settings.setValue("model", model);
  settings.endGroup();
}

/* ================= 读取 configA.ini ================= */

void MainWidget::loadModeConfig() {
  QSettings settings(config_path + "/config.ini", QSettings::IniFormat);
  settings.beginGroup("model");

  QStringList keys = settings.childKeys();
  for (const QString& key : keys) {
    comboModel_->addItem(settings.value(key).toString());
  }

  settings.endGroup();
}

void MainWidget::loadDetectionConfig() {
  QSettings settings(config_path + "/config.ini", QSettings::IniFormat);
  settings.beginGroup("detection");

  detConfigMap_.clear();
  detWeightsMap_.clear();
  comboDetConfig_->clear();
  comboDetWeights_->clear();

  QStringList keys = settings.childKeys();

  // 提取 config_x / weights_x
  QMap<int, QString> tempConfig;
  QMap<int, QString> tempWeights;

  for (const QString& key : keys) {
    if (key.startsWith("config_")) {
      int idx         = key.split("_").last().toInt();
      tempConfig[idx] = settings.value(key).toString();
    } else if (key.startsWith("weights_")) {
      int idx          = key.split("_").last().toInt();
      tempWeights[idx] = settings.value(key).toString();
    }
  }

  // 按 index 排序并加入 ComboBox
  QList<int> indices = tempConfig.keys();
  std::sort(indices.begin(), indices.end());

  for (int idx : indices) {
    if (!tempWeights.contains(idx)) continue;

    detConfigMap_[idx]  = tempConfig[idx];
    detWeightsMap_[idx] = tempWeights[idx];

    comboDetConfig_->addItem(tempConfig[idx]);
    comboDetWeights_->addItem(tempWeights[idx]);
  }

  settings.endGroup();

  // 默认选中第 0 项
  if (comboDetConfig_->count() > 0) {
    comboDetConfig_->setCurrentIndex(0);
    comboDetWeights_->setCurrentIndex(0);
  }
}

void MainWidget::saveDetectionToConfigMMpose(int index) {
  if (index < 0 || index >= comboDetConfig_->count()) return;

  for(int i=0;i<videoWidgets_.size();i++){
      QString ini_path = config_path + QString("/mmpose_%1.ini").arg(i+1);
      QSettings settings(ini_path, QSettings::IniFormat);
      settings.beginGroup("detection");

      settings.setValue("config", comboDetConfig_->itemText(index));
      settings.setValue("weights", comboDetWeights_->itemText(index));

      settings.endGroup();
  }

  qDebug() << comboDetConfig_->itemText(index);
  qDebug() << comboDetWeights_->itemText(index);
}

void MainWidget::loadPose2dConfig() {
  QSettings settings(config_path + "/config.ini", QSettings::IniFormat);
  settings.beginGroup("pose2d");

  poseConfigMap_.clear();
  poseWeightsMap_.clear();
  comboPoseConfig_->clear();
  comboPoseWeights_->clear();

  QStringList keys = settings.childKeys();

  // 提取 config_x / weights_x
  QMap<int, QString> tempConfig;
  QMap<int, QString> tempWeights;

  for (const QString& key : keys) {
    if (key.startsWith("config_")) {
      int idx         = key.split("_").last().toInt();
      tempConfig[idx] = settings.value(key).toString();
    } else if (key.startsWith("weights_")) {
      int idx          = key.split("_").last().toInt();
      tempWeights[idx] = settings.value(key).toString();
    }
  }

  // 按 index 排序并加入 ComboBox
  QList<int> indices = tempConfig.keys();
  std::sort(indices.begin(), indices.end());

  for (int idx : indices) {
    if (!tempWeights.contains(idx)) continue;

    poseConfigMap_[idx]  = tempConfig[idx];
    poseWeightsMap_[idx] = tempWeights[idx];

    comboPoseConfig_->addItem(tempConfig[idx]);
    comboPoseWeights_->addItem(tempWeights[idx]);
  }

  // 默认选中第 0 项
  settings.endGroup();

  if (comboPoseConfig_->count() > 0) {
    comboPoseConfig_->setCurrentIndex(0);
    comboPoseWeights_->setCurrentIndex(0);
  }
}

void MainWidget::savePose2dToConfigMMpose(int index) {
  if (index < 0 || index >= comboPoseConfig_->count()) return;

  QSettings settings(config_path + "/mmpose.ini", QSettings::IniFormat);
  settings.beginGroup("pose2d");

  settings.setValue("config", comboPoseConfig_->itemText(index));
  settings.setValue("weights", comboPoseWeights_->itemText(index));

  settings.endGroup();
  qDebug() << comboPoseConfig_->itemText(index);
  qDebug() << comboPoseWeights_->itemText(index);
}
