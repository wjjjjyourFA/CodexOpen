#include "tools/data_loader/data_loader_realtime.h"

DataLoaderRealtime::DataLoaderRealtime() {}

DataLoaderRealtime::~DataLoaderRealtime() {}

void DataLoaderRealtime::Init(std::shared_ptr<RuntimeConfigRealtime> param) {
  param_ = param;
  // 强制传给父类成员
  DataLoader::param_ = param;

  if (param_->b_camera || param_->b_infra || param_->b_star) {
    // 数据回放时，直接加载已经矫正后的图像
    // InitUndis();
  }
}