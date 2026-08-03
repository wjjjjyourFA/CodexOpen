#include "tools/data_loader/data_loader_realtime.h"

DataLoaderRealtime::DataLoaderRealtime() {}

DataLoaderRealtime::~DataLoaderRealtime() {}

bool DataLoaderRealtime::Init(
    std::shared_ptr<jojo::tools::RuntimeConfig> rparam,
    std::shared_ptr<jojo::tools::InterfaceConfig> iparam) {
  rparam_ = rparam;
  iparam_ = iparam;

  // 强制传给父类成员
  DataLoader::rparam_ = rparam;
  DataLoader::iparam_ = iparam;

  if (rparam_->b_do_undistort) {
    if (iparam_->b_camera || iparam_->b_infra || iparam_->b_star) {
      // 数据回放时，直接加载已经矫正后的图像
      InitUndistortion();
    }
  }

  return true;
}