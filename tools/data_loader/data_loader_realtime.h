#ifndef DATA_LOADER_REALTIME_H
#define DATA_LOADER_REALTIME_H

#pragma once

#include <math.h>

#include "tools/data_loader/config/runtime_config.h"
#include "tools/data_loader/data_container.h"
#include "tools/data_loader/data_loader.h"

using namespace jojo::tools;

class DataLoaderRealtime : public DataLoader {
 public:
  using DataLoader::DataLoader;
  DataLoaderRealtime();
  virtual ~DataLoaderRealtime();

  bool Init(std::shared_ptr<jojo::tools::RuntimeConfig> param,
            std::shared_ptr<jojo::tools::InterfaceConfig> interface);

 protected:
  // 子类的 param_ 会遮蔽父类的 param_, 需要强制传入
  std::shared_ptr<jojo::tools::RuntimeConfig> rparam_;
  std::shared_ptr<jojo::tools::InterfaceConfig> iparam_;
};

#endif
