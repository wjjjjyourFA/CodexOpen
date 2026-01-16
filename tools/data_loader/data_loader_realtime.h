#ifndef DATA_LOADER_REALTIME_HH
#define DATA_LOADER_REALTIME_HH

#pragma once

#include <math.h>

#include "tools/data_loader/config/runtime_config_realtime.h"
#include "tools/data_loader/data_loader.h"
#include "tools/data_loader/data_container.h"

using namespace jojo::tools;

class DataLoaderRealtime : public DataLoader {
 public:
  using DataLoader::DataLoader;
  DataLoaderRealtime();
  virtual ~DataLoaderRealtime();

  void Init(std::shared_ptr<RuntimeConfigRealtime> param);

 protected:
  // 子类的 param_ 会遮蔽父类的 param_, 需要强制传入
  std::shared_ptr<RuntimeConfigRealtime> param_ /*param_simple*/;
};

#endif
