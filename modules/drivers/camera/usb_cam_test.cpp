#include "cyber/common/file.h"
#include "modules/drivers/camera/usb_cam.h"

// using namespace jojo::drivers::camera;
using namespace apollo::drivers::camera;
// using namespace apollo::cyber::common;

int main(int argc, char** argv) {
  std::string config_file_path_ =
      "../../../modules/drivers/camera/conf/camera_front_6mm.pb.txt";

  std::shared_ptr<Config> camera_config_ = std::make_shared<Config>();
  if (!apollo::cyber::common::GetProtoFromFile(config_file_path_,
                                               camera_config_.get())) {
    return false;
  }
  AINFO << "UsbCam config: " << camera_config_->DebugString();

  // BaseCam cam;
  // cam.init(camera_config_);
  // cam.open_device();
  // cam.init_device();
  // cam.start_capturing();

  std::unique_ptr<UsbCam> camera_device_ = std::make_unique<UsbCam>();
  uint32_t device_wait_                  = 2000;
  camera_device_->init(camera_config_);

  CameraImagePtr raw_image_              = std::make_shared<CameraImage>();
  CameraImagePtr raw_image_for_compress_ = std::make_shared<CameraImage>();

  // 这里是Cyber框架的示例
  while (!apollo::cyber::IsShutdown()) {
    if (!camera_device_->wait_for_device()) {
      // from task.h
      // sleep for next check
      // apollo::cyber::SleepFor(std::chrono::milliseconds(device_wait_));
      continue;
    }

    if (!camera_device_->poll(raw_image_, raw_image_for_compress_)) {
      AERROR << "camera device poll failed";
      continue;
    }
  }

  return 0;
}