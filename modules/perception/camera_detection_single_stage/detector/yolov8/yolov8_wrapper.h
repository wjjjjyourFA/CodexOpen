#ifndef Yolov8Wrapper_H
#define Yolov8Wrapper_H

#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "include/cuda_utils.h"
#include "include/logging.h"
#include "include/model.h"
#include "include/postprocess.h"
#include "include/preprocess.h"
#include "include/utils.h"

using namespace nvinfer1;

static Logger gLogger_yolov8;
const static int kOutputSize =
    kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

class Yolov8Wrapper {
 public:
  Yolov8Wrapper();
  virtual ~Yolov8Wrapper();

 public:
  void set_wts(std::string in_wts_name);
  bool yolo_init(std::string in_engine_name = "");
  std::vector<std::vector<Detection>> yolov8(cv::Mat& input_image,
                                             bool show = false);

  void infer(IExecutionContext& context, cudaStream_t& stream, void** buffers,
             float* output, int batchsize, float* decode_ptr_host,
             float* decode_ptr_device, int model_bboxes,
             std::string cuda_post_process);
  void prepare_buffer(ICudaEngine* engine, float** input_buffer_device,
                      float** output_buffer_device, float** output_buffer_host,
                      float** decode_ptr_host, float** decode_ptr_device,
                      std::string cuda_post_process);
  void deserialize_engine(std::string& engine_name, IRuntime** runtime,
                          ICudaEngine** engine, IExecutionContext** context);

  void serialize_engine(std::string& wts_name, std::string& engine_name,
                        int& is_p, std::string& sub_type, float& gd, float& gw,
                        int& max_channels);

 protected:
  std::string wts_name    = "";
  std::string engine_name = "";
  std::string sub_type    = "m";
  int is_p = 0;
  float gd = 0.0f, gw = 0.0f;
  int max_channels = 0;

  std::string cuda_post_process = "g";  // c or g
  int model_bboxes;

  // 序列化
  IRuntime* runtime          = nullptr;
  ICudaEngine* engine        = nullptr;
  IExecutionContext* context = nullptr;

  cudaStream_t stream;

  // Prepare cpu and gpu buffers
  float* device_buffers[2];
  float* output_buffer_host = nullptr;
  float* decode_ptr_host    = nullptr;
  float* decode_ptr_device  = nullptr;
};

#endif  // Yolov8Wrapper_H
