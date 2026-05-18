#ifndef Yolov5Wrapper_H
#define Yolov5Wrapper_H

#include <iostream>
#include <chrono>
#include <cmath>

#include "cuda_runtime_api.h"
#include "NvInfer.h"

#include "src/cuda_utils.h"
#include "src/logging.h"
#include "src/utils.h"
//#include "src/calibrator.h"
#include "src/preprocess.h"
#include "src/postprocess.h"
#include "src/model.h"

using namespace nvinfer1;

static Logger gLogger_yolov5;
const static int kOutputSize =
    kMaxNumOutputBbox * sizeof(Detection) / sizeof(float) + 1;

class Yolov5Wrapper {
 public:
  Yolov5Wrapper();
  virtual ~Yolov5Wrapper();

 public:
  void set_wts(std::string in_wts_name);
  bool yolo_init(std::string in_engine_name = "");
  std::vector<std::vector<Detection>> yolov5(cv::Mat& input_image,
                                             bool show = false);

  int get_width(int x, float gw, int divisor);
  int get_depth(int x, float gd);

  void infer(IExecutionContext& context, cudaStream_t& stream, void** buffers,
             float* output, int batchSize);
  void prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer,
                       float** gpu_output_buffer, float** cpu_output_buffer);
  void deserialize_engine(std::string& engine_name, IRuntime** runtime,
                          ICudaEngine** engine, IExecutionContext** context);

  void serialize_engine(unsigned int max_batchsize, bool& is_p6, float& gd,
                        float& gw, std::string& wts_name,
                        std::string& engine_name);

 protected:
  std::string wts_name    = "";
  std::string engine_name = "";
  bool is_p6 = false;
  float gd = 0.0f, gw = 0.0f;

  // Deserialize the engine from file
  IRuntime* runtime          = nullptr;
  ICudaEngine* engine        = nullptr;
  IExecutionContext* context = nullptr;

  cudaStream_t stream;

  // Prepare cpu and gpu buffers
  float* gpu_buffers[2];
  float* cpu_output_buffer = nullptr;
};

#endif  // TOOL_YOLO4_H
