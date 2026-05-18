#include <yolov5_wrapper.h>

using namespace std;

Yolov5Wrapper::Yolov5Wrapper() {
  // yolo_init();
}

Yolov5Wrapper::~Yolov5Wrapper() {
  // Release stream and buffers
  cudaStreamDestroy(stream);
  CUDA_CHECK(cudaFree(gpu_buffers[0]));
  CUDA_CHECK(cudaFree(gpu_buffers[1]));
  delete[] cpu_output_buffer;
  cuda_preprocess_destroy();
  // Destroy the engine
  context->destroy();
  engine->destroy();
  runtime->destroy();
}

void Yolov5Wrapper::set_wts(std::string in_wts_name) { wts_name = in_wts_name; }

bool Yolov5Wrapper::yolo_init(std::string in_engine_name) {
  cudaSetDevice(kGpuId);

  // std::string wts_name = "";
  // std::string engine_name = "";
  // bool is_p6 = false;
  // float gd = 0.0f, gw = 0.0f;

  if (in_engine_name.empty()) {
    // Create a model using the API directly and serialize it to a file
    if (!wts_name.empty()) {
      serialize_engine(kBatchSize, is_p6, gd, gw, wts_name, engine_name);
      // return 0;
    } else {
      std::cout << "... please input the wts name or engine name" << std::endl;
      return false;
    }
  } else {
    engine_name = in_engine_name;
  }

  // prepare input data ---------------------------
  // Deserialize the engine from file
  // 将序列化得到的结果进行反序列化，以执行后续的inference
  this->deserialize_engine(engine_name, &runtime, &engine, &context);
  // cudaStream_t stream;
  CUDA_CHECK(cudaStreamCreate(&stream));

  // Init CUDA preprocessing
  cuda_preprocess_init(kMaxInputImageSize);

  // Prepare cpu and gpu buffers
  // float* gpu_buffers[2];
  // float* cpu_output_buffer = nullptr;
  prepare_buffers(engine, &gpu_buffers[0], &gpu_buffers[1], &cpu_output_buffer);

  return true;
}

std::vector<std::vector<Detection>> Yolov5Wrapper::yolov5(cv::Mat& input_image,
                                                          bool show) {
  // capture >> img1;
  // cv::Mat img1 = cv::imread("./../data/zidane.jpg");
  cv::Mat img1 = input_image;
  cv::Mat img;
  // cv::resize(img1, img, cv::Size(kInputW, kInputH));
  img = img1;

  // image input here !
  // result base on this image !
  if (img.empty()) {
    std::cout << "Yolov5Wrapper --> image is empty" << std::endl;
  };

  // 图像归一化
  std::vector<cv::Mat> img_batch;
  for (size_t j = 0; j < 0 + kBatchSize; j++) {
    img_batch.push_back(img);
  }
  // std::cout << "img_batch.size() : " << img_batch.size() << std::endl;

  // Preprocess
  cuda_batch_preprocess(img_batch, gpu_buffers[0], kInputW, kInputH, stream);

  // Run inference
  auto start = std::chrono::system_clock::now();
  infer(*context, stream, (void**)gpu_buffers, cpu_output_buffer, kBatchSize);
  auto end = std::chrono::system_clock::now();
  std::cout << "----> inference time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << std::endl;
  // std::cout.flush();  // 强制刷新日志

  // NMS
  std::vector<std::vector<Detection>> res_batch;
  batch_nms(res_batch, cpu_output_buffer, img_batch.size(), kOutputSize,
            kConfThresh, kNmsThresh);

  // /*
  // Draw bounding boxes
  if (show) {
    draw_bbox(img_batch, res_batch);
  }
  // */

  return res_batch;
}

int Yolov5Wrapper::get_width(int x, float gw, int divisor = 8) {
  return int(ceil((x * gw) / divisor)) * divisor;
}

int Yolov5Wrapper::get_depth(int x, float gd) {
  if (x == 1) return 1;
  int r = round(x * gd);
  if (x * gd - int(x * gd) == 0.5 && (int(x * gd) % 2) == 0) {
    --r;
  }
  return std::max<int>(r, 1);
}

void Yolov5Wrapper::prepare_buffers(ICudaEngine* engine, float** gpu_input_buffer, float** gpu_output_buffer, float** cpu_output_buffer) {
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  const int inputIndex = engine->getBindingIndex(kInputTensorName);
  const int outputIndex = engine->getBindingIndex(kOutputTensorName);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc((void**)gpu_input_buffer, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void**)gpu_output_buffer, kBatchSize * kOutputSize * sizeof(float)));

  *cpu_output_buffer = new float[kBatchSize * kOutputSize];
}

void Yolov5Wrapper::infer(IExecutionContext& context, cudaStream_t& stream, void** gpu_buffers, float* output, int batchsize) {
  context.enqueue(batchsize, gpu_buffers, stream, nullptr);
  CUDA_CHECK(cudaMemcpyAsync(output, gpu_buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, stream));
  cudaStreamSynchronize(stream);
}

void Yolov5Wrapper::serialize_engine(unsigned int max_batchsize, bool& is_p6, float& gd, float& gw, std::string& wts_name, std::string& engine_name) {
  // Create builder
  IBuilder* builder = createInferBuilder(gLogger_yolov5);
  IBuilderConfig* config = builder->createBuilderConfig();

  // Create model to populate the network, then set the outputs and create an engine
  ICudaEngine* engine = nullptr;
  if (is_p6) {
    engine = build_det_p6_engine(max_batchsize, builder, config, nvinfer1::DataType::kFLOAT, gd, gw, wts_name);
  } else {
    engine = build_det_engine(max_batchsize, builder, config, nvinfer1::DataType::kFLOAT, gd, gw, wts_name);
  }
  assert(engine != nullptr);

  // Serialize the engine
  IHostMemory* serialized_engine = engine->serialize();
  assert(serialized_engine != nullptr);

  // Save engine to file
  std::ofstream p(engine_name, std::ios::binary);
  if (!p) {
    std::cerr << "Could not open plan output file" << std::endl;
    assert(false);
  }
  p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

  // Close everything down
  engine->destroy();
  config->destroy();
  serialized_engine->destroy();
  builder->destroy();
}

void Yolov5Wrapper::deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, IExecutionContext** context) {
  std::ifstream file(engine_name, std::ios::binary);
  if (!file.good()) {
    std::cerr << "read " << engine_name << " error!" << std::endl;
    assert(false);
  }
  size_t size = 0;
  file.seekg(0, file.end);
  size = file.tellg();
  file.seekg(0, file.beg);
  char* serialized_engine = new char[size];
  assert(serialized_engine);
  file.read(serialized_engine, size);
  file.close();

  *runtime = createInferRuntime(gLogger_yolov5);
  assert(*runtime);
  *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
  assert(*engine);
  *context = (*engine)->createExecutionContext();
  assert(*context);
  delete[] serialized_engine;
}
