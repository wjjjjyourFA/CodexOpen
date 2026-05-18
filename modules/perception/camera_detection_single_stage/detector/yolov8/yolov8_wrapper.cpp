#include <yolov8_wrapper.h>

using namespace std;

Yolov8Wrapper::Yolov8Wrapper() {
  // yolo_init();
}

Yolov8Wrapper::~Yolov8Wrapper() {
  // Release stream and buffers
  cudaStreamDestroy(stream);
  CUDA_CHECK(cudaFree(device_buffers[0]));
  CUDA_CHECK(cudaFree(device_buffers[1]));
  CUDA_CHECK(cudaFree(decode_ptr_device));
  delete[] decode_ptr_host;
  delete[] output_buffer_host;
  cuda_preprocess_destroy();
  // Destroy the engine
  delete context;
  delete engine;
  delete runtime;
}

void Yolov8Wrapper::set_wts(std::string in_wts_name) { wts_name = in_wts_name; }

bool Yolov8Wrapper::yolo_init(std::string in_engine_name) {
  cudaSetDevice(kGpuId);
  // int is_p = 0;
  // float gd = 0.0f, gw = 0.0f;
  // int max_channels = 0;
  if (in_engine_name.empty()) {
    // Create a model using the API directly and serialize it to a file
    if (!wts_name.empty()) {
      serialize_engine(wts_name, engine_name, is_p, sub_type, gd, gw, max_channels);
      // return 0;
    } else {
      std::cout << "... please input the wts name or engine name" << std::endl;
      return false;
    }
  } else {
    engine_name = in_engine_name;
  }

  // Deserialize the engine from file
  // IRuntime* runtime = nullptr;
  // ICudaEngine* engine = nullptr;
  // IExecutionContext* context = nullptr;
  deserialize_engine(engine_name, &runtime, &engine, &context);
  // cudaStream_t stream;
  CUDA_CHECK(cudaStreamCreate(&stream));
  // Init CUDA preprocessing
  cuda_preprocess_init(kMaxInputImageSize);
  auto out_dims = engine->getBindingDimensions(1);
  model_bboxes  = out_dims.d[0];
  // Prepare cpu and gpu buffers
  // float* device_buffers[2];
  // float* output_buffer_host = nullptr;
  // float* decode_ptr_host = nullptr;
  // float* decode_ptr_device = nullptr;

  prepare_buffer(engine, &device_buffers[0], &device_buffers[1], &output_buffer_host, &decode_ptr_host,
                 &decode_ptr_device, cuda_post_process);

  return true;
}

std::vector<std::vector<Detection>> Yolov8Wrapper::yolov8(cv::Mat& input_image,
                                                          bool show) {
  cv::Mat img1 = input_image;
  cv::Mat img;
  // cv::resize(img1, img, cv::Size(kInputW, kInputH));
  img = img1;

  // image input here !
  // result base on this image !
  if (img.empty()) {
    std::cout << "image is empty" << std::endl;
  };

  // 图像归一化
  std::vector<cv::Mat> img_batch;
  for (size_t j = 0; j < 0 + kBatchSize; j++) {
    img_batch.push_back(img);
  }
  // std::cout << "img_batch.size() : " << img_batch.size() << std::endl;

  // Preprocess
  cuda_batch_preprocess(img_batch, device_buffers[0], kInputW, kInputH, stream);

  // Run inference
  auto start = std::chrono::system_clock::now();
  infer(*context, stream, (void**)device_buffers, output_buffer_host, kBatchSize, decode_ptr_host,
        decode_ptr_device, model_bboxes, cuda_post_process);
  auto end = std::chrono::system_clock::now();
  std::cout << "----> inference time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms" << std::endl;
  // std::cout.flush();  // 强制刷新日志

  // NMS
  std::vector<std::vector<Detection>> res_batch;
  if (cuda_post_process == "c") {
    // NMS
    batch_nms(res_batch, output_buffer_host, img_batch.size(), kOutputSize,
              kConfThresh, kNmsThresh);
  } else if (cuda_post_process == "g") {
    //Process gpu decode and nms results
    batch_process(res_batch, decode_ptr_host, img_batch.size(), bbox_element,
                  img_batch);
  }

  // /*
  // Draw bounding boxes
  if (show) {
    draw_bbox(img_batch, res_batch);
  }
  // */

  return res_batch;
}

void Yolov8Wrapper::serialize_engine(std::string& wts_name, std::string& engine_name, int& is_p, std::string& sub_type, float& gd,
                                     float& gw, int& max_channels) {
  IBuilder* builder = createInferBuilder(gLogger_yolov8);
  IBuilderConfig* config = builder->createBuilderConfig();
  IHostMemory* serialized_engine = nullptr;

  if (is_p == 6) {
    serialized_engine = buildEngineYolov8DetP6(builder, config, nvinfer1::DataType::kFLOAT, wts_name, gd, gw, max_channels);
  } else if (is_p == 2) {
    serialized_engine = buildEngineYolov8DetP2(builder, config, nvinfer1::DataType::kFLOAT, wts_name, gd, gw, max_channels);
  } else {
    serialized_engine = buildEngineYolov8Det(builder, config, nvinfer1::DataType::kFLOAT, wts_name, gd, gw, max_channels);
  }

  assert(serialized_engine);
  // Save engine to file
  std::ofstream p(engine_name, std::ios::binary);
  if (!p) {
    std::cout << "could not open plan output file" << std::endl;
    assert(false);
  }
  p.write(reinterpret_cast<const char*>(serialized_engine->data()), serialized_engine->size());

  // Close everything down
  delete serialized_engine;
  delete config;
  delete builder;
}

void Yolov8Wrapper::deserialize_engine(std::string& engine_name, IRuntime** runtime, ICudaEngine** engine, 
                                       IExecutionContext** context) {
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

  *runtime = createInferRuntime(gLogger_yolov8);
  assert(*runtime);
  *engine = (*runtime)->deserializeCudaEngine(serialized_engine, size);
  assert(*engine);
  *context = (*engine)->createExecutionContext();
  assert(*context);
  delete[] serialized_engine;
}

void Yolov8Wrapper::prepare_buffer(ICudaEngine* engine, float** input_buffer_device, float** output_buffer_device,
                                   float** output_buffer_host, float** decode_ptr_host, float** decode_ptr_device,
                                   std::string cuda_post_process) {
  assert(engine->getNbBindings() == 2);
  // In order to bind the buffers, we need to know the names of the input and output tensors.
  // Note that indices are guaranteed to be less than IEngine::getNbBindings()
  const int inputIndex  = engine->getBindingIndex(kInputTensorName);
  const int outputIndex = engine->getBindingIndex(kOutputTensorName);
  assert(inputIndex == 0);
  assert(outputIndex == 1);
  // Create GPU buffers on device
  CUDA_CHECK(cudaMalloc((void**)input_buffer_device, kBatchSize * 3 * kInputH * kInputW * sizeof(float)));
  CUDA_CHECK(cudaMalloc((void**)output_buffer_device, kBatchSize * kOutputSize * sizeof(float)));
  if (cuda_post_process == "c") {
    *output_buffer_host = new float[kBatchSize * kOutputSize];
  } else if (cuda_post_process == "g") {
    if (kBatchSize > 1) {
      std::cerr << "Do not yet support GPU post processing for multiple batches" << std::endl;
      exit(0);
    }
    // Allocate memory for decode_ptr_host and copy to device
    *decode_ptr_host = new float[1 + kMaxNumOutputBbox * bbox_element];
    CUDA_CHECK(cudaMalloc((void**)decode_ptr_device, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element)));
  }
}

void Yolov8Wrapper::infer(IExecutionContext& context, cudaStream_t& stream, void** buffers, float* output, int batchsize,
                          float* decode_ptr_host, float* decode_ptr_device, int model_bboxes, std::string cuda_post_process) {
  // infer on the batch asynchronously, and DMA output back to host
  // auto start = std::chrono::system_clock::now();
  context.enqueue(batchsize, buffers, stream, nullptr);
  if (cuda_post_process == "c") {
    CUDA_CHECK(cudaMemcpyAsync(output, buffers[1], batchsize * kOutputSize * sizeof(float), cudaMemcpyDeviceToHost, 
                               stream));
    // auto end = std::chrono::system_clock::now();
    // std::cout << "inference time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
    //           << "ms" << std::endl;
  } else if (cuda_post_process == "g") {
    CUDA_CHECK(
            cudaMemsetAsync(decode_ptr_device, 0, sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), stream));
    cuda_decode((float*)buffers[1], model_bboxes, kConfThresh, decode_ptr_device, kMaxNumOutputBbox, stream);
    cuda_nms(decode_ptr_device, kNmsThresh, kMaxNumOutputBbox, stream);  //cuda nms
    CUDA_CHECK(cudaMemcpyAsync(decode_ptr_host, decode_ptr_device,
                               sizeof(float) * (1 + kMaxNumOutputBbox * bbox_element), cudaMemcpyDeviceToHost, 
                               stream));
    // auto end = std::chrono::system_clock::now();
    // std::cout << "inference and gpu postprocess time: "
    //           << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;
  }

  CUDA_CHECK(cudaStreamSynchronize(stream));
}
