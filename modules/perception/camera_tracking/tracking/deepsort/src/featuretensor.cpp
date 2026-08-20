#include "featuretensor.h"

#include <algorithm>
#include <fstream>

using namespace nvinfer1;

#define INPUTSTREAM_SIZE (maxBatchSize*3*imgShape.area())
#define OUTPUTSTREAM_SIZE (maxBatchSize*featureDim)

FeatureTensor::FeatureTensor(const int maxBatchSize, const cv::Size imgShape, const int featureDim, int gpuID, ILogger* gLogger) 
        : maxBatchSize(maxBatchSize), imgShape(imgShape), featureDim(featureDim), 
        inputStreamSize(INPUTSTREAM_SIZE), outputStreamSize(OUTPUTSTREAM_SIZE),
        inputBuffer(new float[inputStreamSize]), outputBuffer(new float[outputStreamSize]),
        inputName("input"), outputName("output") {
    cudaSetDevice(gpuID);
    this->gLogger = gLogger;
    runtime = nullptr;
    engine = nullptr;
    context = nullptr; 

    means[0] = 0.485, means[1] = 0.456, means[2] = 0.406;
    std[0] = 0.229, std[1] = 0.224, std[2] = 0.225;

    initFlag = false;
}

FeatureTensor::~FeatureTensor() {
    if (initFlag) {
        cudaStreamSynchronize(cudaStream);
        cudaFree(buffers[inputIndex]);
        cudaFree(buffers[outputIndex]);
        cudaStreamDestroy(cudaStream);
    }

#if NV_TENSORRT_MAJOR >= 8
    delete context;
    delete engine;
    delete runtime;
#else
    if (context != nullptr) context->destroy();
    if (engine != nullptr) engine->destroy();
    if (runtime != nullptr) runtime->destroy();
#endif

    delete [] inputBuffer;
    delete [] outputBuffer;
}

bool FeatureTensor::getRectsFeature(const cv::Mat& img, DETECTIONS& det) {
    if (img.empty() || maxBatchSize <= 0) {
        return false;
    }
    if (det.empty()) {
        return true;
    }

    const cv::Rect imageBounds(0, 0, img.cols, img.rows);
    for (size_t offset = 0; offset < det.size();
         offset += static_cast<size_t>(maxBatchSize)) {
        const size_t count = std::min(
            static_cast<size_t>(maxBatchSize), det.size() - offset);

        std::vector<cv::Mat> mats;
        mats.reserve(count);
        for (size_t i = offset; i < offset + count; ++i) {
            auto& dbox = det[i];
            cv::Rect rect(int(dbox.tlwh(0)), int(dbox.tlwh(1)),
                          int(dbox.tlwh(2)), int(dbox.tlwh(3)));
            if (rect.width <= 0 || rect.height <= 0) {
                return false;
            }

            rect.x -= static_cast<int>(
                (rect.height * 0.5 - rect.width) * 0.5);
            rect.width = static_cast<int>(rect.height * 0.5);
            rect &= imageBounds;
            if (rect.empty()) {
                return false;
            }

            cv::Mat resized;
            cv::resize(img(rect), resized, imgShape);
            mats.emplace_back(std::move(resized));
        }

        if (!doInference(mats)) {
            return false;
        }
        stream2det(outputBuffer, det, offset, count);
    }

    return true;
}

bool FeatureTensor::getRectsFeature(DETECTIONS& det) {
    return true;
}

void FeatureTensor::loadEngine(std::string enginePath) {
    // Deserialize model
    runtime = createInferRuntime(*gLogger);
    assert(runtime != nullptr);
    std::ifstream engineStream(enginePath, std::ios::binary);
    std::string engineCache("");
    while (engineStream.peek() != EOF) {
        std::stringstream buffer;
        buffer << engineStream.rdbuf();
        engineCache.append(buffer.str());
    }
    engineStream.close();
    engine = runtime->deserializeCudaEngine(engineCache.data(), engineCache.size(), nullptr);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    initResource();
} 

void FeatureTensor::loadOnnx(std::string onnxPath) {
    auto builder = createInferBuilder(*gLogger);
    assert(builder != nullptr);
    const auto explicitBatch = 1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = builder->createNetworkV2(explicitBatch);
    assert(network != nullptr);
    auto config = builder->createBuilderConfig();
    assert(config != nullptr);

    auto profile = builder->createOptimizationProfile();
    Dims dims = Dims4{1, 3, imgShape.height, imgShape.width};
    profile->setDimensions(inputName.c_str(),
                OptProfileSelector::kMIN, Dims4{1, dims.d[1], dims.d[2], dims.d[3]});
    profile->setDimensions(inputName.c_str(),
                OptProfileSelector::kOPT, Dims4{maxBatchSize, dims.d[1], dims.d[2], dims.d[3]});
    profile->setDimensions(inputName.c_str(),
                OptProfileSelector::kMAX, Dims4{maxBatchSize, dims.d[1], dims.d[2], dims.d[3]});
    config->addOptimizationProfile(profile);

    nvonnxparser::IParser* parser = nvonnxparser::createParser(*network, *gLogger);
    assert(parser != nullptr);
    auto parsed = parser->parseFromFile(onnxPath.c_str(), static_cast<int>(ILogger::Severity::kWARNING));
    assert(parsed);
    config->setMaxWorkspaceSize(1 << 20);
    engine = builder->buildEngineWithConfig(*network, *config);
    assert(engine != nullptr);
    context = engine->createExecutionContext();
    assert(context != nullptr);
    initResource();
}

int FeatureTensor::getResult(float*& buffer) {
    if (buffer != nullptr)
        delete [] buffer;
    int curStreamSize = curBatchSize*featureDim;
    buffer = new float[curStreamSize];
    for (int i = 0; i < curStreamSize; ++i) {
        buffer[i] = outputBuffer[i];
    }
    return curStreamSize;
}

bool FeatureTensor::doInference(vector<cv::Mat>& imgMats) {
    if (imgMats.empty()) {
        return true;
    }
    mat2stream(imgMats, inputBuffer);
    return doInference(inputBuffer, outputBuffer);
}

void FeatureTensor::initResource() {
    inputIndex = engine->getBindingIndex(inputName.c_str());
    outputIndex = engine->getBindingIndex(outputName.c_str());
    if (inputIndex < 0 || outputIndex < 0 || inputIndex >= 2 ||
        outputIndex >= 2 || inputIndex == outputIndex) {
        std::cerr << "DeepSORT engine bindings are invalid." << std::endl;
        return;
    }

    // Create CUDA stream
    cudaError_t error = cudaStreamCreate(&cudaStream);
    if (error != cudaSuccess) {
        std::cerr << "DeepSORT stream creation failed: "
                  << cudaGetErrorString(error) << std::endl;
        cudaStream = nullptr;
        return;
    }

    // Malloc CUDA memory
    error = cudaMalloc(&buffers[inputIndex], inputStreamSize * sizeof(float));
    if (error != cudaSuccess) {
        std::cerr << "DeepSORT input allocation failed: "
                  << cudaGetErrorString(error) << std::endl;
        cudaStreamDestroy(cudaStream);
        cudaStream = nullptr;
        return;
    }

    error = cudaMalloc(&buffers[outputIndex], outputStreamSize * sizeof(float));
    if (error != cudaSuccess) {
        std::cerr << "DeepSORT output allocation failed: "
                  << cudaGetErrorString(error) << std::endl;
        cudaFree(buffers[inputIndex]);
        buffers[inputIndex] = nullptr;
        cudaStreamDestroy(cudaStream);
        cudaStream = nullptr;
        return;
    }

    initFlag = true;
}

bool FeatureTensor::doInference(float* inputBuffer, float* outputBuffer) {
    if (!initFlag || context == nullptr || curBatchSize <= 0) {
        return false;
    }

    const size_t inputBytes = static_cast<size_t>(curBatchSize) * 3 *
                              imgShape.area() * sizeof(float);
    const size_t outputBytes = static_cast<size_t>(curBatchSize) *
                               featureDim * sizeof(float);

    cudaError_t error = cudaMemcpyAsync(buffers[inputIndex], inputBuffer,
                                        inputBytes, cudaMemcpyHostToDevice,
                                        cudaStream);
    if (error != cudaSuccess) {
        std::cerr << "DeepSORT H2D copy failed: "
                  << cudaGetErrorString(error) << std::endl;
        return false;
    }

    Dims4 inputDims{curBatchSize, 3, imgShape.height, imgShape.width};
    if (!context->setBindingDimensions(inputIndex, inputDims) ||
        !context->enqueueV2(buffers, cudaStream, nullptr)) {
        std::cerr << "DeepSORT inference enqueue failed." << std::endl;
        return false;
    }

    error = cudaMemcpyAsync(outputBuffer, buffers[outputIndex], outputBytes,
                            cudaMemcpyDeviceToHost, cudaStream);
    if (error != cudaSuccess) {
        std::cerr << "DeepSORT D2H copy failed: "
                  << cudaGetErrorString(error) << std::endl;
        return false;
    }

    error = cudaStreamSynchronize(cudaStream);
    if (error != cudaSuccess) {
        std::cerr << "DeepSORT stream synchronization failed: "
                  << cudaGetErrorString(error) << std::endl;
        return false;
    }
    return true;
}

void FeatureTensor::mat2stream(vector<cv::Mat>& imgMats, float* stream) {
    int imgArea = imgShape.area();
    curBatchSize = imgMats.size();
    if (curBatchSize > maxBatchSize) {
        std::cout << "[WARNING]::Batch size overflow, input will be truncated!" << std::endl;
        curBatchSize = maxBatchSize;
    }
    for (int batch = 0; batch < curBatchSize; ++batch) {
        cv::Mat tempMat = imgMats[batch];
        int i = 0; 
        for (int row = 0; row < imgShape.height; ++row) {
            uchar* uc_pixel = tempMat.data + row * tempMat.step;
            for (int col = 0; col < imgShape.width; ++col) {
                stream[batch * 3 * imgArea + i] = ((float)uc_pixel[0] / 255.0 - means[0]) / std[0];
                stream[batch * 3 * imgArea + i + imgArea] = ((float)uc_pixel[1] / 255.0 - means[1]) / std[1];
                stream[batch * 3 * imgArea + i + 2 * imgArea] = ((float)uc_pixel[2] / 255.0 - means[2]) / std[2];
                uc_pixel += 3;
                ++i;
            }
        }
    }
}

void FeatureTensor::stream2det(float* stream, DETECTIONS& det,
                               size_t offset, size_t count) {
    const size_t end = std::min(det.size(), offset + count);
    size_t batchIndex = 0;
    for (size_t i = offset; i < end; ++i, ++batchIndex) {
        DETECTION_ROW& dbox = det[i];
        for (int j = 0; j < featureDim; ++j)
            dbox.feature[j] = stream[batchIndex * featureDim + j];
            // dbox.feature[j] = (float)1.0;
    }
}
