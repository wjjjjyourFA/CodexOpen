#ifndef FEATURETENSOR_H
#define FEATURETENSOR_H

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include "model.hpp"
#include "datatype.h"
#include "cuda_runtime_api.h"

using std::vector;
using nvinfer1::ILogger;

class FeatureTensor {
public:
    FeatureTensor(const int maxBatchSize, const cv::Size imgShape, const int featureDim, int gpuID, ILogger* gLogger);
    ~FeatureTensor();

    FeatureTensor(const FeatureTensor&) = delete;
    FeatureTensor& operator=(const FeatureTensor&) = delete;

public:
    bool getRectsFeature(const cv::Mat& img, DETECTIONS& det);
    bool getRectsFeature(DETECTIONS& det);
    void loadEngine(std::string enginePath);
    void loadOnnx(std::string onnxPath);
    int getResult(float*& buffer);
    bool doInference(vector<cv::Mat>& imgMats);

private:
    void initResource();
    bool doInference(float* inputBuffer, float* outputBuffer);
    void mat2stream(vector<cv::Mat>& imgMats, float* stream);
    void stream2det(float* stream, DETECTIONS& det,
                    size_t offset, size_t count);

private:
    nvinfer1::IRuntime* runtime;
    nvinfer1::ICudaEngine* engine;
    nvinfer1::IExecutionContext* context;
    const int maxBatchSize;
    const cv::Size imgShape;
    const int featureDim;

private:
    int curBatchSize = 0;
    const int inputStreamSize, outputStreamSize;
    bool initFlag;
    float* const inputBuffer;
    float* const outputBuffer;
    int inputIndex = -1, outputIndex = -1;
    void* buffers[2] = {nullptr, nullptr};
    cudaStream_t cudaStream = nullptr;
    // BGR format
    float means[3], std[3];
    const std::string inputName, outputName;
    ILogger* gLogger;
};

#endif
