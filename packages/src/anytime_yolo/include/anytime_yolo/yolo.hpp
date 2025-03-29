#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvOnnxParser.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <variant>
#include <vector>

using namespace nvinfer1;
using namespace nvonnxparser;
using json = nlohmann::json;

#define CHECK_CUDA(call)                                                                   \
  do {                                                                                     \
    cudaError_t status = call;                                                             \
    if (status != cudaSuccess) {                                                           \
      std::cerr << "CUDA Error: " << cudaGetErrorString(status) << " at line " << __LINE__ \
                << std::endl;                                                              \
      exit(EXIT_FAILURE);                                                                  \
    }                                                                                      \
  } while (0)

// Basic TensorRT logger
class Logger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cout << msg << std::endl;
    }
  }
} logger;

class CudaBuffer
{
public:
  CudaBuffer() : device_ptr(nullptr), size(0) {}

  ~CudaBuffer() { free(); }

  CudaBuffer(const CudaBuffer & other) : device_ptr(nullptr), size(0)
  {
    if (other.size > 0) {
      // Allocate memory of the same size
      if (allocate(other.size)) {
        // Copy data from other's device memory to this device memory
        cudaError_t status =
          cudaMemcpy(device_ptr, other.device_ptr, size, cudaMemcpyDeviceToDevice);
        if (status != cudaSuccess) {
          std::cerr << "Error copying data in copy constructor: " << cudaGetErrorString(status)
                    << std::endl;
          free();  // Clean up if copy failed
        }
      }
    }
  }

  CudaBuffer & operator=(const CudaBuffer & other)
  {
    if (this != &other) {  // Self-assignment check
      // Free existing resources
      free();

      // Allocate and copy if the source has data
      if (other.size > 0) {
        if (allocate(other.size)) {
          cudaError_t status =
            cudaMemcpy(device_ptr, other.device_ptr, size, cudaMemcpyDeviceToDevice);
          if (status != cudaSuccess) {
            std::cerr << "Error copying data in assignment operator: " << cudaGetErrorString(status)
                      << std::endl;
            free();  // Clean up if copy failed
          }
        }
      }
    }
    return *this;
  }

  CudaBuffer(CudaBuffer && other) noexcept : device_ptr(nullptr), size(0)
  {
    // Steal resources from other
    device_ptr = other.device_ptr;
    size = other.size;

    // Leave other in a valid but empty state
    other.device_ptr = nullptr;
    other.size = 0;
  }

  // Move assignment operator
  CudaBuffer & operator=(CudaBuffer && other) noexcept
  {
    if (this != &other) {  // Self-assignment check
      // Free existing resources
      free();

      // Steal resources from other
      device_ptr = other.device_ptr;
      size = other.size;

      // Leave other in a valid but empty state
      other.device_ptr = nullptr;
      other.size = 0;
    }
    return *this;
  }

  // Allocate device memory
  bool allocate(size_t size)
  {
    free();
    this->size = size;
    cudaError_t status = cudaMalloc(&device_ptr, size);
    if (status != cudaSuccess) {
      std::cerr << "Error allocating CUDA memory: " << cudaGetErrorString(status) << std::endl;
    }
    return (status == cudaSuccess);
  }

  // Copy from host to device
  bool copyFromHost(const void * host_ptr, size_t size)
  {
    if (size > this->size || device_ptr == nullptr) {
      return false;
    }
    cudaError_t status = cudaMemcpy(device_ptr, host_ptr, size, cudaMemcpyHostToDevice);
    return (status == cudaSuccess);
  }

  // Copy from device to host
  bool copyToHost(void * host_ptr, size_t size) const
  {
    if (size > this->size || device_ptr == nullptr) {
      return false;
    }
    cudaError_t status = cudaMemcpy(host_ptr, device_ptr, size, cudaMemcpyDeviceToHost);
    if (status != cudaSuccess) {
      std::cerr << "Error copying data from device to host: " << cudaGetErrorString(status)
                << std::endl;
    }
    return (status == cudaSuccess);
  }

  // Get device pointer
  void * getDevicePtr() const { return device_ptr; }

  // Get size
  size_t getSize() const { return size; }

  // Free memory
  void free()
  {
    if (device_ptr != nullptr) {
      cudaFree(device_ptr);
      device_ptr = nullptr;
    }
    size = 0;
  }

private:
  void * device_ptr;
  size_t size;
};

std::vector<float> convertToNCHW(const cv::Mat & img)
{
  const int channels = img.channels();
  const int height = img.rows;
  const int width = img.cols;
  std::vector<float> nchw_data(channels * height * width);

  for (int c = 0; c < channels; c++) {
    for (int h = 0; h < height; h++) {
      for (int w = 0; w < width; w++) {
        nchw_data[c * height * width + h * width + w] = img.at<cv::Vec3f>(h, w)[c];
      }
    }
  }

  return nchw_data;
}

bool loadImageFromFile(
  const std::string & imagePath, CudaBuffer & buffer, [[maybe_unused]] bool halfPrecision = false)
{
  cv::Mat img = cv::imread(imagePath);
  if (img.empty()) {
    std::cerr << "Failed to load image: " << imagePath << std::endl;
    return false;
  }

  cv::Mat resized_img;
  cv::resize(img, resized_img, cv::Size(640, 640));

  cv::cvtColor(resized_img, resized_img, cv::COLOR_BGR2RGB);

  cv::Mat float_img;
  resized_img.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

  const auto nchw_data = convertToNCHW(float_img);

  std::vector<__half> nchw_half_data(nchw_data.size());
  for (size_t i = 0; i < nchw_data.size(); i++) {
    nchw_half_data[i] = __float2half(nchw_data[i]);
  }

  // const size_t data_size = nchw_data.size() * sizeof(__half);
  const size_t data_size = nchw_data.size() * sizeof(float);

  if (!buffer.allocate(data_size)) {
    std::cerr << "Failed to allocate CUDA buffer" << std::endl;
    return false;
  }

  if (!buffer.copyFromHost(nchw_data.data(), data_size)) {
    std::cerr << "Failed to copy image to CUDA buffer" << std::endl;
    return false;
  }

  return true;
}

bool saveEngineToFile(const std::string & enginePath, const IHostMemory & serializedEngine)
{
  std::ofstream engineFile(enginePath, std::ios::binary);
  if (!engineFile) {
    std::cerr << "Failed to open file for writing: " << enginePath << std::endl;
    return false;
  }

  engineFile.write(static_cast<const char *>(serializedEngine.data()), serializedEngine.size());

  engineFile.close();

  if (engineFile.fail()) {
    std::cerr << "Error occurred while writing engine to file" << std::endl;
    return false;
  }

  std::cout << "Engine successfully serialized to: " << enginePath << std::endl;
  return true;
}

bool buildOnnxEngine(const std::string & onnx_path, const std::string & output_path)
{
  // setup the builder and network, then parse the model

  const std::unique_ptr<IBuilder> builder(createInferBuilder(logger));

  if (!builder) {
    return false;
  }
  const std::unique_ptr<INetworkDefinition> network(builder->createNetworkV2(0));
  if (!network) {
    return false;
  }

  const std::unique_ptr<IParser> parser(createParser(*network, logger));
  if (!parser) {
    return false;
  }

  parser->parseFromFile(onnx_path.c_str(), static_cast<int32_t>(ILogger::Severity::kWARNING));
  for (int32_t i = 0; i < parser->getNbErrors(); ++i) {
    std::cout << parser->getError(i)->desc() << std::endl;
  }

  const std::unique_ptr<IBuilderConfig> config(builder->createBuilderConfig());
  if (!config) {
    return false;
  }

  std::unique_ptr<IHostMemory> serializedEngine(builder->buildSerializedNetwork(*network, *config));
  if (!serializedEngine) {
    return false;
  }

  if (!saveEngineToFile(output_path, *serializedEngine)) {
    return false;
  }
  return true;
}

bool loadEngine(
  const std::string & enginePath, std::unique_ptr<IRuntime> & runtime,
  std::unique_ptr<ICudaEngine> & engine)
{
  std::ifstream engineFile(enginePath, std::ios::binary);
  if (!engineFile) {
    std::cerr << "Failed to open file for reading: " << enginePath << std::endl;
    return false;
  }

  engineFile.seekg(0, std::ios::end);
  const size_t engineSize = engineFile.tellg();
  engineFile.seekg(0, std::ios::beg);

  std::unique_ptr<char[]> engineData(new char[engineSize]);
  engineFile.read(engineData.get(), engineSize);
  if (engineFile.fail()) {
    std::cerr << "Error occurred while reading engine from file" << std::endl;
    return false;
  }
  engineFile.close();
  // Validate data before deserialization
  if (engineSize == 0) {
    std::cerr << "Engine file is empty" << std::endl;
    return false;
  }

  std::cout << "EngineData pointer" << engineData.get() << std::endl;

  engine =
    std::unique_ptr<ICudaEngine>(runtime->deserializeCudaEngine(engineData.get(), engineSize));
  if (!engine) {
    return false;
  }

  return true;
}

struct AnytimeYOLOChunk
{
  const std::string type = "";
  const int index;
  const std::vector<int> f;

  std::unique_ptr<ICudaEngine> engine;
  std::unique_ptr<IExecutionContext> context;
};
class InferenceState
{
public:
  enum Stage { LAYER_PROCESSING, EXIT_PROCESSING, NMS_PROCESSING, COMPLETED };

  InferenceState(
    const CudaBuffer & inputBuffer, [[maybe_unused]] cudaStream_t stream,
    [[maybe_unused]] bool halfPrecision, int chunkCount,
    [[maybe_unused]] int exitIndexToProcess = 0)
  : currentStage(LAYER_PROCESSING), currentIndex(0)
  {
    this->inputBuffer.allocate(inputBuffer.getSize());
    this->inputBuffer.copyFromHost(inputBuffer.getDevicePtr(), inputBuffer.getSize());

    // Initialize output buffers for each layer
    layerOutputBuffers.resize(chunkCount);
    outputDims.resize(chunkCount);

    // Initialize exit layer output buffers
    // By default, we only process exit 0
    exitOutputDims.resize(1);

    // Initialize NMS output buffers
    finalOutput.resize(1);
  }

  // Current processing state
  Stage currentStage;
  size_t currentIndex;

  // Input data
  CudaBuffer inputBuffer;

  // Output buffers for each stage
  std::vector<CudaBuffer> layerOutputBuffers;
  std::vector<std::vector<int>> outputDims;

  CudaBuffer exitOutputBuffer;
  std::vector<int> exitOutputDims;

  std::vector<float> finalOutput;

  bool isCompleted() const { return currentStage == COMPLETED; }

  std::vector<float> getResults() const
  {
    if (!isCompleted()) {
      throw std::runtime_error("Inference not completed yet");
    }
    return finalOutput;
  }
};

class AnytimeYOLO
{
public:
  AnytimeYOLO(const std::string & folderPath, bool halfPrecision = false)
  {
    this->runtime = std::unique_ptr<IRuntime>(createInferRuntime(logger));
    // Create CUDA stream
    cudaStreamCreate(&stream);

    this->halfPrecision = halfPrecision;

    std::ifstream configFile(folderPath + "/model.json");
    json config;
    configFile >> config;

    std::cout << "Loaded config file: " << folderPath + "/model.json" << std::endl;
    std::cout << "Loading layers..." << std::endl;

    for (const auto & layerConfig : config["layers"]) {
      std::cout << "Loading layer: " << layerConfig["index"] << " - " << layerConfig["type"]
                << std::endl;
      loadLayer(layerConfig, folderPath);
    }

    std::cout << "Loading exits..." << std::endl;

    for (const auto & exitConfig : config["exits"]) {
      std::cout << "Loading exit: " << exitConfig["index"] << std::endl;
      loadExit(exitConfig, folderPath);
    }

    std::cout << "Loading NMS engine..." << std::endl;

    if (!loadNMS(config["nms"], folderPath)) {
      std::cerr << "Failed to load NMS engine" << std::endl;
      throw std::runtime_error("Failed to load NMS engine");
    }

    // report lengths of layers and exits
    std::cout << "Loaded " << chunks.size() << " layers" << std::endl;
    std::cout << "Loaded " << exits.size() << " exits" << std::endl;
  }

  // Helper function to get binding information from an engine
  struct BindingInfo
  {
    std::vector<std::string> inputNames;
    std::vector<std::string> outputNames;
  };

  BindingInfo getBindingInfo(const nvinfer1::ICudaEngine * engine)
  {
    BindingInfo info;
    int numBindings = engine->getNbIOTensors();

    // Identify input and output bindings
    for (int b = 0; b < numBindings; b++) {
      const char * name = engine->getIOTensorName(b);
      if (engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
        info.inputNames.push_back(name);
      } else {
        info.outputNames.push_back(name);
      }
    }

    return info;
  }

  // Helper function to get output dimensions and allocate output buffer
  struct OutputInfo
  {
    std::vector<int> dimensions;
    size_t bufferSize;
  };

  OutputInfo getOutputInfo(const nvinfer1::ICudaEngine * engine, const std::string & outputName)
  {
    OutputInfo info;

    // Get output dimensions
    nvinfer1::Dims outDims = engine->getTensorShape(outputName.c_str());
    size_t outputSize = 1;

    for (int d = 0; d < outDims.nbDims; d++) {
      info.dimensions.push_back(outDims.d[d]);
      outputSize *= outDims.d[d];
    }

    // Get output data type
    nvinfer1::DataType outType = engine->getTensorDataType(outputName.c_str());
    size_t elementSize = 0;

    switch (outType) {
      case nvinfer1::DataType::kFLOAT:
        elementSize = sizeof(float);
        break;
      case nvinfer1::DataType::kHALF:
        elementSize = sizeof(float) / 2;
        break;
      case nvinfer1::DataType::kINT8:
        elementSize = sizeof(int8_t);
        break;
      case nvinfer1::DataType::kINT32:
        elementSize = sizeof(int32_t);
        break;
      default:
        throw std::runtime_error("Unsupported data type");
    }

    info.bufferSize = outputSize * elementSize;
    return info;
  }

  // Execute a single engine
  bool executeEngine(
    nvinfer1::IExecutionContext * context,
    const std::vector<std::pair<std::string, void *>> & inputs,
    const std::vector<std::pair<std::string, void *>> & outputs, cudaStream_t stream)
  {
    // Set input bindings
    for (const auto & input : inputs) {
      context->setTensorAddress(input.first.c_str(), input.second);
    }

    // Set output bindings
    for (const auto & output : outputs) {
      context->setTensorAddress(output.first.c_str(), output.second);
    }

    // Execute the engine
    if (!context->enqueueV3(stream)) {
      return false;
    }

    return true;
  }

  bool processChunk(
    const AnytimeYOLOChunk & chunk, const CudaBuffer & inputBuffer,
    const std::vector<CudaBuffer> & prevLayerOutputs, CudaBuffer & outputBuffer,
    std::vector<int> & outputDims, cudaStream_t stream,
    bool isLayer = true  // true for Layer, false for Exit
  )
  {
    // std::cout << "Running " << (isLayer ? "layer " : "exit ") <<
    // block.index
    //   << (isLayer ? " (" + block.type + ")" : "") << std::endl;

    // Get binding information
    auto bindingInfo = getBindingInfo(chunk.engine.get());

    // Prepare inputs
    std::vector<std::pair<std::string, void *>> inputs;
    for (size_t j = 0; j < chunk.f.size(); j++) {
      void * inputPtr;
      if (isLayer) {
        int f = chunk.f[j];
        if (f == -1) {
          if (chunk.index == 0) {
            inputPtr = inputBuffer.getDevicePtr();
          } else {
            inputPtr = prevLayerOutputs[chunk.index - 1].getDevicePtr();
          }
        } else
          inputPtr = prevLayerOutputs[f].getDevicePtr();
      } else {
        inputPtr = prevLayerOutputs[chunk.f[j]].getDevicePtr();
      }

      inputs.push_back({bindingInfo.inputNames[j], inputPtr});
    }

    // Get output info
    auto outputInfo = getOutputInfo(chunk.engine.get(), bindingInfo.outputNames[0]);
    outputDims = outputInfo.dimensions;

    // Allocate output buffer
    if (!outputBuffer.allocate(outputInfo.bufferSize)) {
      std::cerr << "Failed to allocate " << (isLayer ? "layer" : "exit") << " output buffer"
                << std::endl;
      std::cerr << "Output size: " << outputInfo.bufferSize << std::endl;
      return false;
    }

    // Prepare outputs
    std::vector<std::pair<std::string, void *>> outputs = {
      {bindingInfo.outputNames[0], outputBuffer.getDevicePtr()}};

    // Execute the node
    if (!executeEngine(chunk.context.get(), inputs, outputs, stream)) {
      std::cerr << (isLayer ? "Layer" : "Exit") << " execution failed" << std::endl;
      return false;
    }

    return true;
  }

  // Process NMS on exit outputs
  bool processNMS(
    const CudaBuffer & exitOutputBuffer, CudaBuffer & nmsOutputBuffer, cudaStream_t stream)
  {
    // Get binding information
    auto bindingInfo = getBindingInfo(nmsEngine.get());

    // Fixed size for NMS output (as in original code)
    size_t nmsOutputSize = 6 * 1000 * sizeof(float);  // Assuming 1000 detections
    // std::cout << "NMS output size: " << nmsOutputSize << std::endl;

    // Allocate NMS output buffer
    if (!nmsOutputBuffer.allocate(nmsOutputSize)) {
      std::cerr << "Failed to allocate NMS output buffer" << std::endl;
      return false;
    }

    // Prepare inputs and outputs
    std::vector<std::pair<std::string, void *>> inputs = {
      {bindingInfo.inputNames[0], exitOutputBuffer.getDevicePtr()}};

    std::vector<std::pair<std::string, void *>> outputs = {
      {bindingInfo.outputNames[0], nmsOutputBuffer.getDevicePtr()}};

    auto nmsContext = std::unique_ptr<IExecutionContext>(nmsEngine->createExecutionContext());
    // Execute NMS
    if (!executeEngine(nmsContext.get(), inputs, outputs, stream)) {
      std::cerr << "NMS execution failed" << std::endl;
      cudaError_t status = cudaGetLastError();
      if (status != cudaSuccess) {
        std::cout << "CUDA Error: " << cudaGetErrorString(status) << std::endl;
        return false;
      }
    }

    return true;
  }

  void dump_output(
    const CudaBuffer & outputBuffer, const std::vector<int> & outputDims, const std::string & path)
  {
    // Calculate total number of elements based on dimensions
    size_t total_elements = 1;
    for (const auto & dim : outputDims) {
      total_elements *= dim;
    }

    const size_t element_size = sizeof(__half);

    // Allocate host memory to receive data from GPU
    std::vector<__half> host_data(total_elements);

    // Copy data from device to host
    if (!outputBuffer.copyToHost(host_data.data(), total_elements * element_size)) {
      std::cerr << "Error copying data from device to host" << std::endl;
      return;
    }

    // Create JSON object
    nlohmann::json j;

    // Add dimensions
    j["dims"] = outputDims;

    // Add data type
    j["dtype"] = "float16";

    std::vector<float> host_data_float(total_elements);
    for (size_t i = 0; i < total_elements; i++) {
      host_data_float[i] = __half2float(host_data[i]);
    }

    // Add data array
    j["data"] = host_data_float;

    // Write to file
    std::ofstream file(path);
    if (file.is_open()) {
      file << j.dump(4);  // Pretty print with 4-space indentation
      file.close();
      // std::cout << "Successfully wrote tensor data to " << path
      //           << std::endl;
    } else {
      std::cerr << "Error opening file for writing: " << path << std::endl;
    }
  }

  std::vector<float> processNMSAndGetResults(const CudaBuffer & input)
  {
    std::cout << "Processing NMS" << std::endl;

    CudaBuffer nmsOutputBuffer;
    std::vector<float> finalOutput;

    if (!processNMS(input, nmsOutputBuffer, stream)) {
      throw std::runtime_error("NMS processing failed");
    }

    CHECK_CUDA(cudaStreamSynchronize(stream));

    // Calculate output size in elements
    size_t nmsOutputSize = 6 * 1000 * sizeof(float);
    size_t outputElements = nmsOutputSize / sizeof(float);

    // Copy result back to host
    finalOutput.resize(outputElements);
    if (!nmsOutputBuffer.copyToHost(finalOutput.data(), nmsOutputSize)) {
      throw std::runtime_error("Failed to copy NMS output to host");
    }

    return finalOutput;
  }

  InferenceState createInferenceState(const CudaBuffer & inputBuffer)
  {
    return InferenceState(inputBuffer, stream, halfPrecision, chunks.size());
  }

  bool inferStep(
    InferenceState & state, bool async = true, void (*callback)(void *) = nullptr,
    void * userData = nullptr)
  {
    // Check if inference is already complete
    if (state.isCompleted()) {
      return true;
    }

    switch (state.currentStage) {
      case InferenceState::LAYER_PROCESSING: {
        if (state.currentIndex < chunks.size()) {
          const auto & chunk = chunks[state.currentIndex];

          std::cout << "Processing layer " << state.currentIndex << std::endl;
          // Process this layer
          if (!processChunk(
                chunk, state.inputBuffer, state.layerOutputBuffers,
                state.layerOutputBuffers[state.currentIndex], state.outputDims[state.currentIndex],
                stream)) {
            throw std::runtime_error(
              "Layer processing failed at index " + std::to_string(state.currentIndex));
          }

          // Move to next layer
          state.currentIndex++;
        } else {
          // All layers processed, move to exit processing
          state.currentStage = InferenceState::EXIT_PROCESSING;
        }
        break;
      }

      case InferenceState::EXIT_PROCESSING: {
        std::cout << "Processing exit" << std::endl;

        // Process exit using layers output
        const auto & exit = exits[0];

        if (!processChunk(
              exit, state.inputBuffer, state.layerOutputBuffers, state.exitOutputBuffer,
              state.exitOutputDims, stream, false)) {
          throw std::runtime_error("Exit processing failed");
        }

        // Move to NMS processing
        state.currentStage = InferenceState::NMS_PROCESSING;
        break;
      }

      case InferenceState::NMS_PROCESSING: {
        CudaBuffer & input = state.exitOutputBuffer;

        // Use the new function to process NMS and get results
        state.finalOutput = processNMSAndGetResults(input);

        // Inference is complete
        state.currentStage = InferenceState::COMPLETED;
        break;
      }

      case InferenceState::COMPLETED:
        // Nothing to do
        break;
    }

    if (!async) {
      cudaStreamSynchronize(stream);
    }

    auto stream = this->stream;
    cudaLaunchHostFunc(stream, callback, userData);

    return state.isCompleted();
  }

  std::vector<float> infer(const CudaBuffer & inputBuffer)
  {
    auto state = createInferenceState(inputBuffer);
    while (!inferStep(state)) {
    }
    return finishEarly(state);
  }

  // 6 outputs, coordinates 4, confidence 1, class 1
  std::vector<float> calculateLatestExit(InferenceState & state)
  {
    int lastExit = -1;
    for (auto & exit : exits) {
      bool ready = true;
      for (auto & f : exit.f) {
        if (static_cast<size_t>(f) < state.currentIndex) {
          continue;
        }
        ready = false;
        break;
      }
      if (ready) {
        lastExit = exit.index;
        break;
      }
    }
    std::cout << "Calculating result of exit: " << lastExit << std::endl;

    if (lastExit == -1) {
      return {};  // terminated too early
    }

    // process the exit
    const auto & exit = exits[lastExit];
    if (!processChunk(
          exit, state.inputBuffer, state.layerOutputBuffers, state.exitOutputBuffer,
          state.exitOutputDims, stream, false)) {
      throw std::runtime_error("Exit processing failed");
    }

    // nms
    CudaBuffer & input = state.exitOutputBuffer;
    std::vector<float> results = processNMSAndGetResults(input);
    return results;
  }

  std::vector<float> finishEarly(InferenceState & state)
  {
    CudaBuffer nmsOutputBuffer;

    // check the farthest exit for which all inputs are ready
    int lastExit = -1;
    for (auto & exit : exits) {
      bool ready = true;
      for (auto & f : exit.f) {
        if (static_cast<size_t>(f) < state.currentIndex) {
          continue;
        }
        ready = false;
        break;
      }
      if (ready) {
        lastExit = exit.index;
        break;
      }
    }

    std::cout << "Finishing at exit: " << lastExit << std::endl;

    if (lastExit == -1) {
      return {};  // terminated too early
    }

    // process the exit
    const auto & exit = exits[lastExit];
    if (!processChunk(
          exit, state.inputBuffer, state.layerOutputBuffers, state.exitOutputBuffer,
          state.exitOutputDims, stream, false)) {
      throw std::runtime_error("Exit processing failed");
    }

    // nms
    CudaBuffer & input = state.exitOutputBuffer;

    // Use the new function to process NMS and get results
    state.finalOutput = processNMSAndGetResults(input);

    state.currentStage = InferenceState::COMPLETED;

    return state.getResults();
  }

private:
  bool halfPrecision = false;

  std::unique_ptr<IRuntime> runtime;

  std::vector<AnytimeYOLOChunk> chunks;
  std::vector<AnytimeYOLOChunk> exits;

  std::unique_ptr<ICudaEngine> nmsEngine;

  cudaStream_t stream;

  bool loadNMS(const json & nmsConfig, const std::string & folderPath)
  {
    const std::string nmsEnginePath = folderPath + "/" + nmsConfig["weights"].get<std::string>();
    if (loadEngine(nmsEnginePath, runtime, nmsEngine)) {
      std::cout << "Successfully loaded cached NMS engine from: " << nmsEnginePath << std::endl;
      return true;
    } else {
      std::cerr << "Failed to load cached NMS engine, falling back to build" << std::endl;
      buildOnnxEngine(folderPath + "/nms.onnx", nmsEnginePath);

      std::cout << "Successfully built NMS engine" << std::endl;
      if (!loadEngine(nmsEnginePath, runtime, nmsEngine)) {
        throw std::runtime_error("Failed to load NMS engine");
      }
      return true;
    }
  }

  bool loadLayer(const json & layerConfig, const std::string & folderPath)
  {
    const std::string type = layerConfig["type"];
    const int index = layerConfig["index"];
    std::vector<int> f;
    if (layerConfig["f"].is_array()) {
      for (const auto & fValue : layerConfig["f"]) {
        f.push_back(fValue);
      }
    } else {
      f.push_back(layerConfig["f"]);
    }

    const std::string weightsPath = folderPath + "/" + layerConfig["weights"].get<std::string>();

    // engine path is the same as weights path but with .engine extension
    const std::string enginePath = weightsPath.substr(0, weightsPath.find_last_of('.')) + ".engine";
    std::cout << "Engine path: " << enginePath << std::endl;

    std::unique_ptr<ICudaEngine> engine;
    if (loadEngine(enginePath, runtime, engine)) {
      std::cout << "Successfully loaded cached engine from: " << enginePath << std::endl;
    } else {
      std::cerr << "Failed to load cached engine, falling back to build" << std::endl;
      buildOnnxEngine(weightsPath, enginePath);

      std::cout << "Successfully built layer" << std::endl;

      if (!loadEngine(enginePath, runtime, engine)) {
        throw std::runtime_error("Failed to load engine");
      }
    }

    auto context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());

    std::cout << "Loaded engine for layer: " << index << std::endl;

    chunks.push_back({type, index, f, std::move(engine), std::move(context)});
    return true;
  }

  bool loadExit(const json & exitConfig, const std::string & folderPath)
  {
    const std::string type = "exit";
    const int index = exitConfig["index"];
    std::vector<int> f;
    if (exitConfig["f"].is_array()) {
      f = exitConfig["f"].get<std::vector<int>>();
    } else {
      f.push_back(exitConfig["f"]);
    }
    const std::string weightsPath = folderPath + "/" + exitConfig["weights"].get<std::string>();

    // engine path is the same as weights path but with .engine extension
    const std::string enginePath = weightsPath.substr(0, weightsPath.find_last_of('.')) + ".engine";

    std::unique_ptr<ICudaEngine> engine;
    if (loadEngine(enginePath, runtime, engine)) {
      std::cout << "Successfully loaded cached engine from: " << enginePath << std::endl;
    } else {
      std::cerr << "Failed to load cached engine, falling back to build" << std::endl;
      buildOnnxEngine(weightsPath, enginePath);

      std::cout << "Successfully built exit" << std::endl;

      if (!loadEngine(enginePath, runtime, engine)) {
        throw std::runtime_error("Failed to load engine");
      }
    }

    auto context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());

    exits.push_back({type, index, f, std::move(engine), std::move(context)});
    return true;
  }
};

// // Example usage
// int main(int argc, char *argv[]) {
//     bool halfPrecision = false;

//     AnytimeYOLO yolo("/home/vscode/workspace/weights_32");

//     // load image
//     CudaBuffer input;
//     if (!loadImageFromFile("/home/vscode/workspace/horses.jpg", input,
//                            halfPrecision)) {
//         return 1;
//     }

//     for (int i = 0; i < 5; i++) {
//         std::cout << "warmup " << i << std::endl;
//         yolo.infer(input);
//     }

//     auto start = std::chrono::high_resolution_clock::now();
//     std::vector<std::vector<float>> results;

//     for (int i = 0; i < 200; i++) {
//         auto state = yolo.createInferenceState(input);
//         // while (!yolo.inferStep(state)) {
//         //     // std::cout << "Inference step completed" << std::endl;
//         // }
//         for (int i = 0; i < 24; i++) {
//             yolo.inferStep(state);
//         }
//         results.push_back(yolo.finishEarly(state));
//     }

//     auto end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed = end - start;
//     std::cout << "Inference time (average over 1000 runs): "
//               << (elapsed.count() / 200.0) * 1000.0 << " ms" << std::endl;

//     const auto result = results[0];

//     for (size_t i = 0; i < 30; i += 6) {
//         std::cout << "Detection: " << result[i] << " " << result[i + 1] << " "
//                   << result[i + 2] << " " << result[i + 3] << " "
//                   << result[i + 4] << " " << result[i + 5] << std::endl;
//     }

//     // const auto outputs = yolo.infer(input, {3, 640, 640});

//     return 0;
// }