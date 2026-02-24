#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvOnnxParser.h>
#include <cuda_fp16.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <ratio>
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
      // std::cout << msg << std::endl;
    }
  }
} logger;

class CudaHostBuffer
{
public:
  CudaHostBuffer(size_t size) : host_ptr(nullptr), size(size)
  {
    // Allocate pinned memory
    cudaError_t status = cudaMallocHost(&host_ptr, size);
    if (status != cudaSuccess) {
      std::cerr << "Error allocating pinned memory: " << cudaGetErrorString(status) << std::endl;
    }
  }

  ~CudaHostBuffer() { free(); }

  CudaHostBuffer(const CudaHostBuffer & other) = delete;

  CudaHostBuffer & operator=(const CudaHostBuffer & other) = delete;

  CudaHostBuffer(CudaHostBuffer && other) noexcept : host_ptr(nullptr), size(0)
  {
    // Steal resources from other
    host_ptr = other.host_ptr;
    size = other.size;

    // Leave other in a valid but empty state
    other.host_ptr = nullptr;
    other.size = 0;
  }

  // Move assignment operator
  CudaHostBuffer & operator=(CudaHostBuffer && other) noexcept
  {
    if (this != &other) {  // Self-assignment check
      // Free existing resources
      free();

      // Steal resources from other
      host_ptr = other.host_ptr;
      size = other.size;

      // Leave other in a valid but empty state
      other.host_ptr = nullptr;
      other.size = 0;
    }
    return *this;
  }

  void * host_ptr;
  size_t size;

  void free()
  {
    if (host_ptr != nullptr) {
      cudaFreeHost(host_ptr);
      host_ptr = nullptr;
    }
    size = 0;
  }

  bool copyFromHost(const void * src, size_t copy_size)
  {
    if (host_ptr == nullptr || copy_size > size) {
      std::cerr << "Error: Invalid destination buffer or size mismatch" << std::endl;
      return false;
    }

    cudaError_t status = cudaMemcpy(host_ptr, src, copy_size, cudaMemcpyHostToHost);
    if (status != cudaSuccess) {
      std::cerr << "Error copying data to host buffer: " << cudaGetErrorString(status) << std::endl;
      return false;
    }
    return true;
  }
};

class CudaBuffer
{
public:
  CudaBuffer() : device_ptr(nullptr), size(0) {}

  ~CudaBuffer() { free(); }

  CudaBuffer(const CudaBuffer & other) = delete;

  CudaBuffer & operator=(const CudaBuffer & other) = delete;

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
    if (size == this->size) {
      return true;
    }

    free();
    this->size = size;
    cudaError_t status = cudaMalloc(&device_ptr, size);
    cudaMemset(device_ptr, 0, size);

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
    // cudaMallocHost(&h_data, size); // Allocates pinned memory

    cudaError_t status = cudaMemcpy(device_ptr, host_ptr, size, cudaMemcpyHostToDevice);
    return (status == cudaSuccess);
  }

  // Copy from host buffer to device buffer
  bool copyFromHostBuffer(const CudaHostBuffer & host_buffer)
  {
    if (host_buffer.size > this->size || device_ptr == nullptr) {
      return false;
    }
    cudaError_t status =
      cudaMemcpyAsync(device_ptr, host_buffer.host_ptr, host_buffer.size, cudaMemcpyHostToDevice);
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

  void printFirstN(size_t n, int lineBreakInterval = 0) const
  {
    if (size == 0) {
      // std::cout << "Empty buffer" << std::endl;
      return;
    }

    size_t elements = std::min(n, size / sizeof(float));
    std::vector<float> host_data(elements);
    copyToHost(host_data.data(), elements * sizeof(float));

    for (size_t i = 0; i < elements; i++) {
      // std::cout << host_data[i] << " ";

      if (lineBreakInterval > 0 && (i + 1) % lineBreakInterval == 0) {
        // std::cout << std::endl;
      }
    }
    // std::cout << std::endl;
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

cv::Mat loadImageFromFile(const std::string & imagePath)
{
  cv::Mat img = cv::imread(imagePath);
  if (img.empty()) {
    std::cerr << "Failed to load image: " << imagePath << std::endl;
    return cv::Mat();
  }
  return img;
}

bool prepareImage(const cv::Mat & img, CudaHostBuffer & buffer, bool halfPrecision)
{
  cv::Mat blob = cv::dnn::blobFromImage(
    img,                  // input image
    1.0 / 255.0,          // scale factor (normalization)
    cv::Size(640, 640),   // output size
    cv::Scalar(0, 0, 0),  // mean subtraction (none here)
    true,                 // swapRB - converts BGR to RGB
    false                 // crop - no cropping
  );

  if (halfPrecision) {
    blob.convertTo(blob, CV_16F);  // Convert to half precision
  } else {
    blob.convertTo(blob, CV_32F);  // Convert to float
  }

  const size_t data_size =
    blob.total() * blob.elemSize();  // Total number of elements * size of each element

  // check if the buffer is large enough
  if (data_size > buffer.size) {
    std::cerr << "Buffer size is not large enough" << std::endl;
    return false;
  }
  // Copy data to the buffer
  if (!buffer.copyFromHost(blob.data, data_size)) {
    std::cerr << "Error copying data to buffer" << std::endl;
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

  // std::cout << "Engine successfully serialized to: " << enginePath << std::endl;
  return true;
}

bool buildOnnxEngine(
  const std::string & onnx_path, const std::string & output_path, const bool halfPrecision = false)
{
  // setup the builder and network, then parse the model

  // std::cout << "Building engine from ONNX model: " << onnx_path << std::endl;
  if (halfPrecision) {
    // std::cout << "Using half precision" << std::endl;
  } else {
    // std::cout << "Using full precision" << std::endl;
  }

  const std::unique_ptr<IBuilder> builder(createInferBuilder(logger));

  if (!builder) {
    std::cerr << "Failed to create builder" << std::endl;
    cudaError_t status = cudaGetLastError();
    if (status != cudaSuccess) {
      std::cerr << "CUDA Error: " << cudaGetErrorString(status) << std::endl;
    }
    return false;
  }
  const std::unique_ptr<INetworkDefinition> network(builder->createNetworkV2(0));
  if (!network) {
    std::cerr << "Failed to create network definition" << std::endl;
    cudaError_t status = cudaGetLastError();
    if (status != cudaSuccess) {
      std::cerr << "CUDA Error: " << cudaGetErrorString(status) << std::endl;
    }
    return false;
  }

  const std::unique_ptr<IParser> parser(createParser(*network, logger));
  if (!parser) {
    std::cerr << "Failed to create parser" << std::endl;
    cudaError_t status = cudaGetLastError();
    if (status != cudaSuccess) {
      std::cerr << "CUDA Error: " << cudaGetErrorString(status) << std::endl;
    }
    return false;
  }

  parser->parseFromFile(onnx_path.c_str(), static_cast<int32_t>(ILogger::Severity::kWARNING));
  for (int32_t i = 0; i < parser->getNbErrors(); ++i) {
    // std::cout << parser->getError(i)->desc() << std::endl;
  }

  const std::unique_ptr<IBuilderConfig> config(builder->createBuilderConfig());
  if (!config) {
    std::cerr << "Failed to create builder config" << std::endl;
    cudaError_t status = cudaGetLastError();
    if (status != cudaSuccess) {
      std::cerr << "CUDA Error: " << cudaGetErrorString(status) << std::endl;
    }
    return false;
  }

  if (halfPrecision) {
    config->setFlag(BuilderFlag::kFP16);
  }

  std::unique_ptr<IHostMemory> serializedEngine(builder->buildSerializedNetwork(*network, *config));
  if (!serializedEngine) {
    std::cerr << "Failed to build serialized network" << std::endl;
    // print cuda errors
    cudaError_t status = cudaGetLastError();
    if (status != cudaSuccess) {
      std::cerr << "CUDA Error: " << cudaGetErrorString(status) << std::endl;
    }
    return false;
  }

  if (!saveEngineToFile(output_path, *serializedEngine)) {
    std::cerr << "Failed to save engine to file" << std::endl;
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

  // std::cout << "EngineData pointer" << engineData.get() << std::endl;

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

class Subexit
{
public:
  Subexit(std::unique_ptr<AnytimeYOLOChunk> chunk, int exit, int subexit)
  : chunk(std::move(chunk)), exit(exit), subexit(subexit)
  {
  }

  const std::unique_ptr<AnytimeYOLOChunk> chunk;
  const int exit;
  const int subexit;
};

class SubexitCombiner
{
public:
  SubexitCombiner(
    std::unique_ptr<ICudaEngine> engine, std::unique_ptr<IExecutionContext> context, int exit)
  : engine(std::move(engine)), context(std::move(context)), exit(exit)
  {
  }

  std::unique_ptr<ICudaEngine> engine;
  std::unique_ptr<IExecutionContext> context;

  const int exit;
};

class InferenceState
{
public:
  enum Stage { LAYER_PROCESSING, EXIT_PROCESSING, NMS_PROCESSING, COMPLETED };

  InferenceState(size_t chunkCount, int image_x = 640, int image_y = 640)
  : currentStage(LAYER_PROCESSING), currentIndex(0)
  {
    this->inputBuffer.allocate(image_x * image_y * 3 * sizeof(float));

    // Initialize output buffers for each layer
    layerOutputBuffers.resize(chunkCount);
    outputDims.resize(chunkCount);

    // Initialize exit layer output buffers
    // By default, we only process exit 0
    exitOutputDims.resize(1);

    // Initialize NMS output buffers
    finalOutput.resize(1);
  }

  std::vector<std::vector<int>> possibleExits = {
    {3, -1, -1}, {4, -1, -1}, {4, 5, -1},  {4, 6, -1},   {4, 6, 7},   {4, 6, 8},
    {14, 12, 9}, {15, 12, 9}, {15, 18, 9}, {15, 18, 20}, {15, 18, 21}};

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

  void restart(const CudaHostBuffer & inputBuffer)
  {
    currentStage = LAYER_PROCESSING;
    currentIndex = 0;

    if (inputBuffer.size > this->inputBuffer.getSize()) {
      this->inputBuffer.free();
      this->inputBuffer.allocate(inputBuffer.size);
    }

    if (!this->inputBuffer.copyFromHostBuffer(inputBuffer)) {
      throw std::runtime_error("Failed to copy input buffer");
    }
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

    // std::cout << "Loaded config file: " << folderPath + "/model.json" << std::endl;
    // std::cout << "Loading layers..." << std::endl;

    for (const auto & layerConfig : config["layers"]) {
      // std::cout << "Loading layer: " << layerConfig["index"] << " - " << layerConfig["type"] <<
      // std::endl;
      loadLayer(layerConfig, folderPath, false);
    }

    // std::cout << "Loading exits..." << std::endl;

    for (const auto & exitConfig : config["subexits"]) {
      // std::cout << "Loading exit: " << exitConfig["index"] << std::endl;
      loadSubexit(exitConfig, folderPath, false);
    }

    // std::cout << config["combine_subheads"].size() << " heads" << std::endl;

    for (const auto & combiner : config["combine_subheads"]) {
      loadSubexitCombiners(combiner, folderPath, false);
    }

    // std::cout << "Loading NMS engine..." << std::endl;

    if (!loadNMS(config["nms"], folderPath, false)) {
      std::cerr << "Failed to load NMS engine" << std::endl;
      throw std::runtime_error("Failed to load NMS engine");
    }

    // report lengths of layers and exits
    // std::cout << "Loaded " << chunks.size() << " layers" << std::endl;
    // std::cout << "Loaded " << subexits.size() << " exits" << std::endl;
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
    context->getEngine();

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

  std::vector<std::pair<std::string, void *>> prepareInputs(
    const AnytimeYOLOChunk & chunk, const CudaBuffer & inputBuffer,
    const std::vector<CudaBuffer> & prevLayerOutputs, const std::vector<std::string> & inputNames,
    bool isLayer)
  {
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
        } else if (f < 0) {
          // relative index
          auto relativeIndex = f + chunk.index;
          if (relativeIndex < 0 || static_cast<size_t>(relativeIndex) >= prevLayerOutputs.size()) {
            std::cerr << "Invalid relative index: " << relativeIndex << std::endl;
            throw std::runtime_error("Invalid relative index");
          }
          inputPtr = prevLayerOutputs[relativeIndex].getDevicePtr();
        } else
          inputPtr = prevLayerOutputs[f].getDevicePtr();
      } else {
        inputPtr = prevLayerOutputs[chunk.f[j]].getDevicePtr();
      }

      inputs.push_back({inputNames[j], inputPtr});
    }

    return inputs;
  }

  bool processChunk(
    const AnytimeYOLOChunk & chunk, const CudaBuffer & inputBuffer,
    const std::vector<CudaBuffer> & prevLayerOutputs, CudaBuffer & outputBuffer,
    std::vector<int> & outputDims, cudaStream_t stream,
    bool isLayer = true  // true for Layer, false for Exit
  )
  {
    // Get binding information
    auto bindingInfo = getBindingInfo(chunk.engine.get());

    // Prepare inputs
    std::vector<std::pair<std::string, void *>> inputs =
      prepareInputs(chunk, inputBuffer, prevLayerOutputs, bindingInfo.inputNames, isLayer);

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

  bool processExit(
    const InferenceState & state, const std::vector<Subexit *> & subexits,
    const std::vector<CudaBuffer> & layerOutputs, const SubexitCombiner & head,
    const CudaBuffer & inputBuffer, CudaBuffer & outputBuffer, cudaStream_t stream)
  {
    // subexit outputs
    std::vector<CudaBuffer> subexitOutputs;

    // Process each subexit
    for (const auto & subexit : subexits) {
      if (subexit == nullptr) {
        auto zeroBuffer = CudaBuffer();
        zeroBuffer.allocate(144 * 80 * 80);
        subexitOutputs.push_back(std::move(zeroBuffer));
        continue;
      }

      // std::cout << "Processing subexit " << subexit->exit << "-" << subexit->subexit << " " <<
      // subexit->chunk->f[0] << std::endl;
      const AnytimeYOLOChunk & subexitChunk = *subexit->chunk;

      // check if all inputs are ready
      for (auto & f : subexitChunk.f) {
        if (static_cast<size_t>(f) >= state.currentIndex) {
          throw std::runtime_error(
            "Subexit input " + std::to_string(f) + " not ready, but exit " +
            std::to_string(subexit->exit) + " is being processed");
        }
      }

      auto inputs = prepareInputs(
        subexitChunk, inputBuffer, layerOutputs,
        getBindingInfo(subexitChunk.engine.get()).inputNames, false);

      // Allocate output buffer
      auto outputInfo = getOutputInfo(
        subexitChunk.engine.get(), getBindingInfo(subexitChunk.engine.get()).outputNames[0]);
      CudaBuffer outputBuffer;

      if (!outputBuffer.allocate(outputInfo.bufferSize)) {
        std::cerr << "Failed to allocate output buffer for subexit" << std::endl;
        return false;
      }

      // Prepare outputs
      std::vector<std::pair<std::string, void *>> outputs = {
        {getBindingInfo(subexitChunk.engine.get()).outputNames[0], outputBuffer.getDevicePtr()}};

      if (!executeEngine(subexitChunk.context.get(), inputs, outputs, stream)) {
        std::cerr << "Subexit execution failed" << std::endl;
        return false;
      }
      subexitOutputs.push_back(std::move(outputBuffer));
    }

    auto bindingInfo = getBindingInfo(head.engine.get());
    // combine subexit outputs
    std::vector<std::pair<std::string, void *>> inputs;

    auto inputNames = bindingInfo.inputNames;
    for (size_t j = 0; j < subexitOutputs.size(); j++) {
      inputs.push_back({inputNames[j], subexitOutputs[j].getDevicePtr()});
    }

    // Allocate output buffer
    auto outputInfo =
      getOutputInfo(head.engine.get(), getBindingInfo(head.engine.get()).outputNames[0]);
    if (!outputBuffer.allocate(outputInfo.bufferSize)) {
      std::cerr << "Failed to allocate output buffer for head" << std::endl;
      return false;
    }

    // Prepare outputs
    std::vector<std::pair<std::string, void *>> outputs = {
      {getBindingInfo(head.engine.get()).outputNames[0], outputBuffer.getDevicePtr()}};

    if (!executeEngine(head.context.get(), inputs, outputs, stream)) {
      std::cerr << "Head execution failed" << std::endl;
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
    // // std::cout << "NMS output size: " << nmsOutputSize << std::endl;

    // Allocate NMS output buffer
    if (!nmsOutputBuffer.allocate(nmsOutputSize)) {
      std::cerr << "Failed to allocate NMS output buffer" << std::endl;
      return false;
    }

    auto & inputBuffer = exitOutputBuffer;

    // Prepare inputs and outputs
    std::vector<std::pair<std::string, void *>> inputs = {
      {bindingInfo.inputNames[0], inputBuffer.getDevicePtr()}};

    std::vector<std::pair<std::string, void *>> outputs = {
      {bindingInfo.outputNames[0], nmsOutputBuffer.getDevicePtr()}};

    // Execute NMS
    if (!executeEngine(nmsContext.get(), inputs, outputs, stream)) {
      std::cerr << "NMS execution failed" << std::endl;
      cudaError_t status = cudaGetLastError();
      if (status != cudaSuccess) {
        // std::cout << "CUDA Error: " << cudaGetErrorString(status) << std::endl;
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
      // // std::cout << "Successfully wrote tensor data to " << path
      //           << std::endl;
    } else {
      std::cerr << "Error opening file for writing: " << path << std::endl;
    }
  }

  std::vector<float> processNMSAndGetResults(const CudaBuffer & input)
  {
    // std::cout << "Processing NMS" << std::endl;

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

  InferenceState createInferenceState() { return InferenceState(chunks.size()); }

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

          // std::cout << "Processing layer " << state.currentIndex << std::endl;
          // Process this layer
          if (!processChunk(
                chunk, state.inputBuffer, state.layerOutputBuffers,
                state.layerOutputBuffers[state.currentIndex], state.outputDims[state.currentIndex],
                stream)) {
            throw std::runtime_error(
              "Layer processing failed at index " + std::to_string(state.currentIndex));
          }
          if (!async) {
            cudaStreamSynchronize(stream);
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
        std::lock_guard<std::mutex> lock(exitMutex);
        // std::cout << "Reached exit" << std::endl;

        // Process exit using layers output
        const std::vector<Subexit *> exitPtrs = {
          subexits[0].get(), subexits[1].get(), subexits[2].get()};
        const SubexitCombiner & head = heads[0];

        // std::cout << "Processing exit " << 0 << std::endl;

        if (!processExit(
              state, exitPtrs, state.layerOutputBuffers, head, state.inputBuffer,
              state.exitOutputBuffer, stream)) {
          throw std::runtime_error("Exit processing failed");
        }
        if (!async) {
          cudaStreamSynchronize(stream);
        }
        // Move to NMS processing
        state.currentStage = InferenceState::NMS_PROCESSING;
        break;
      }

      case InferenceState::NMS_PROCESSING: {
        std::lock_guard<std::mutex> lock(exitMutex);
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

    if (callback != nullptr) {
      cudaLaunchHostFunc(this->stream, callback, userData);
    }

    return state.isCompleted();
  }

  std::vector<float> infer(const CudaHostBuffer & inputBuffer)
  {
    // Prepare input buffer
    CudaBuffer input;
    auto state = createInferenceState();
    state.restart(inputBuffer);

    while (state.currentStage != InferenceState::NMS_PROCESSING) {
      inferStep(state);
    }
    return finishEarly(state);
  }

  std::optional<std::vector<Subexit *>> findBestExit(
    const std::vector<std::vector<int>> & possibleCombinations, int currentIndex)
  {
    // std::cout << "Looking for best exit for index " << currentIndex << std::endl;

    // Create a map for quick lookup of Subexit by f
    std::map<int, Subexit *> exitMap;
    for (const auto & subexit : subexits) {
      exitMap[subexit->chunk->f[0]] = subexit.get();
    }
    exitMap[-1] = nullptr;

    std::optional<std::vector<Subexit *>> earliestCombination;
    int earliestMaxF0 = 0;

    // Check each possible combination
    for (const auto & combination : possibleCombinations) {
      // Ensure the combination has exactly 3 elements
      if (combination.size() != 3) {
        continue;
      }

      std::vector<Subexit *> currentCombo;
      bool allValid = true;
      int maxF0 = std::numeric_limits<int>::min();

      // Process each subexit in the combination
      for (int subexitF : combination) {
        auto it = exitMap.find(subexitF);
        if (it == exitMap.end()) {  // Subexit not found
          allValid = false;
          break;
        }

        Subexit * subexitPtr = it->second;

        if (subexitPtr == nullptr) {
          currentCombo.push_back(nullptr);
          continue;
        }

        // Check if this subexit satisfies the condition
        if (subexitPtr->chunk->f[0] >= currentIndex) {
          allValid = false;
          break;
        }

        maxF0 = std::max(maxF0, subexitPtr->chunk->f[0]);
        currentCombo.push_back(subexitPtr);
      }

      // If all subexits are valid and this combination is later than previous best
      if (allValid && maxF0 > earliestMaxF0) {
        earliestMaxF0 = maxF0;
        earliestCombination = currentCombo;
      }
    }

    return earliestCombination;
  }

  std::vector<float> calculateLatestExit(InferenceState & state)
  {
    synchronize();
    CudaBuffer nmsOutputBuffer;

    if (state.currentStage == InferenceState::NMS_PROCESSING) {
      auto & input = state.exitOutputBuffer;
      auto result = processNMSAndGetResults(input);
      return result;
    }

    std::lock_guard<std::mutex> lock(exitMutex);

    // check the farthest exits for which all inputs are ready
    auto exitPtrs = findBestExit(state.possibleExits, state.currentIndex);
    if (!exitPtrs.has_value()) {
      // std::cout << "No valid exit found" << std::endl;
      return {};
    }
    // std::cout << "Found exit :";

    for (auto & subexit : exitPtrs.value()) {
      if (subexit == nullptr) {
        // std::cout << "-1 ";
      } else {
        // std::cout << subexit->chunk->f[0] << " ";
      }
    }
    // std::cout << std::endl;

    int exit = heads.size() - 1;  // last exit
    if (exitPtrs.value()[0] != nullptr) exit = exitPtrs.value()[0]->exit;
    const SubexitCombiner & head = heads[exit];

    // process the exit
    if (!processExit(
          state, exitPtrs.value(), state.layerOutputBuffers, head, state.inputBuffer,
          state.exitOutputBuffer, stream)) {
      throw std::runtime_error("Exit processing failed");
    }

    // nms
    CudaBuffer & input = state.exitOutputBuffer;

    // Use the new function to process NMS and get results
    auto result = processNMSAndGetResults(input);

    return result;
  }

  std::vector<float> finishEarly(InferenceState & state)
  {
    if (state.currentStage == InferenceState::COMPLETED) {
      return state.getResults();
    }

    auto result = calculateLatestExit(state);

    state.finalOutput = result;
    state.currentStage = InferenceState::COMPLETED;

    return state.getResults();
  }

  void synchronize() { cudaStreamSynchronize(stream); }

private:
  bool halfPrecision = false;

  std::unique_ptr<IRuntime> runtime;

  std::vector<AnytimeYOLOChunk> chunks;
  std::vector<std::unique_ptr<Subexit>> subexits;
  std::vector<SubexitCombiner> heads;

  std::unique_ptr<ICudaEngine> nmsEngine;
  std::unique_ptr<IExecutionContext> nmsContext;

  cudaStream_t stream;

  std::mutex exitMutex;

  bool loadNMS(const json & nmsConfig, const std::string & folderPath, bool halfPrecision = false)
  {
    const std::string weightsPath = folderPath + "/" + nmsConfig["weights"].get<std::string>();
    const std::string enginePath = weightsPath.substr(0, weightsPath.find_last_of('.')) +
                                   (halfPrecision ? "_fp16.engine" : ".engine");
    // std::cout << "Engine path: " << enginePath << std::endl;

    if (loadEngine(enginePath, runtime, nmsEngine)) {
      // std::cout << "Successfully loaded cached NMS engine from: " << enginePath << std::endl;
    } else {
      std::cerr << "Failed to load cached NMS engine, falling back to build" << std::endl;
      buildOnnxEngine(weightsPath, enginePath, halfPrecision);

      std::cout << "Successfully built NMS engine" << std::endl;
      if (!loadEngine(enginePath, runtime, nmsEngine)) {
        throw std::runtime_error("Failed to load NMS engine");
      }
    }
    nmsContext = std::unique_ptr<nvinfer1::IExecutionContext>(nmsEngine->createExecutionContext());

    return true;
  }

  bool loadLayer(
    const json & layerConfig, const std::string & folderPath, bool halfPrecision = false)
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
    const std::string enginePath = weightsPath.substr(0, weightsPath.find_last_of('.')) +
                                   (halfPrecision ? "_fp16.engine" : ".engine");
    // std::cout << "Engine path: " << enginePath << std::endl;

    std::unique_ptr<ICudaEngine> engine;
    if (loadEngine(enginePath, runtime, engine)) {
      // std::cout << "Successfully loaded cached engine from: " << enginePath << std::endl;
    } else {
      std::cerr << "Failed to load cached engine, falling back to build" << std::endl;
      buildOnnxEngine(weightsPath, enginePath, halfPrecision);

      std::cout << "Successfully built layer" << std::endl;

      if (!loadEngine(enginePath, runtime, engine)) {
        throw std::runtime_error("Failed to load engine");
      }
    }

    auto context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());

    // std::cout << "Loaded engine for layer: " << index << std::endl;

    chunks.push_back({type, index, f, std::move(engine), std::move(context)});
    return true;
  }

  bool loadSubexit(
    const json & exitConfig, const std::string & folderPath, bool halfPrecision = false)
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
    const std::string enginePath = weightsPath.substr(0, weightsPath.find_last_of('.')) +
                                   (halfPrecision ? "_fp16.engine" : ".engine");

    std::unique_ptr<ICudaEngine> engine;
    if (loadEngine(enginePath, runtime, engine)) {
      // std::cout << "Successfully loaded cached engine from: " << enginePath << std::endl;
    } else {
      std::cerr << "Failed to load cached engine, falling back to build" << std::endl;
      buildOnnxEngine(weightsPath, enginePath, halfPrecision);

      std::cout << "Successfully built exit" << std::endl;

      if (!loadEngine(enginePath, runtime, engine)) {
        throw std::runtime_error("Failed to load engine");
      }
    }

    auto context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());

    auto subexit = std::make_unique<Subexit>(
      std::make_unique<AnytimeYOLOChunk>(
        AnytimeYOLOChunk{type, index, f, std::move(engine), std::move(context)}),
      exitConfig["exit"], exitConfig["subexit"]);

    subexits.push_back(std::move(subexit));

    return true;
  }

public:
  bool loadSubexitCombiners(
    const json & layerConfig, const std::string & folderPath, bool halfPrecision = false)
  {
    // std::cout << "Loading subexit combiner" << std::endl;

    const std::string weightsPath = folderPath + "/" + layerConfig["weights"].get<std::string>();

    const std::string enginePath = weightsPath.substr(0, weightsPath.find_last_of('.')) +
                                   (halfPrecision ? "_fp16.engine" : ".engine");

    // std::cout << "Engine path: " << enginePath << std::endl;

    std::unique_ptr<ICudaEngine> engine;
    if (loadEngine(enginePath, runtime, engine)) {
      // std::cout << "Successfully loaded cached engine from: " << enginePath << std::endl;
    } else {
      std::cerr << "Failed to load cached engine, falling back to build" << std::endl;
      buildOnnxEngine(weightsPath, enginePath, halfPrecision);

      std::cout << "Successfully built subexit combiner" << std::endl;

      if (!loadEngine(enginePath, runtime, engine)) {
        throw std::runtime_error("Failed to load engine");
      }
    }

    auto context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());

    heads.push_back(SubexitCombiner(std::move(engine), std::move(context), layerConfig["exit"]));

    return true;
  }
};