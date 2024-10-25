#include <ATen/core/ivalue.h>
#include <torch/script.h>
#include <string>
#include <variant>

#include "waitable_test/cuda_waitable.hpp"

using LayerSourceInfo = std::variant<int, std::vector<int>>;

struct YOLOModelConfig {
  const std::string weights_path;
  const std::string source_info_path;
  const torch::Stream stream;
};

struct Layer {
  int id;
  LayerSourceInfo source_info;
  torch::jit::Module module;
};

class YOLOModel {
 private:
  torch::jit::Module model;
  std::vector<Layer> layers = {};

  torch::Stream stream;

  torch::IValue previous_layer_output;

 public:
  std::vector<torch::IValue> outputs = {};

  std::function<void()> callback;

  torch::IValue forward_full(torch::Tensor x);
  void forward_full_at_once(torch::Tensor x);

  void forward_init(torch::Tensor x);
  bool forward_one();
  bool forward_one_blocking();
  bool forward_one_callback();

  YOLOModel(YOLOModelConfig config);
  ~YOLOModel(){};

  std::shared_ptr<CudaWaitable> cuda_waitable_;

  void set_waitable(std::shared_ptr<CudaWaitable> waitable);
};