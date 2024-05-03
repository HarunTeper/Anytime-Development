#include "yolo_rt/yolo_model.hpp"
#include <ATen/core/interned_strings.h>
#include <nlohmann/json.hpp>
#include <regex>
#include <iostream>

using json = nlohmann::json;

std::vector<LayerSourceInfo> parse_source_layer_info(std::string path)
{
    std::ifstream file(path);
    if (!file.good()){
        throw std::runtime_error("Could not open file");
    }
    json data = json::parse(file);
    
    // // data should be a list of std::variant<int, std::vector<int>>
    std::vector<LayerSourceInfo> source_layer_infos; 
    for (const auto& item : data)
    {
        if (item.is_number())
        {
          source_layer_infos.push_back(item.get<int>());
        }
        else if (item.is_array())
        {
          source_layer_infos.push_back(item.get<std::vector<int>>());
        }
        else
        {
            throw std::runtime_error("Invalid source layer info");
        }
    }
    return source_layer_infos;
}

std::string layer_src_info_to_string(const LayerSourceInfo& from){
    std::stringstream ss;
    std::visit([&ss](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, int>)
        {
            ss << arg;
        }
        else if constexpr (std::is_same_v<T, std::vector<int>>)
        {
            ss << "[";
            for (const auto& i : arg)
            {
                ss << i << ", ";
            }
            ss << "]";
        }
    }, from);
    return ss.str();
}

int translate_index(int length, int i) {
    if (i < 0) {
        return length + i;
    }
    return i;
}



YOLOModel::YOLOModel(YOLOModelConfig config)
: stream(config.stream)
{
    std::cout << "Loading model from " << config.weights_path << std::endl;
    std::cout << config.weights_path << std::endl;
    std::cout << "/home/f110xaviernx/ros2_rt_yolo/src/yolo_rt/weights/yolov7-tiny-traced-half.pt" << std::endl;
    this->model = torch::jit::load(config.weights_path);
    std::cout << "Model loaded" << std::endl;
    this->model.to(at::kCUDA);
    std::cout << "Model transferred to CUDA device" << std::endl;
    this->model.eval();
    std::cout << "Model loaded set eval mode" << std::endl;
    auto submodules = this->model.named_modules();
    std::cout << "Found " << submodules.size() << " submodules" << std::endl;
    auto model_layers = std::vector<torch::jit::Module>();
    for (const auto& submodule : submodules)
    {
        auto name = submodule.name;
        //determine if name is of the format "model.{layer}"
        std::regex pattern(R"(model\.([0-9]+))");
        std::smatch match;
        if (std::regex_match(name, match, pattern)){
            auto layer = match[1].str();
            model_layers.push_back(submodule.value);
        }
    }
    std::cout << "Found " << model_layers.size() << " layers" << std::endl;
    auto source_layer_infos = parse_source_layer_info(config.source_info_path);

    // set layers
    this->layers = std::vector<Layer>();
    for (uint i = 0; i < model_layers.size(); i++)
    {
        this->layers.push_back({
            .id=i, 
            .source_info=source_layer_infos[i], 
            .module=model_layers[i]
        });
    }

    std::cout << "Model loaded" << std::endl;
}

torch::IValue layer_forward(Layer& layer, std::vector<torch::IValue> outputs, torch::IValue previous_output)
{
    auto source_layer_info = layer.source_info;
    torch::IValue output;
    std::visit([&previous_output, &outputs, &output, &layer](auto&& arg) {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, int>)
        {
            torch::IValue input;
            if (arg == -1) {
                input = previous_output;
            }
            else { 
                auto index = translate_index(outputs.size(), arg);
                input = outputs[index];
            }
            output = layer.module.forward({input});
        }
        else if constexpr (std::is_same_v<T, std::vector<int>>)
        {
            std::vector<torch::IValue> inputs;
            for (const auto& i : arg)
            {
                if (i == -1) {
                    inputs.push_back(previous_output);
                }
                else {
                    auto index = translate_index(outputs.size(), i);
                    inputs.push_back(outputs[index]);
                }
            }
            output = layer.module.forward(inputs);
        }
    }, source_layer_info);
    return output;
}

torch::IValue YOLOModel::forward_full(torch::Tensor x){
    std::vector<torch::IValue> outputs;
    torch::IValue previous_output = x;
    
    int i = 0;
    // std::cout << "Running model with " << this->layers.size() << " layers" << std::endl;
    for (auto& layer : this->layers)
    {
        // std::cout << "Running layer " << i << " with source info " << layer_src_info_to_string(layer.source_info) << std::endl;
        auto output = layer_forward(layer, outputs, previous_output);
        previous_output = output;
        outputs.push_back(output);
        i++;
        this->stream.synchronize();
    }
    return outputs[outputs.size() - 1];
}

void YOLOModel::forward_full_at_once(torch::Tensor x){
    this->model.forward({x});
    // sync stream
    this->stream.synchronize();
}

void YOLOModel::forward_init(torch::Tensor x)
{
    this->outputs.clear();
    this->previous_layer_output = x;
}

bool YOLOModel::forward_one()
{
    if (this->stream.query()){
        if (this->outputs.size() == this->layers.size()){
            return true;
        }
        auto& layer = this->layers[this->outputs.size()];
        // std::cout << "Running layer " << layer.id << " with source info " << layer_src_info_to_string(layer.source_info) << std::endl;
        auto output = layer_forward(layer, this->outputs, this->previous_layer_output);
        this->outputs.push_back(output);
        this->previous_layer_output = output;
    }
    return false;
}

bool YOLOModel::forward_one_blocking()
{
    if (this->outputs.size() == this->layers.size()){
        return true;
    }
    auto& layer = this->layers[this->outputs.size()];
    // std::cout << "Running layer " << layer.id << " with source info " << layer_src_info_to_string(layer.source_info) << std::endl;
    auto output = layer_forward(layer, this->outputs, this->previous_layer_output);
    this->outputs.push_back(output);
    this->previous_layer_output = output;

    if (this->outputs.size() % 6 == 0){
        this->stream.synchronize();
    }
    return false;
}