#include "anytime_yolo/yolo.hpp"

#include <filesystem>
#include <iostream>
#include <string>

int main(int argc, char * argv[])
{
  if (argc < 2) {
    std::cerr << "Usage: warmup_engines <weights_path> [--force-rebuild]" << std::endl;
    return 1;
  }

  const std::string weights_path = argv[1];
  bool force_rebuild = false;

  for (int i = 2; i < argc; i++) {
    if (std::string(argv[i]) == "--force-rebuild") {
      force_rebuild = true;
    }
  }

  if (force_rebuild) {
    std::cout << "Force rebuild: removing cached .engine files..." << std::endl;
    for (const auto & entry : std::filesystem::directory_iterator(weights_path)) {
      if (entry.path().extension() == ".engine") {
        std::filesystem::remove(entry.path());
        std::cout << "  Removed: " << entry.path().filename() << std::endl;
      }
    }
    std::filesystem::remove(weights_path + "/.gpu_fingerprint");
  }

  std::cout << "Building/loading TensorRT engines from: " << weights_path << std::endl;

  try {
    AnytimeYOLO yolo(weights_path, false);
    std::cout << "All engines ready." << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "Engine warmup failed: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
