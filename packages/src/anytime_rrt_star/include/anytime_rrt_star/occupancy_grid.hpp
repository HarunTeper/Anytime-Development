// Copyright 2025 Anytime System
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ANYTIME_RRT_STAR__OCCUPANCY_GRID_HPP_
#define ANYTIME_RRT_STAR__OCCUPANCY_GRID_HPP_

#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct Point2D
{
  double x = 0.0;
  double y = 0.0;
};

class OccupancyGrid
{
public:
  OccupancyGrid() = default;

  bool loadFromYaml(const std::string & yaml_path)
  {
    // Parse the simple flat YAML (Nav2 map format)
    std::string image_file;
    std::string mode = "trinary";
    double origin_yaw = 0.0;
    int negate = 0;

    std::ifstream yaml_in(yaml_path);
    if (!yaml_in.is_open()) {
      throw std::runtime_error("Cannot open YAML file: " + yaml_path);
    }

    std::string line;
    while (std::getline(yaml_in, line)) {
      // Skip empty lines and comments
      if (line.empty() || line[0] == '#') {continue;}

      auto colon_pos = line.find(':');
      if (colon_pos == std::string::npos) {continue;}

      std::string key = line.substr(0, colon_pos);
      std::string value = line.substr(colon_pos + 1);

      // Trim whitespace
      while (!key.empty() && (key.front() == ' ' || key.front() == '\t')) {key.erase(0, 1);}
      while (!key.empty() && (key.back() == ' ' || key.back() == '\t')) {key.pop_back();}
      while (!value.empty() && (value.front() == ' ' || value.front() == '\t')) {
        value.erase(0, 1);
      }
      while (!value.empty() && (value.back() == ' ' || value.back() == '\t')) {value.pop_back();}

      if (key == "image") {
        image_file = value;
      } else if (key == "resolution") {
        resolution_ = std::stod(value);
      } else if (key == "origin") {
        // Parse [x, y, yaw] format
        auto bracket_start = value.find('[');
        auto bracket_end = value.find(']');
        if (bracket_start != std::string::npos && bracket_end != std::string::npos) {
          std::string inner = value.substr(bracket_start + 1, bracket_end - bracket_start - 1);
          std::istringstream ss(inner);
          std::string token;
          int idx = 0;
          while (std::getline(ss, token, ',')) {
            // Trim whitespace from token
            while (!token.empty() && token.front() == ' ') {token.erase(0, 1);}
            while (!token.empty() && token.back() == ' ') {token.pop_back();}
            double val = std::stod(token);
            if (idx == 0) {
              origin_x_ = val;
            } else if (idx == 1) {
              origin_y_ = val;
            } else if (idx == 2) {
              origin_yaw = val;
            }
            idx++;
          }
        }
      } else if (key == "occupied_thresh") {
        occupied_thresh_ = std::stod(value);
      } else if (key == "free_thresh") {
        free_thresh_ = std::stod(value);
      } else if (key == "negate") {
        negate = std::stoi(value);
      } else if (key == "mode") {
        mode = value;
      }
    }
    yaml_in.close();
    (void)origin_yaw;
    (void)mode;
    negate_ = (negate != 0);

    // Resolve image path relative to YAML directory
    std::string yaml_dir;
    auto last_slash = yaml_path.find_last_of('/');
    if (last_slash != std::string::npos) {
      yaml_dir = yaml_path.substr(0, last_slash + 1);
    }
    std::string pgm_path = yaml_dir + image_file;

    // Load PGM file (P5 binary format)
    return loadPgm(pgm_path);
  }

  bool isFree(double world_x, double world_y) const
  {
    int gx, gy;
    if (!worldToGrid(world_x, world_y, gx, gy)) {
      return false;  // out of bounds = not free
    }
    uint8_t pixel = getPixel(gx, gy);
    double normalized = static_cast<double>(pixel) / 255.0;
    if (negate_) {
      normalized = 1.0 - normalized;
    }
    // In Nav2 trinary mode:
    // normalized > occupied_thresh => occupied
    // normalized < free_thresh => free
    // Between => unknown (treat as occupied)
    //
    // But note: PGM convention for Nav2 maps is that high pixel values (white/205=0.804)
    // represent free space, and low values (black/0) represent obstacles.
    // After normalization: free pixels have high values, occupied have low values.
    // The thresholds in the YAML use "occupancy probability":
    //   occupancy = (255 - pixel) / 255 when negate=0
    // So we need to convert pixel to occupancy probability first.
    double occ_prob = (255.0 - static_cast<double>(pixel)) / 255.0;
    if (negate_) {
      occ_prob = static_cast<double>(pixel) / 255.0;
    }
    return occ_prob < free_thresh_;
  }

  bool isEdgeFree(const Point2D & from, const Point2D & to) const
  {
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 1e-9) {
      return isFree(from.x, from.y);
    }

    // Sample at half-resolution intervals
    double step = resolution_ * 0.5;
    int num_steps = static_cast<int>(std::ceil(dist / step));
    if (num_steps < 1) {num_steps = 1;}

    for (int i = 0; i <= num_steps; ++i) {
      double t = static_cast<double>(i) / static_cast<double>(num_steps);
      double wx = from.x + t * dx;
      double wy = from.y + t * dy;
      if (!isFree(wx, wy)) {
        return false;
      }
    }
    return true;
  }

  double getMinX() const {return origin_x_;}
  double getMaxX() const {return origin_x_ + width_ * resolution_;}
  double getMinY() const {return origin_y_;}
  double getMaxY() const {return origin_y_ + height_ * resolution_;}
  double getResolution() const {return resolution_;}
  int getWidth() const {return width_;}
  int getHeight() const {return height_;}
  bool isLoaded() const {return !data_.empty();}

  double computeFreeArea() const
  {
    int free_count = 0;
    for (int gy = 0; gy < height_; ++gy) {
      for (int gx = 0; gx < width_; ++gx) {
        uint8_t pixel = getPixel(gx, gy);
        double occ_prob = (255.0 - static_cast<double>(pixel)) / 255.0;
        if (negate_) {
          occ_prob = static_cast<double>(pixel) / 255.0;
        }
        if (occ_prob < free_thresh_) {
          free_count++;
        }
      }
    }
    return free_count * resolution_ * resolution_;
  }

private:
  std::vector<uint8_t> data_;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.05;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double occupied_thresh_ = 0.65;
  double free_thresh_ = 0.25;
  bool negate_ = false;

  bool worldToGrid(double wx, double wy, int & gx, int & gy) const
  {
    gx = static_cast<int>(std::floor((wx - origin_x_) / resolution_));
    gy = static_cast<int>(std::floor((wy - origin_y_) / resolution_));
    return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
  }

  uint8_t getPixel(int gx, int gy) const
  {
    // PGM stores rows top-to-bottom, but map origin is bottom-left
    int row = (height_ - 1) - gy;
    return data_[row * width_ + gx];
  }

  bool loadPgm(const std::string & pgm_path)
  {
    std::ifstream file(pgm_path, std::ios::binary);
    if (!file.is_open()) {
      throw std::runtime_error("Cannot open PGM file: " + pgm_path);
    }

    // Read magic number
    std::string magic;
    file >> magic;
    if (magic != "P5") {
      throw std::runtime_error("Not a P5 PGM file: " + pgm_path);
    }

    // Skip comments
    char c;
    file.get(c);  // consume whitespace after magic
    while (file.peek() == '#') {
      std::string comment;
      std::getline(file, comment);
    }

    // Read width, height, maxval
    int maxval;
    file >> width_ >> height_ >> maxval;
    file.get(c);  // consume single whitespace byte before pixel data

    if (width_ <= 0 || height_ <= 0 || maxval <= 0) {
      throw std::runtime_error("Invalid PGM header in: " + pgm_path);
    }

    // Read pixel data
    data_.resize(width_ * height_);
    file.read(reinterpret_cast<char *>(data_.data()), width_ * height_);

    if (!file) {
      throw std::runtime_error("Failed to read pixel data from: " + pgm_path);
    }

    return true;
  }
};

#endif  // ANYTIME_RRT_STAR__OCCUPANCY_GRID_HPP_
