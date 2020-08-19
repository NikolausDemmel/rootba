/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021, Nikolaus Demmel.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "rootba/bal/bal_problem.hpp"

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <utility>

#include <cereal/archives/binary.hpp>
#include <glog/logging.h>

#include "rootba/bal/bal_dataset_options.hpp"
#include "rootba/bal/bal_pipeline_summary.hpp"
#include "rootba/bal/bal_problem_io.hpp"
#include "rootba/util/format.hpp"
#include "rootba/util/stl_utils.hpp"
#include "rootba/util/time_utils.hpp"

namespace rootba {

namespace {  // helper

template <typename T>
void fscan_or_throw(FILE* fptr, const char* format, T* value) {
  int num_scanned = fscanf(fptr, format, value);
  if (num_scanned != 1) {
    throw std::runtime_error("");
  }
}

template <typename T, int N>
void fscan_or_throw(FILE* fptr, Eigen::Matrix<T, N, 1>& values) {
  for (int i = 0; i < values.size(); ++i) {
    fscan_or_throw(fptr, "%lf", values.data() + i);
  }
}

void readcommentline_or_throw(FILE* fptr) {
  char buffer[1000];
  bool comment_ok = false;
  while (fgets(buffer, 1000, fptr) != nullptr) {
    size_t len = strlen(buffer);

    if (len == 0) {
      throw std::runtime_error("empty line; expected comment...");
    }

    // first part of line, check # character
    if (!comment_ok) {
      if (buffer[0] == '#') {
        comment_ok = true;
      } else {
        throw std::runtime_error("non-comment line; expected comment...");
      }
    }

    // check if we reached eol
    if (buffer[len - 1] == '\n') {
      return;
    }
  }

  // fgets failed
  throw std::runtime_error("could not read comment line");
}

template <typename T, int N, class RandomEngine>
Eigen::Matrix<T, N, 1> perturbation(const T sigma, RandomEngine& eng) {
  std::normal_distribution<T> normal;
  Eigen::Matrix<T, N, 1> vec;
  vec.setZero();
  for (int i = 0; i < vec.size(); ++i) {
    vec[i] += normal(eng) * sigma;
  }
  return vec;
}

template <typename T>
T median_destructive(std::vector<T>& data) {
  int n = data.size();
  auto mid_point = data.begin() + n / 2;
  std::nth_element(data.begin(), mid_point, data.end());
  return *mid_point;
}

BalDatasetOptions::DatasetType autodetect_input_type(const std::string& path) {
  const std::string filename = std::filesystem::path(path).filename();

  if (ends_with(filename, ".cereal")) {
    return BalDatasetOptions::DatasetType::ROOTBA;
  } else if (std::string::npos != filename.find("bundle")) {
    return BalDatasetOptions::DatasetType::BUNDLER;
  } else {
    // default to BAL
    return BalDatasetOptions::DatasetType::BAL;
  }
}

class BalProblemSaver : public FileSaver<cereal::BinaryOutputArchive> {
 public:
  using Scalar = double;

  inline BalProblemSaver(std::string path,
                         const BalProblem<Scalar>& bal_problem)
      : FileSaver(BAL_PROBLEM_FILE_INFO, std::move(path)),
        bal_problem_(bal_problem) {}

 protected:
  inline bool save_impl(cereal::BinaryOutputArchive& archive) override {
    archive(bal_problem_);
    return true;
  }

  inline std::string format_summary() const override {
    return bal_problem_.stats_to_string();
  }

 private:
  const BalProblem<Scalar>& bal_problem_;
};

class BalProblemLoader : public FileLoader<cereal::BinaryInputArchive> {
 public:
  using Scalar = double;

  inline BalProblemLoader(std::string path, BalProblem<Scalar>& bal_problem)
      : FileLoader(BAL_PROBLEM_FILE_INFO, std::move(path)),
        bal_problem_(bal_problem) {}

 protected:
  inline bool load_impl() override {
    (*archive_)(bal_problem_);
    return true;
  }

  inline std::string format_summary() const override {
    return bal_problem_.stats_to_string();
  }

 private:
  BalProblem<Scalar>& bal_problem_;
};

}  // namespace

template <typename Scalar>
BalProblem<Scalar>::BalProblem(const std::string& path) {
  load_bal(path);
}

template <typename Scalar>
void BalProblem<Scalar>::load_bal(const std::string& path) {
  FILE* fptr = std::fopen(path.c_str(), "r");
  if (fptr == nullptr) {
    LOG(FATAL) << "Could not open '{}'"_format(path);
  };

  try {
    // parse header
    int num_cams;
    int num_lms;
    int num_obs;
    fscan_or_throw(fptr, "%d", &num_cams);
    fscan_or_throw(fptr, "%d", &num_lms);
    fscan_or_throw(fptr, "%d", &num_obs);
    CHECK_GT(num_cams, 0);
    CHECK_GT(num_lms, 0);
    CHECK_GT(num_obs, 0);

    // clear memory and re-allocate
    if (cameras_.capacity() > unsigned_cast(num_cams)) {
      decltype(cameras_)().swap(cameras_);
    }
    if (landmarks_.capacity() > unsigned_cast(num_lms)) {
      decltype(landmarks_)().swap(landmarks_);
    }
    cameras_.resize(num_cams);
    landmarks_.resize(num_lms);

    // parse observations
    for (int i = 0; i < num_obs; ++i) {
      int cam_idx;
      int lm_idx;
      fscan_or_throw(fptr, "%d", &cam_idx);
      fscan_or_throw(fptr, "%d", &lm_idx);
      CHECK_GE(cam_idx, 0);
      CHECK_LT(cam_idx, num_cams);
      CHECK_GE(lm_idx, 0);
      CHECK_LT(lm_idx, num_lms);

      auto [obs, inserted] = landmarks_.at(lm_idx).obs.try_emplace(cam_idx);
      CHECK(inserted) << "Invalid file '{}'"_format(path);
      Eigen::Matrix<double, 2, 1> posd;
      fscan_or_throw(fptr, posd);
      obs->second.pos = posd.cast<Scalar>();

      // For the camera frame we assume the positive z axis pointing
      // forward in view direction and in the image, y is poiting down, x to the
      // right. In the original BAL formulation, the camera points in negative z
      // axis, y is up in the image. Thus when loading the data, we invert the y
      // and z camera axes (y also in the image) in the perspective projection,
      // we don't have the "minus" like in the original Snavely model.

      // invert y axis
      obs->second.pos.y() = -obs->second.pos.y();
    }

    // invert y and z axis (same as rotation around x by 180; self-inverse)
    const SO3 axis_inversion = SO3(Vec3(1, -1, -1).asDiagonal());

    // parse camera parameters
    for (int i = 0; i < num_cams; ++i) {
      Vec9 params;
      Eigen::Matrix<double, 9, 1> paramsd;
      fscan_or_throw(fptr, paramsd);
      params = paramsd.cast<Scalar>();

      auto& cam = cameras_.at(i);
      cam.T_c_w.so3() = axis_inversion * SO3::exp(params.template head<3>());
      cam.T_c_w.translation() = axis_inversion * params.template segment<3>(3);
      cam.intrinsics = CameraModel(params.template tail<3>());
    }

    // parse landmark parameters
    for (int i = 0; i < num_lms; ++i) {
      Eigen::Matrix<double, 3, 1> p_wd;
      fscan_or_throw(fptr, p_wd);
      landmarks_.at(i).p_w = p_wd.cast<Scalar>();
    }
  } catch (const std::exception& e) {
    LOG(FATAL) << "Failed to parse '{}'"_format(path);
  }

  if (!quiet_) {
    LOG(INFO)
        << "Loaded BAL problem ({} cams, {} lms, {} obs) from '{}'"_format(
               num_cameras(), num_landmarks(), num_observations(), path);
  }

  // Current implementation uses int to compute state vector indices
  CHECK_LT(num_cameras(), std::numeric_limits<int>::max() / CAM_STATE_SIZE);

  std::fclose(fptr);
}

template <typename Scalar>
void BalProblem<Scalar>::load_bundler(const std::string& path) {
  FILE* fptr = std::fopen(path.c_str(), "r");
  if (fptr == nullptr) {
    LOG(FATAL) << "Could not open '{}'"_format(path);
  };

  try {
    // expect one comment line
    readcommentline_or_throw(fptr);
    // parse header
    int num_cams;
    int num_lms;
    fscan_or_throw(fptr, "%d", &num_cams);
    fscan_or_throw(fptr, "%d", &num_lms);
    CHECK_GT(num_cams, 0);
    CHECK_GT(num_lms, 0);

    // clear memory and re-allocate
    cameras_.clear();
    if (cameras_.capacity() > unsigned_cast(num_cams)) {
      decltype(cameras_)().swap(cameras_);
    }
    cameras_.reserve(num_cams);

    // invert y and z axis (same as rotation around x by 180; self-inverse)
    const SO3 axis_inversion = SO3(Vec3(1, -1, -1).asDiagonal());

    // not all cameras are initialized; so keep mapping from index in loaded
    // file to actual index
    std::unordered_map<int, int> cam_idx_mapping;

    // parse cameras
    for (int i = 0; i < num_cams; ++i) {
      Eigen::Matrix<double, 15, 1> paramsd;
      fscan_or_throw(fptr, paramsd);

      if (paramsd(0) == 0) {
        // focal length 0 --> assume uninitialzed camera
        continue;
      }

      Eigen::Matrix<Scalar, 15, 1> params = paramsd.cast<Scalar>();

      // remember where camera i is in the cameras_ vector
      cam_idx_mapping[i] = int(cameras_.size());

      // create camera object
      auto& cam = cameras_.emplace_back();
      cam.intrinsics = CameraModel(params.template head<3>());
      Eigen::Map<Eigen::Matrix<Scalar, 3, 3, Eigen::RowMajor>> R(params.data() +
                                                                 3);
      cam.T_c_w.so3() = axis_inversion * SO3(R);
      cam.T_c_w.translation() = axis_inversion * params.template tail<3>();
    }

    if (landmarks_.capacity() > unsigned_cast(num_lms)) {
      decltype(landmarks_)().swap(landmarks_);
    }
    landmarks_.resize(num_lms);

    // parse landmarks and observation list
    for (int i = 0; i < num_lms; ++i) {
      auto& lm = landmarks_.at(i);

      // parse 3 vector for position
      Eigen::Matrix<double, 3, 1> p_wd;
      fscan_or_throw(fptr, p_wd);
      lm.p_w = p_wd.cast<Scalar>();

      // parse and ignore 3 vector for color
      fscan_or_throw(fptr, p_wd);

      // parse view list
      int num_obs;
      fscan_or_throw(fptr, "%d", &num_obs);

      for (int j = 0; j < num_obs; ++j) {
        int cam_idx;
        int feature_idx;  // we ignore this
        fscan_or_throw(fptr, "%d", &cam_idx);
        fscan_or_throw(fptr, "%d", &feature_idx);

        Eigen::Matrix<double, 2, 1> posd;
        fscan_or_throw(fptr, posd);

        const bool cam_exists = cam_idx_mapping.count(cam_idx);
        if (cam_exists) {
          auto [obs, inserted] =
              lm.obs.try_emplace(cam_idx_mapping.at(cam_idx));
          CHECK(inserted) << "Invalid file '{}'"_format(path);
          obs->second.pos = posd.cast<Scalar>();

          // For the camera frame we assume the positive z axis pointing
          // forward in view direction and in the image, y is poiting down, x to
          // the right. In the original BAL formulation, the camera points in
          // negative z axis, y is up in the image. Thus when loading the data,
          // we invert the y and z camera axes (y also in the image) in the
          // perspective projection, we don't have the "minus" like in the
          // original Snavely model.

          // invert y axis
          obs->second.pos.y() = -obs->second.pos.y();
        }
      }
    }
  } catch (const std::exception& e) {
    LOG(FATAL) << "Failed to parse '{}'"_format(path);
  }

  if (!quiet_) {
    LOG(INFO)
        << "Loaded BAL problem ({} cams, {} lms, {} obs) from '{}'"_format(
               num_cameras(), num_landmarks(), num_observations(), path);
  }

  // Current implementation uses int to compute state vector indices
  CHECK_LT(num_cameras(), std::numeric_limits<int>::max() / CAM_STATE_SIZE);

  std::fclose(fptr);
}

template <typename Scalar>
bool BalProblem<Scalar>::load_rootba(const std::string& path) {
  if constexpr (std::is_same_v<Scalar, double>) {
    return BalProblemLoader(path, *this).load();
  } else {
    BalProblem<double> temp;
    bool res = BalProblemLoader(path, temp).load();
    *this = temp.copy_cast<Scalar>();
    return res;
  }
}

template <typename Scalar>
bool BalProblem<Scalar>::save_rootba(const std::string& path) {
  if constexpr (std::is_same_v<Scalar, double>) {
    return BalProblemSaver(path, *this).save();
  } else {
    auto temp = copy_cast<double>();
    return BalProblemSaver(path, temp).save();
  }
}

template <typename Scalar>
void BalProblem<Scalar>::normalize(const double new_scale) {
  // TODO: try out normalization mentioned in MCBA paper to see if it has
  // additional benefit on numerics (note that we already have jacobian scaling)

  // compute median point coordinates (x,y,z)
  std::vector<Scalar> tmp(num_landmarks());
  Vec3 median;
  for (int j = 0; j < 3; ++j) {
    for (int i = 0; i < num_landmarks(); ++i) {
      tmp[i] = landmarks_[i].p_w(j);
    }
    median(j) = median_destructive(tmp);
  }

  // compute median absolute deviation (l1-norm)
  for (int i = 0; i < num_landmarks(); ++i) {
    tmp[i] = (landmarks_[i].p_w - median).template lpNorm<1>();
  }
  const Scalar median_abs_deviation = median_destructive(tmp);

  // normalize scale to constant
  const Scalar scale = new_scale / median_abs_deviation;

  if (!quiet_) {
    LOG(INFO) << "Normalizing BAL problem (median: " << median.transpose()
              << ", MAD: " << median_abs_deviation << ", scale: " << scale
              << ")";
  }

  // update landmarks: X = scale * (X - median)
  for (auto& lm : landmarks_) {
    lm.p_w = scale * (lm.p_w - median);
  }

  // update cameras: center = scale * (center - median)
  for (auto& cam : cameras_) {
    SE3 T_w_c = cam.T_c_w.inverse();
    T_w_c.translation() = scale * (T_w_c.translation() - median);
    cam.T_c_w = T_w_c.inverse();
  }
}

template <typename Scalar>
void BalProblem<Scalar>::filter_obs(const double threshold) {
  CHECK_GE(threshold, 0.0);

  if (threshold > 0) {
    if (!quiet_) {
      LOG(INFO) << "Filtering observations with z < {}"_format(threshold);
    }
  } else {
    return;
  }

  // Remove observations with depth of 3D point in camera frame closer than
  // threshold.
  for (auto& lm : landmarks_) {
    for (auto it = lm.obs.cbegin(); it != lm.obs.cend();) {
      const auto& cam = cameras_.at(it->first);
      Vec3 p3d_cam = cam.T_c_w * lm.p_w;

      if (p3d_cam.z() < threshold) {
        it = lm.obs.erase(it);
      } else {
        ++it;
      }
    }
  }

  Landmarks filtered_landmarks;

  // Filter landmarks with number of observations less than 2
  std::copy_if(landmarks_.begin(), landmarks_.end(),
               std::back_inserter(filtered_landmarks),
               [](const auto& lm) { return lm.obs.size() >= 2; });

  landmarks_ = std::move(filtered_landmarks);
}

template <typename Scalar>
void BalProblem<Scalar>::perturb(double rotation_sigma,
                                 double translation_sigma,
                                 double landmark_sigma, int seed) {
  CHECK_GE(rotation_sigma, 0.0);
  CHECK_GE(translation_sigma, 0.0);
  CHECK_GE(landmark_sigma, 0.0);

  if (rotation_sigma > 0 || translation_sigma > 0 || landmark_sigma > 0) {
    if (!quiet_) {
      LOG(INFO) << "Perturbing state (seed: {}): R: {}, t: {}, p: {}"
                   ""_format(seed, rotation_sigma, translation_sigma,
                             landmark_sigma);
    }
  }

  std::random_device r;
  std::default_random_engine eng =
      seed < 0
          ? std::default_random_engine{std::random_device{}()}
          : std::default_random_engine{
                static_cast<std::default_random_engine::result_type>(seed)};

  if (rotation_sigma > 0 || translation_sigma > 0) {
    for (auto& cam : cameras_) {
      // perturb camera center in world coordinates
      if (translation_sigma > 0) {
        SE3 T_w_c = cam.T_c_w.inverse();
        T_w_c.translation() += perturbation<Scalar, 3>(translation_sigma, eng);
        cam.T_c_w = T_w_c.inverse();
      }
      // local rotation perturbation in camera frame
      if (rotation_sigma > 0) {
        cam.T_c_w.so3() =
            SO3::exp(perturbation<Scalar, 3>(rotation_sigma, eng)) *
            cam.T_c_w.so3();
      }
    }
  }

  // perturb landmarks
  if (landmark_sigma > 0) {
    for (auto& lm : landmarks_) {
      lm.p_w += perturbation<Scalar, 3>(landmark_sigma, eng);
    }
  }
}

template <class Scalar>
void BalProblem<Scalar>::postprocress(const BalDatasetOptions& options,
                                      PipelineTimingSummary* timing_summary) {
  Timer t;

  if (options.save_output) {
    save_rootba(options.output_optimized_path);
  }

  if (timing_summary) {
    timing_summary->postprocess_time = t.elapsed();
  }
}

template <typename Scalar>
void BalProblem<Scalar>::copy_to_camera_state(VecX& camera_state) const {
  CHECK_EQ(camera_state.size(), num_cameras() * CAM_STATE_SIZE);
  for (int i = 0; i < num_cameras(); ++i) {
    auto& cam = cameras_[i];
    camera_state.template segment<CAM_STATE_SIZE>(i * CAM_STATE_SIZE) =
        cam.params();
  }
}

template <typename Scalar>
void BalProblem<Scalar>::copy_from_camera_state(const VecX& camera_state) {
  CHECK_EQ(camera_state.size(), num_cameras() * CAM_STATE_SIZE);
  for (int i = 0; i < num_cameras(); ++i) {
    auto& cam = cameras_[i];
    cam.from_params(
        camera_state.template segment<CAM_STATE_SIZE>(i * CAM_STATE_SIZE));
  }
}

template <typename Scalar>
void BalProblem<Scalar>::backup() {
  for (auto& cam : cameras_) {
    cam.backup();
  }
  for (auto& lm : landmarks_) {
    lm.backup();
  }
}

template <typename Scalar>
void BalProblem<Scalar>::restore() {
  for (auto& cam : cameras_) {
    cam.restore();
  }
  for (auto& lm : landmarks_) {
    lm.restore();
  }
}

template <typename Scalar>
int BalProblem<Scalar>::num_observations() const {
  int num = 0;
  for (auto& lm : landmarks_) {
    num += lm.obs.size();
  }
  return num;
}

template <class Scalar>
void BalProblem<Scalar>::summarize_problem(DatasetSummary& summary) const {
  summary.type = "bal";
  summary.num_cameras = num_cameras();
  summary.num_landmarks = num_landmarks();
  summary.num_observations = num_observations();

  auto stats = [](const ArrXd& data) {
    DatasetSummary::Stats res;
    res.mean = data.mean();
    res.min = data.minCoeff();
    res.max = data.maxCoeff();
    res.stddev = std::sqrt((data - res.mean).square().sum() / data.size());
    return res;
  };

  // per landmark observation stats
  {
    ArrXd per_lm_obs(num_landmarks());
    for (int i = 0; i < num_landmarks(); ++i) {
      per_lm_obs(i) = landmarks_.at(i).obs.size();
    }
    summary.per_lm_obs = stats(per_lm_obs);
    CHECK_NEAR(summary.per_lm_obs.mean,
               double(num_observations()) / num_landmarks(), 1e-9);
  }

  // no per hostframe landmark stats
  summary.per_host_lms = DatasetSummary::Stats();
}

template <typename Scalar>
std::string BalProblem<Scalar>::stats_to_string() const {
  DatasetSummary summary;
  summarize_problem(summary);

  return "BAL problem stats: {} cams, {} lms, {} obs, per-lm-obs: "
         "{:.1f}+-{:.1f}/{}/{}"
         ""_format(num_cameras(), num_landmarks(), num_observations(),
                   summary.per_lm_obs.mean, summary.per_lm_obs.stddev,
                   int(summary.per_lm_obs.min), int(summary.per_lm_obs.max));
}

#ifdef ROOTBA_INSTANTIATIONS_FLOAT
template class BalProblem<float>;
#endif

#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
template class BalProblem<double>;
#endif

template <class Scalar>
BalProblem<Scalar> load_normalized_bal_problem(
    const BalDatasetOptions& options, DatasetSummary* dataset_summary,
    PipelineTimingSummary* timing_summary) {
  // random seed
  if (options.random_seed >= 0) {
    std::srand(options.random_seed);
  }

  Timer timer;

  // auto detect input type
  BalDatasetOptions::DatasetType input_type = options.input_type;
  if (BalDatasetOptions::DatasetType::AUTO == input_type) {
    input_type = autodetect_input_type(options.input);
    LOG(INFO) << "Autodetected input dataset type as {}."_format(
        wise_enum::to_string(input_type));
  }

  // load dataset as double
  BalProblem<double> bal_problem;
  bal_problem.set_quiet(options.quiet);
  switch (input_type) {
    case BalDatasetOptions::DatasetType::ROOTBA:
      bal_problem.load_rootba(options.input);
      break;
    case BalDatasetOptions::DatasetType::BAL:
      bal_problem.load_bal(options.input);
      break;
    case BalDatasetOptions::DatasetType::BUNDLER:
      bal_problem.load_bundler(options.input);
      break;
    default:
      LOG(FATAL) << "unreachable";
  }

  const double time_load = timer.reset();

  // normalize to fixed scale and center (as double, since there are some
  // overflow issues with float for large problems)
  if (options.normalize) {
    bal_problem.normalize(options.normalization_scale);
  }

  // perturb state if sigmas are positive
  bal_problem.perturb(options.rotation_sigma, options.translation_sigma,
                      options.point_sigma, options.random_seed);

  // Filter observations of points closer than threshold to the camera
  bal_problem.filter_obs(options.init_depth_threshold);

  // convert to Scalar if needed
  BalProblem<Scalar> res;
  if constexpr (std::is_same_v<Scalar, double>) {
    res = std::move(bal_problem);
  } else {
    res = bal_problem.copy_cast<Scalar>();
  }

  const double time_preprocess = timer.reset();

  if (timing_summary) {
    timing_summary->load_time = time_load;
    timing_summary->preprocess_time = time_preprocess;
  }

  if (dataset_summary) {
    dataset_summary->input_path = options.input;
    res.summarize_problem(*dataset_summary);
  }

  // print some info
  if (!options.quiet) {
    LOG(INFO) << res.stats_to_string();
  }

  return res;
}

template <class Scalar>
BalProblem<Scalar> load_normalized_bal_problem(
    const std::string& path, DatasetSummary* dataset_summary,
    PipelineTimingSummary* timing_summary) {
  BalDatasetOptions options;
  options.input = path;
  return load_normalized_bal_problem<Scalar>(options, dataset_summary,
                                             timing_summary);
}

template <class Scalar>
BalProblem<Scalar> load_normalized_bal_problem_quiet(const std::string& path) {
  BalDatasetOptions options;
  options.input = path;
  options.quiet = true;
  return load_normalized_bal_problem<Scalar>(options);
}

#ifdef ROOTBA_INSTANTIATIONS_FLOAT
template BalProblem<float> load_normalized_bal_problem<float>(
    const BalDatasetOptions& options, DatasetSummary* dataset_summary,
    PipelineTimingSummary* timing_summary);

template BalProblem<float> load_normalized_bal_problem<float>(
    const std::string& path, DatasetSummary* dataset_summary,
    PipelineTimingSummary* timing_summary);

template BalProblem<float> load_normalized_bal_problem_quiet<float>(
    const std::string& path);
#endif

#ifdef ROOTBA_INSTANTIATIONS_DOUBLE
template BalProblem<double> load_normalized_bal_problem<double>(
    const BalDatasetOptions& options, DatasetSummary* dataset_summary,
    PipelineTimingSummary* timing_summary);

template BalProblem<double> load_normalized_bal_problem<double>(
    const std::string& path, DatasetSummary* dataset_summary,
    PipelineTimingSummary* timing_summary);

template BalProblem<double> load_normalized_bal_problem_quiet<double>(
    const std::string& path);
#endif

}  // namespace rootba
