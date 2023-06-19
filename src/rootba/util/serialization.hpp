/**
BSD 3-Clause License

This file is part of the RootBA project.
https://github.com/NikolausDemmel/rootba

Copyright (c) 2021-2023, Nikolaus Demmel.
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
#pragma once

#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>
#include <glog/logging.h>

#include "rootba/util/format.hpp"
#include "rootba/util/time_utils.hpp"

namespace rootba {

struct FileInfo {
  std::string type;
  std::string version;

  template <class Archive>
  void serialize(Archive& ar) {
    ar(CEREAL_NVP(type));
    ar(CEREAL_NVP(version));
  }

  bool operator==(const FileInfo& other) const {
    return type == other.type && version == other.version;
  }
};

}  // namespace rootba

namespace fmt {

template <>
struct formatter<rootba::FileInfo> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const rootba::FileInfo& info, FormatContext& ctx) {
    return format_to(ctx.begin(), "{}@{}", info.type, info.version);
  }
};

}  // namespace fmt

namespace rootba {

template <class Archive>
void save_file_info(Archive& archive, const FileInfo& info) {
  archive(cereal::make_nvp("file_info", info));
}

template <class Archive>
FileInfo load_file_info(Archive& archive) {
  FileInfo info;
  archive(cereal::make_nvp("file_info", info));
  return info;
}

template <class Archive>
bool load_and_check_file_info(Archive& archive, const FileInfo& expected) {
  FileInfo loaded;
  archive(cereal::make_nvp("file_info", loaded));
  if (expected == loaded) {
    return true;
  }

  if (loaded.type != expected.type) {
    LOG(ERROR) << "Loaded file of type {} version {}, which does not match "
                  "expected {} version {}"
                  ""_format(loaded.type, loaded.version, expected.type,
                            expected.version);

  } else if (loaded.version != expected.version) {
    LOG(ERROR) << "Loaded file of type {} with version {}, which does not "
                  "match expected version {}"
                  ""_format(loaded.type, loaded.version, expected.version);
  }
  return false;
}

template <class Archive>
std::string load_and_check_file_info_any_version(
    Archive& archive, const std::string& expected_file_type) {
  FileInfo loaded;
  archive(cereal::make_nvp("file_info", loaded));
  if (loaded.type == expected_file_type) {
    return loaded.version;
  } else {
    LOG(ERROR) << "Loaded file of type {} version {}, which does not match "
                  "expected type {}"
                  ""_format(loaded.type, loaded.version, expected_file_type);
    return "";
  }
}

template <class OutputArchive>
class FileSaver {
 public:
  virtual ~FileSaver() = default;

  virtual bool save() {
    Timer timer;

    std::ofstream os(path_, std::ios::binary);

    bool result;
    if (!os.is_open()) {
      save_error_ =
          "File could not be opened for writing."_format(info_.type, path_);
      result = false;
    } else {
      OutputArchive archive(os);
      save_file_info(archive, info_);
      result = save_impl(archive);
    }

    if (result) {
      LOG(INFO) << "Saved {} to {} ({:.3f}s): {}"_format(
          info_.type, path_, timer.elapsed(), format_summary());
      return true;
    } else {
      LOG(ERROR) << "Failed to save {} to {} ({:.3f}s): {}"_format(
          info_.type, path_, timer.elapsed(), save_error_);
      return false;
    }
  }

 protected:
  explicit FileSaver(const FileInfo& info, std::string path)
      : info_(info), path_(std::move(path)) {}

  virtual std::string format_summary() const { return ""; }

  virtual bool save_impl(OutputArchive& archive) = 0;

 protected:
  FileInfo info_;
  std::string save_error_;
  std::string path_;
};

template <class InputArchive>
class FileLoader {
 public:
  virtual ~FileLoader() = default;

  using LoaderFunctionType = std::function<bool()>;

  virtual bool load() {
    Timer timer;

    istream_ = std::make_unique<std::ifstream>(path_, std::ios::binary);

    bool result;
    std::string version_info;

    if (!istream_->is_open()) {
      load_error_ = "File could not be opened for reading."_format(
          requested_info_.type, path_);
      result = false;
    } else {
      archive_ = std::make_unique<InputArchive>(*istream_);
      loaded_info_ = load_file_info(*archive_);
      if (loaded_info_.type != requested_info_.type) {
        // file type didn't match...
        load_error_ = "Loaded file has different type {} version {}"_format(
            loaded_info_.type, loaded_info_.version);
        result = false;
      } else {
        if (loaded_info_.version == requested_info_.version) {
          // current version
          result = load_impl();
        } else {
          // legacy version
          if (legacy_loaders_.count(loaded_info_.version)) {
            version_info =
                " with legacy version {}"_format(loaded_info_.version);
            auto& loader = legacy_loaders_.at(loaded_info_.version);
            if (!loader) {
              result = false;
              load_error_ = "loading from that version not supported"_format(
                  loaded_info_.version);
            } else {
              result = (*loader)();
            }
          } else {
            result = false;
            load_error_ = "unknown version {}"_format(loaded_info_.version);
          }
        }
      }
    }

    archive_.reset();
    istream_.reset();

    if (result) {
      LOG(INFO) << "Loaded {} from {}{} ({:.3f}s): {}"
                   ""_format(requested_info_.type, path_, version_info,
                             timer.elapsed(), format_summary());
      return true;
    } else {
      LOG(ERROR) << "Failed to load {} from {}{} ({:.3f}s): {}"
                    ""_format(requested_info_.type, path_, version_info,
                              timer.elapsed(), load_error_);
      return false;
    }
  }

 protected:
  explicit FileLoader(const FileInfo& info, std::string path)
      : requested_info_(info), path_(std::move(path)) {}

  virtual bool load_impl() = 0;

  virtual std::string format_summary() const { return ""; }

  void add_legacy_version(const std::string& version,
                          std::optional<LoaderFunctionType> loader) {
    legacy_loaders_[version] = std::move(loader);
  }

 protected:
  FileInfo requested_info_;
  FileInfo loaded_info_;
  std::string path_;
  std::string load_error_;
  std::map<std::string, std::optional<LoaderFunctionType>> legacy_loaders_;
  std::unique_ptr<InputArchive> archive_;
  std::unique_ptr<std::ifstream> istream_;
};

}  // namespace rootba
