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

#include "rootba/options/options_interface.hpp"

#include <regex>
#include <set>

#include "rootba/util/json_utils.hpp"
#include "rootba/util/pprint_utils.hpp"
#include "rootba/util/stl_utils.hpp"
#include "rootba/util/template_utils.hpp"
#include "rootba/util/toml_utils.hpp"

namespace rootba {

// helper
namespace {

using nlohmann::json;
using nlohmann::ordered_json;

class ToJsonOptionsVisitor : public ConstOptionsVisitor {
 public:
  ToJsonOptionsVisitor() {
    root_node_ = ordered_json::object();
    current_node_stack_.push_back(&root_node_);
  }

  void operator()(const char* name, const OptionsVariableConstRef var,
                  const OptionsMeta& meta) override {
    ROOTBA_UNUSED(meta);
    auto& current_node = *current_node_stack_.back();
    std::visit([&](auto val) { current_node[name] = val.get(); }, var);
  }

  void operator()(const char* name, const OptionsInterface& child) override {
    auto& current_node = *current_node_stack_.back();
    current_node[name] = ordered_json::object();
    current_node_stack_.push_back(&current_node[name]);
    child._accept(*this);
    current_node_stack_.pop_back();
  }

  void get_result(ordered_json& result) const { result = root_node_; }

 private:
  std::vector<ordered_json*> current_node_stack_;
  ordered_json root_node_;
};

// if types are incompatible, this visitor can throw json exceptions
class FromJsonOptionsVisitor : public OptionsVisitor {
 public:
  FromJsonOptionsVisitor(const json& root) : root_node_(root) {
    // may throw json error
    current_prefix_stack_.emplace_back();
    current_node_stack_.push_back(&root_node_);
  }

  void operator()(const char* name, const OptionsVariableRef var,
                  const OptionsMeta& meta) override {
    ROOTBA_UNUSED(meta);
    const auto& current_node = *current_node_stack_.back();
    // if name exists, convert to expected type; else do nothing
    if (current_node.contains(name)) {
      std::visit(
          [&](auto val) {
            if constexpr (mp::is_instance_v<decltype(val),
                                            std::reference_wrapper>) {
              val.get() = current_node.at(name)
                              .get<std::decay_t<decltype(val.get())>>();
            } else if constexpr (std::is_same_v<decltype(val),
                                                EnumRefWrapper>) {
              const auto str = current_node.at(name).get<std::string>();
              if (!val.set(str)) {
                throw std::runtime_error(
                    "Could not convert value {} of option {} to enum {}."
                    ""_format(str, name, val.enum_name()));
              }
            } else if constexpr (std::is_same_v<decltype(val), FlagsWrapper>) {
              const auto vec =
                  current_node.at(name).get<std::vector<std::string>>();
              if (!val.set(vec)) {
                throw std::runtime_error(
                    "Could not convert value {} of option {} to enum-flags {}."
                    ""_format(pprint_compact(vec), name, val.enum_name()));
              }
            } else {
              static_assert(mp::always_false_v<decltype(val)>,
                            "non-exhaustive visitor!");
            }
            used_json_ptrs_.insert(
                (current_prefix_stack_.back() / name).to_string());
          },
          var);
    }
  }

  void operator()(const char* name, OptionsInterface& child) override {
    const auto& current_node = *current_node_stack_.back();
    // if name exists, assume it is object type and recurse into it; else do
    // nothing
    if (current_node.contains(name)) {
      current_prefix_stack_.push_back(current_prefix_stack_.back() / name);
      current_node_stack_.push_back(&current_node.at(name));
      child._accept(*this);
      current_prefix_stack_.pop_back();
      current_node_stack_.pop_back();
    }
  }

  bool check_unused(
      const json::json_pointer& root_prefix = json::json_pointer{},
      std::vector<std::string> allow_unused_top_level = {}) const {
    // if json is empty, cannot have unused entries
    if (root_node_.empty()) {
      return true;
    }

    std::vector<json::json_pointer> unused;  // unused entries
    std::set<std::string> used;       // used entries including all prefixes
    std::regex digits_regex("\\d+");  // for checking array keys
    const std::set<std::string> allowed_top_level =
        to_set(allow_unused_top_level);

    // check used and unused entries
    json flat = root_node_.flatten();
    for (const auto& kv : flat.items()) {
      std::string path = kv.key();
      json::json_pointer json_ptr(path);
      // interpret all-digits key at first level as array and remove it
      if (std::regex_match(json_ptr.back(), digits_regex)) {
        json_ptr.pop_back();
        path = json_ptr.to_string();
      }
      // check if path was used or not
      if (used_json_ptrs_.count(path)) {
        // add to used including all group-prefixes
        while (!json_ptr.empty()) {
          used.insert(json_ptr.to_string());
          json_ptr.pop_back();
        }
      } else {
        unused.emplace_back(json_ptr);
      }
    }

    // group unused
    std::map<json::json_pointer, int> unused_grouped;
    for (auto json_ptr : unused) {
      CHECK(!json_ptr.empty());
      // look for highest grouping with used parts
      bool is_group = false;
      while (true) {
        std::string part = json_ptr.back();
        json_ptr.pop_back();
        if (used.count(json_ptr.to_string()) || json_ptr.empty()) {
          json_ptr.push_back(part);
          break;
        }
        is_group = true;
      }

      // if no grouping is possible, indicate with count 0 that its a leaf entry
      if (is_group) {
        unused_grouped[json_ptr] += 1;
      } else {
        unused_grouped[json_ptr] = 0;
      }
    }

    // report unused
    for (const auto& [path, count] : unused_grouped) {
      if (count > 0) {
        if (allowed_top_level.count(path.to_string())) {
          LOG(WARNING) << "{} in config unused ({} entries)."_format(
              (root_prefix / path).to_string(), count);
        }
      } else {
        // count 0 indicates leaf node (no grouping)
        LOG(WARNING) << "{} in config unused."_format(
            (root_prefix / path).to_string());
      }
    }

    // return true if any were unused
    return unused_grouped.empty();
  }

  size_t num_loaded_items() const { return used_json_ptrs_.size(); }

 private:
  std::set<std::string> used_json_ptrs_;
  std::vector<json::json_pointer> current_prefix_stack_;
  std::vector<const json*> current_node_stack_;
  const json& root_node_;
};

}  // namespace

bool OptionsBase::_load(const nlohmann::json& json_data,
                        const LoadConfigOptions& options) {
  try {
    // descend into prefix, if exists, else load nothing
    json::json_pointer prefix_json_ptr(options.prefix);
    if (json_data.contains(prefix_json_ptr)) {
      // set from json object with prefix
      FromJsonOptionsVisitor visitor(json_data.at(prefix_json_ptr));
      _accept(visitor);
      if (options.check_unused) {
        visitor.check_unused(prefix_json_ptr, options.allow_unused_top_level);
      }
    }
  } catch (const std::exception& e) {
    LOG(ERROR) << "Error while loading options for json: " << e.what();
    return false;
  }

  return true;
}

bool OptionsBase::_load(const std::string& filename,
                        const LoadConfigOptions& options) {
  std::ifstream ifs(filename, std::ios_base::binary);

  if (!ifs.good()) {
    if (options.missing_config_file_is_error) {
      LOG(ERROR) << "Could not open config file {} for reading."_format(
          filename);
      return false;
    } else {
      LOG(INFO) << "Config file {} doesn't exist. Loading defaults."_format(
          filename);
      return true;
    }
  }

  try {
    // parse toml file
    const auto toml_data = toml::parse(ifs, filename);

    // convert to json object
    nlohmann::json json_data;
    toml_to_json(toml_data, json_data);

    // descend into prefix, if exists, else load nothing
    json::json_pointer prefix_json_ptr(options.prefix);
    if (json_data.contains(prefix_json_ptr)) {
      // set from json object with prefix
      FromJsonOptionsVisitor visitor(json_data.at(prefix_json_ptr));
      _accept(visitor);
      LOG(INFO) << "Loaded {} items from config file {}"
                   ""_format(visitor.num_loaded_items(), filename);
      if (options.check_unused) {
        visitor.check_unused(prefix_json_ptr, options.allow_unused_top_level);
      }
    } else {
      LOG(INFO) << "Loaded 0 items from config file {} "
                   "(prefix {} doesn't exist)"_format(filename, options.prefix);
    }

  } catch (const std::exception& e) {
    LOG(ERROR) << "Error while loading config file {}: {}"
                  ""_format(filename, e.what());
    return false;
  }

  return true;
}

void OptionsBase::_print(std::ostream& os) const {
  ToJsonOptionsVisitor visitor;
  _accept(visitor);
  ordered_json tmp;
  visitor.get_result(tmp);
  ordered_toml result;
  json_to_toml(tmp, result);
  os << result;
  // Alternative: Serialize json:
  // os << std::setw(4) << std::setfill(' ') << tmp;
}

std::ostream& operator<<(std::ostream& os, const OptionsInterface& options) {
  options._print(os);
  return os;
}

}  // namespace rootba
