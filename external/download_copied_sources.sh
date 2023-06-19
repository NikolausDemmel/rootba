#!/usr/bin/env bash

# This is here for future reference, to document which source files,
# that are included as a copy (and not e.g. submodule), are downloaded
# from where and with which version.

# paths are relative to the directory containing this script
THIS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd "$THIS_DIR"

# magic_enum
curl https://raw.githubusercontent.com/Neargye/magic_enum/v0.9.2/include/magic_enum.hpp --create-dirs -o magic_enum/magic_enum/magic_enum.hpp

# json
curl https://raw.githubusercontent.com/nlohmann/json/v3.11.2/single_include/nlohmann/json.hpp --create-dirs -o json/nlohmann/json.hpp
curl https://raw.githubusercontent.com/nlohmann/json/v3.11.2/single_include/nlohmann/json_fwd.hpp -o json/nlohmann/json_fwd.hpp

# pprint
curl https://raw.githubusercontent.com/p-ranav/pprint/0ee09c8d8a9eebc944d07ac69e3b86d41f2304df/include/pprint.hpp --create-dirs -o pprint/pprint/pprint.hpp
