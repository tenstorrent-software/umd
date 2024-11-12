/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <filesystem>
#include <iostream>
#include <string>

namespace tt::umd::utils {

std::string get_abs_path(std::string path) {
    // Note that __FILE__ might be resolved at compile time to an absolute or relative address, depending on the
    // compiler.
    return std::filesystem::absolute(path).string();
    // std::filesystem::path current_file_path = std::filesystem::absolute(__FILE__);
    // std::filesystem::path umd_root = current_file_path.parent_path().parent_path();
    // std::filesystem::path abs_path = umd_root / path;
    // return abs_path.string();
}

}  // namespace tt::umd::utils
