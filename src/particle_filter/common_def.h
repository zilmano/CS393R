//
// Created by liyanc on 10/5/20.
//

#ifndef REPO_COMMON_DEF_H
#define REPO_COMMON_DEF_H

#include <vector>
#include "xsimd/xsimd.hpp"

using simd_vec_type = std::vector<float, xsimd::aligned_allocator<float, XSIMD_DEFAULT_ALIGNMENT>>;


#endif //REPO_COMMON_DEF_H
