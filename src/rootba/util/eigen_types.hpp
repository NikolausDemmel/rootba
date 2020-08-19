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
#pragma once

#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace rootba {

using SO3d = Sophus::SO3d;
using SE3d = Sophus::SE3d;

template <typename _Scalar, int _Rows>
using Vec = Eigen::Matrix<_Scalar, _Rows, 1>;

template <typename _Scalar, int _Rows, int _Cols>
using Arr = Eigen::Array<_Scalar, _Rows, _Cols>;

template <typename _Scalar, int _Rows, int _Cols>
using Mat = Eigen::Matrix<_Scalar, _Rows, _Cols>;

using Vec1d = Eigen::Matrix<double, 1, 1>;
using Vec2d = Eigen::Matrix<double, 2, 1>;
using Vec3d = Eigen::Matrix<double, 3, 1>;
using Vec4d = Eigen::Matrix<double, 4, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec7d = Eigen::Matrix<double, 7, 1>;
using Vec9d = Eigen::Matrix<double, 9, 1>;
using VecXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

using Vec2f = Eigen::Matrix<float, 2, 1>;
using Vec3f = Eigen::Matrix<float, 3, 1>;
using Vec4f = Eigen::Matrix<float, 4, 1>;
using Vec9f = Eigen::Matrix<float, 9, 1>;
using VecXf = Eigen::Matrix<float, Eigen::Dynamic, 1>;

using Arr1d = Eigen::Matrix<double, 1, 1>;
using ArrXd = Eigen::Array<double, Eigen::Dynamic, 1>;

using Mat1d = Eigen::Matrix<double, 1, 1>;
using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat3d = Eigen::Matrix<double, 3, 3>;
using Mat4d = Eigen::Matrix<double, 4, 4>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat9d = Eigen::Matrix<double, 9, 9>;
using MatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using RowMat2d = Eigen::Matrix<double, 2, 2, Eigen::RowMajor>;
using RowMat3d = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
using RowMat4d = Eigen::Matrix<double, 4, 4, Eigen::RowMajor>;
using RowMat6d = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
using RowMat9d = Eigen::Matrix<double, 9, 9, Eigen::RowMajor>;
using RowMatXd =
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

using Mat23d = Eigen::Matrix<double, 2, 3>;
using Mat24d = Eigen::Matrix<double, 2, 4>;
using Mat26d = Eigen::Matrix<double, 2, 6>;
using Mat42d = Eigen::Matrix<double, 4, 2>;
using Mat43d = Eigen::Matrix<double, 4, 3>;
using Mat93d = Eigen::Matrix<double, 9, 3>;

}  // namespace rootba
