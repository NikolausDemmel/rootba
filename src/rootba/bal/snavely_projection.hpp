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

#include "rootba/util/eigen_types.hpp"

namespace rootba {

// //////////////////////////////////////////////////////////////////////////
// This is an example of a camera model class that is compatible with
// Eigen::Map, which can be useful for optimization with Ceres, for example. It
// is currently NOT used in the code. Instead, we use the basalt::BalCamera
// everywhere, to ensure maximum comparability between solvers (it's advantage
// over SnavelyCameraModel below is that it implements analytic Jacobians).
// //////////////////////////////////////////////////////////////////////////

// Snavley camera model (for 3D point P in camera frame)
//
// p  = [P / P.z]_xy    (perspective division)
// p' =  f * r(p) * p   (conversion to pixel coordinates)
// r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
//
// Note: In the perspective projection, we don't have the "minus"
// like in the original Snavely model. For the camera frame we assume the
// positive z axis pointing forward in view direction and in the image, y is
// poiting down, x to the right. In the original BAL formulation, the camera
// points in negative z axis, y is up in the image. Thus when loading the data,
// we invert the y and z camera axes (y also in the image).

// projection free function
template <class T>
Eigen::Matrix<T, 2, 1> project_snavely(const Eigen::Matrix<T, 3, 1>& p_cam,
                                       const T& f, const T& k1, const T& k2) {
  Eigen::Matrix<T, 2, 1> p_proj = p_cam.template head<2>() / p_cam.z();
  const auto r2 = p_proj.squaredNorm();
  const auto scale = 1.0 + r2 * (k1 + r2 * k2);
  p_proj = f * scale * p_proj;
  return p_proj;
}

// forward declaration for traits
template <class Scalar_, int OPTIONS_ = 0>
class SnavelyCameraModel;

}  // namespace rootba

namespace Eigen::internal {

template <class Scalar_, int OPTIONS_>
struct traits<rootba::SnavelyCameraModel<Scalar_, OPTIONS_>> {
  static constexpr int NUM_PARAMETERS = 3;
  static constexpr int OPTIONS = OPTIONS_;
  using Scalar = Scalar_;
  using ParamType = Eigen::Matrix<Scalar, NUM_PARAMETERS, 1, OPTIONS>;
};

template <class Scalar_, int OPTIONS_>
struct traits<Map<rootba::SnavelyCameraModel<Scalar_>, OPTIONS_>> {
  static constexpr int NUM_PARAMETERS = 3;
  static constexpr int OPTIONS = OPTIONS_;
  using Scalar = Scalar_;
  using ParamType = Map<Eigen::Matrix<Scalar, NUM_PARAMETERS, 1>, OPTIONS>;
};

template <class Scalar_, int OPTIONS_>
struct traits<Map<const rootba::SnavelyCameraModel<Scalar_>, OPTIONS_>> {
  static constexpr int NUM_PARAMETERS = 3;
  static constexpr int OPTIONS = OPTIONS_;
  using Scalar = Scalar_;
  using ParamType =
      const Map<const Eigen::Matrix<Scalar, NUM_PARAMETERS, 1>, OPTIONS>;
};

}  // namespace Eigen::internal

namespace rootba {

// intrinsics base class
template <class Derived>
class SnavelyCameraModelBase {
 public:
  static constexpr int NUM_PARAMETERS =
      Eigen::internal::traits<Derived>::NUM_PARAMETERS;
  static constexpr int OPTIONS = Eigen::internal::traits<Derived>::OPTIONS;
  using Scalar = typename Eigen::internal::traits<Derived>::Scalar;
  using ParamType = typename Eigen::internal::traits<Derived>::ParamType;

  static constexpr bool is_const = std::is_const_v<ParamType>;

  template <typename U>
  using enable_for_const = std::enable_if_t<is_const, U>;

  template <typename U>
  using disable_for_const = typename std::enable_if<!is_const, U>::type;

  /// For projection the return type is determined with the
  /// ScalarBinaryOpTraits feature of Eigen. This allows mixing concrete and Map
  /// types, as well as other compatible scalar types such as Ceres::Jet and
  /// double scalars.
  template <typename OtherScalar>
  using ReturnScalar =
      typename Eigen::ScalarBinaryOpTraits<Scalar, OtherScalar>::ReturnType;

  /// Returns copy of instance casted to NewScalarType.
  template <class NewScalarType>
  SnavelyCameraModel<NewScalarType> cast() const {
    return SnavelyCameraModel<NewScalarType>(params());
  }

  /// Data access, e.g. for optimization
  template <class S = Scalar>
  disable_for_const<S*> data() {
    return params().data();
  }
  Scalar const* data() const { return params().data(); }

  /// Assignment-like operator from OtherDerived.
  ///
  template <class OtherDerived>
  SnavelyCameraModelBase<Derived>& operator=(
      const SnavelyCameraModelBase<OtherDerived>& other) {
    params() = other.params();
    return *this;
  }

  /// Internal parameters
  const ParamType& params() const {
    return static_cast<const Derived*>(this)->params();
  }

  // named accessors
  Scalar f() const { return params()(0); }
  Scalar k1() const { return params()(1); }
  Scalar k2() const { return params()(2); }
  template <class S = Scalar>
  disable_for_const<S&> f() {
    return params()(0);
  }
  template <class S = Scalar>
  disable_for_const<S&> k1() {
    return params()(1);
  }
  template <class S = Scalar>
  disable_for_const<S&> k2() {
    return params()(2);
  }

  /// Projection
  template <class OtherScalar>
  Eigen::Matrix<ReturnScalar<OtherScalar>, 2, 1> project(
      const Eigen::Matrix<OtherScalar, 3, 1>& p_cam) const {
    auto p_proj = (p_cam.template head<2>() / p_cam.z()).eval();
    const auto r2 = p_proj.squaredNorm();
    const auto scale = Scalar(1.0) + r2 * (k1() + r2 * k2());
    p_proj = f() * scale * p_proj;
    return p_proj;
  }

 protected:
  /// no public constructor, since we are not allowed to use the base class
  /// directly, only via a derived class
  SnavelyCameraModelBase() = default;

 private:
  /// Mutator of internal parameter
  template <class P = ParamType>
  disable_for_const<P&> params() {
    return static_cast<Derived*>(this)->params();
  }
};

/// SnavelyCameraModel using  default storage; derived from
/// SnavelyCameraModelBase.
template <class Scalar_, int OPTIONS_>
class SnavelyCameraModel
    : public SnavelyCameraModelBase<SnavelyCameraModel<Scalar_, OPTIONS_>> {
 public:
  using Base = SnavelyCameraModelBase<SnavelyCameraModel<Scalar_, OPTIONS_>>;
  static int constexpr NUM_PARAMETERS = Base::NUM_PARAMETERS;
  static int constexpr OPTIONS = Base::OPTIONS;

  using Scalar = Scalar_;
  using ParamType = typename Base::ParamType;

  /// ``Base`` is friend so params can be accessed from ``Base``.
  friend class SnavelyCameraModelBase<SnavelyCameraModel<Scalar_, OPTIONS>>;

  /// Default constructor
  SnavelyCameraModel() = default;

  /// Copy constructor
  SnavelyCameraModel(SnavelyCameraModel const& other) = default;

  /// Copy-like constructor from OtherDerived.
  template <class OtherDerived>
  SnavelyCameraModel(SnavelyCameraModelBase<OtherDerived> const& other)
      : params_(other.params()) {}

  /// Accessor of params
  const ParamType& params() const { return params_; }

  /// Mutator of params
  ParamType& params() { return params_; }

 protected:
  ParamType params_;
};

}  // namespace rootba

namespace Eigen {

/// Specialization of Eigen::Map for ``SnavelyCameraModel``; derived from
/// SnavelyCameraModelBase.
///
/// Allows us to wrap SnavelyCameraModel objects around POD array (e.g. external
/// c style double array).
template <class Scalar_, int OPTIONS_>
class Map<rootba::SnavelyCameraModel<Scalar_>, OPTIONS_>
    : public rootba::SnavelyCameraModelBase<
          Map<rootba::SnavelyCameraModel<Scalar_>, OPTIONS_>> {
 public:
  using Base = rootba::SnavelyCameraModelBase<
      Map<rootba::SnavelyCameraModel<Scalar_>, OPTIONS_>>;
  using Scalar = Scalar_;
  using ParamType = typename Base::ParamType;

  /// ``Base`` is friend so params can be accessed from ``Base``.
  friend class rootba::SnavelyCameraModelBase<
      Map<rootba::SnavelyCameraModel<Scalar_>, OPTIONS_>>;

  using Base::operator=;

  Map(Scalar* coeffs) : params_(coeffs) {}

  const ParamType& params() const { return params_; }

  ParamType& params() { return params_; }

 protected:
  ParamType params_;
};

/// Specialization of Eigen::Map for ``const SnavelyCameraModel``; derived from
/// SnavelyCameraModelBase.
///
/// Allows us to wrap SnavelyCameraModel objects around POD array (e.g. external
/// c style double arrays).
template <class Scalar_, int OPTIONS_>
class Map<const rootba::SnavelyCameraModel<Scalar_>, OPTIONS_>
    : public rootba::SnavelyCameraModelBase<
          Map<const rootba::SnavelyCameraModel<Scalar_>, OPTIONS_>> {
 public:
  using Base = rootba::SnavelyCameraModelBase<
      Map<rootba::SnavelyCameraModel<Scalar_> const, OPTIONS_>>;
  using Scalar = Scalar_;
  using ParamType = typename Base::ParamType;

  Map(const Scalar* coeffs) : params_(coeffs) {}

  const ParamType& params() const { return params_; }

  // no mutator, since it's const map

 protected:
  const ParamType params_;
};

}  // namespace Eigen
