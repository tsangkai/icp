
#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <concepts>

#include "Util.hpp"

namespace LieGroup {

template <std::floating_point T> class SO3;
template <std::floating_point T> class SO3Algb;

/**
 * @brief The exponential map of the Lie algebra
 * Corresponds to (172) in [Sola, 2021]
 */
template <std::floating_point T> inline SO3<T> Exp(const SO3Algb<T> &vec);

/**
 * @brief The logarithm map of the Lie group
 * Corresponds to (173) in [Sola, 2021]
 */
template <std::floating_point T> inline SO3Algb<T> Log(const SO3<T> &R);

/**
 * @brief The right-plus operator that gives m + tau.
 * Corresponds to (25) in [Sola, 2021]
 *
 * @tparam T
 * @param m the Lie group object
 * @param tau the Lie algebra object
 * @return m + tau = m * Exp(tau)
 */
template <std::floating_point T>
inline SO3<T> operator+(const SO3<T> &m, const SO3Algb<T> &tau);

template <std::floating_point T>
std::ostream &operator<<(std::ostream &os, const SO3<T> &obj);
template <std::floating_point T>
std::ostream &operator<<(std::ostream &os, const SO3Algb<T> &obj);

/**
 * @brief The class definition of the Lie group SO(3)
 *
 * @tparam T
 */
template <std::floating_point T> class SO3 {
    /**
     * @brief The underlying data type
     *
     */
    using DataType = Eigen::Quaternion<T>;

  public:
    /**
     * @brief Construct a new SO3 object
     * The default constructor will construct the identity of SO(3)
     *
     */
    SO3();
    SO3(DataType data);

    SO3(Eigen::Matrix<T, 3, 3> mat);

    SO3 operator*(const SO3 &other) const;

    DataType data() const { return data_; };

    friend std::ostream &operator<< <>(std::ostream &os, const SO3<T> &obj);

  private:
    DataType data_;
};

template <std::floating_point T> class SO3Algb {
    constexpr static int Dim = 3;
    using DataType = Eigen::Vector<T, Dim>;

  public:
    SO3Algb() : data_{} {}
    SO3Algb(DataType data) : data_{std::move(data)} {}

    friend SO3<T> Exp<>(const SO3Algb<T> &tau);
    DataType data() const { return data_; };

    friend std::ostream &operator<< <>(std::ostream &os, const SO3Algb<T> &obj);

  private:
    DataType data_;
};

using SO3_d = SO3<double>;
using SO3Algb_d = SO3Algb<double>;

#include "SO3.ipp"
}    // namespace LieGroup