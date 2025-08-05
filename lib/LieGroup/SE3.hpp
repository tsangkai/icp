
#pragma once
#include "SO3.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <concepts>

#include "Util.hpp"

namespace LieGroup {

template <std::floating_point T> class SE3;
template <std::floating_point T> class SE3Algb;

/**
 * @brief The exponential map of the Lie algebra
 * Corresponds to (172) in [Sola, 2021]
 */
template <std::floating_point T> inline SE3<T> Exp(const SE3Algb<T> &tau);

/**
 * @brief The logarithm map of the Lie group
 * Corresponds to (173) in [Sola, 2021]
 */
template <std::floating_point T> inline SE3Algb<T> Log(const SE3<T> &M);

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
inline SE3<T> operator+(const SE3<T> &m, const SE3Algb<T> &tau);

/**
 * @brief The right-minus operator that gives m1 - m2
 * @param m1
 * @param m2
 * @return * template <std::floating_point T>
 */
template <std::floating_point T>
inline SE3Algb<T> operator-(const SE3<T> &m1, const SE3<T> &m2);

template <std::floating_point T>
std::ostream &operator<<(std::ostream &os, const SE3<T> &obj);
template <std::floating_point T>
std::ostream &operator<<(std::ostream &os, const SE3Algb<T> &obj);

/**
 * @brief The class definition of the Lie group SE(3)
 *
 * @tparam T
 */
template <std::floating_point T> class SE3 {
    /**
     * @brief The underlying data type
     *
     */
    using DataType = Eigen::Transform<T, 3, Eigen::Isometry>;

  public:
    /**
     * @brief Construct a new SE3 object
     * The default constructor will construct the identity of SE(3)
     *
     */
    SE3();
    SE3(DataType data);
    SE3(Eigen::Matrix<T, 3, 3> rotation, Eigen::Vector<T, 3> translation);

    SE3 inv() const { return data_.inverse(); }

    SE3 operator*(const SE3 &other) const;

    DataType data() const { return data_; };

    friend std::ostream &operator<< <>(std::ostream &os, const SE3<T> &obj);

  private:
    DataType data_;
};

template <std::floating_point T> class SE3Algb {
    constexpr static int Dim = 6;
    using DataType = Eigen::Vector<T, Dim>;

  public:
    SE3Algb() : data_{} {}
    SE3Algb(DataType data) : data_{std::move(data)} {}

    friend SE3<T> Exp<>(const SE3Algb<T> &tau);
    friend SE3Algb<T> Log<>(const SE3<T> &M);

    friend std::ostream &operator<< <>(std::ostream &os, const SE3Algb<T> &obj);

    DataType data() const { return data_; };

  private:
    DataType data_;
};

using SE3_d = SE3<double>;
using SE3Algb_d = SE3Algb<double>;

#include "SE3.ipp"

}    // namespace LieGroup