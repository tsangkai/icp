
#pragma once
#include "Util.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <concepts>

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
template <std::floating_point T> inline SE3Algb<T> Log(const SE3<T> &m);

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

    SE3 operator*(const SE3 &other) const;

    friend std::ostream &operator<< <>(std::ostream &os, const SE3<T> &obj);

  private:
    DataType data_;
};

template <std::floating_point T> class SE3Algb {
    using DataType = Eigen::Vector<T, 6>;

  public:
    SE3Algb() : data_{} {}
    SE3Algb(DataType data) : data_{std::move(data)} {}

    friend SE3<T> Exp<>(const SE3Algb<T> &tau);

    friend std::ostream &operator<< <>(std::ostream &os, const SE3Algb<T> &obj);

  private:
    DataType data_;
};

using SE3_d = SE3<double>;
using SE3Algb_d = SE3Algb<double>;

/*******
 *  Implementation
 */

template <std::floating_point T> SE3<T>::SE3() : data_{} {
    data_.setIdentity();
}

template <std::floating_point T>
SE3<T>::SE3(DataType data) : data_{std::move(data)} {}

template <std::floating_point T>
SE3<T>::SE3(Eigen::Matrix<T, 3, 3> rotation, Eigen::Vector<T, 3> translation)
    : data_{} {
    data_.linear() = rotation;
    data_.translation() = translation;
}

template <std::floating_point T>
SE3<T> SE3<T>::operator*(const SE3<T> &other) const {
    SE3 obj;
    obj.data_ = this->data_ * other.data_;
    return obj;
}

template <std::floating_point T>
std::ostream &operator<<(std::ostream &os, const SE3<T> &obj) {
    os << obj.data_.matrix();
    return os;
}

/// Implementation for the Lie albegra: se(3)

template <std::floating_point T> inline SE3<T> Exp(const SE3Algb<T> &tau) {
    auto const &rho = tau.data_.template head<3>();
    auto const &theta = tau.data_.template tail<3>();

    return {Exp(theta.eval()), V(theta.eval()) * rho};
}

template <std::floating_point T>
inline SE3<T> operator+(const SE3<T> &a, const SE3Algb<T> &b) {
    return a * Exp(b);
};

template <std::floating_point T>
std::ostream &operator<<(std::ostream &os, const SE3Algb<T> &obj) {
    os << obj.data_;
    return os;
}
