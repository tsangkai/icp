

/// Implementation for the Lie group: SE(3)

inline Eigen::Matrix3d V(const Eigen::Vector3d &vec) {
    auto const norm = vec.norm();
    auto const firstOrderCoeff = (1.0 - std::cos(norm)) / std::pow(norm, 2);
    auto const secondOrderCoeff = (norm - std::sin(norm)) / std::pow(norm, 3);

    return Eigen::Matrix3d::Identity() + firstOrderCoeff * cross(vec) +
           secondOrderCoeff * cross(vec) * cross(vec);
}

/**
 * @brief
 *  (146)
 * @param vec
 * @return Eigen::Matrix3d
 */
inline Eigen::Matrix3d invV(const Eigen::Vector3d &vec) {
    auto const norm = vec.norm();
    auto const secondOrderCoeff =
        1.0 / (norm * norm) -
        (1.0 + std::cos(norm) / (2 * norm * std::sin(norm)));

    return Eigen::Matrix3d::Identity() - 0.5 * cross(vec) +
           secondOrderCoeff * cross(vec) * cross(vec);
}

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

/// Implementation for the Lie algebra: se(3)

template <std::floating_point T> inline SE3<T> Exp(const SE3Algb<T> &tau) {
    auto const &rho = tau.data_.template head<3>();
    auto const &theta = tau.data_.template tail<3>();

    return {Exp(SO3Algb<T>{theta}).data().toRotationMatrix(), V(theta) * rho};
}

template <std::floating_point T> inline SE3Algb<T> Log(const SE3<T> &M) {
    auto const theta = Log(SO3<T>{Eigen::Quaternion<T>{M.data().rotation()}});

    auto tau = SE3Algb<T>{};
    tau.data_.template head<3>() = invV(theta.data()) * M.data().translation();
    tau.data_.template tail<3>() = theta.data();

    return tau;
}

template <std::floating_point T>
inline SE3<T> operator+(const SE3<T> &a, const SE3Algb<T> &b) {
    return a * Exp(b);
};

template <std::floating_point T>
inline SE3Algb<T> operator-(const SE3<T> &m1, const SE3<T> &m2) {
    return Log(m2.inv() * m1);
}

template <std::floating_point T>
inline Eigen::Matrix<T, 3, 6> jacobbian_Mp_to_M(const SE3<T> &m,
                                                const Eigen::Vector<T, 3> &p) {
    auto const &R = m.data().rotation();
    auto jacobbian = Eigen::Matrix<T, 3, 6>::Zero().eval();
    jacobbian.template block<3, 3>(0, 0) = R;
    jacobbian.template block<3, 3>(0, 3) = -R * cross(p);

    return jacobbian;
}

template <std::floating_point T>
std::ostream &operator<<(std::ostream &os, const SE3Algb<T> &obj) {
    os << obj.data_;
    return os;
}
