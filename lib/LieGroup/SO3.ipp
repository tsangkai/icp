

/// Implementation for the Lie group: SO(3)

template <std::floating_point T> SO3<T>::SO3() : data_{} {
    data_.setIdentity();
}

template <std::floating_point T>
SO3<T>::SO3(DataType data) : data_{std::move(data)} {}

template <std::floating_point T>
SO3<T>::SO3(Eigen::Matrix<T, 3, 3> mat) : data_{mat} {}

template <std::floating_point T>
SO3<T> SO3<T>::operator*(const SO3<T> &other) const {
    SO3 obj;
    obj.data_ = this->data_ * other.data_;
    return obj;
}

template <std::floating_point T>
std::ostream &operator<<(std::ostream &os, const SO3<T> &obj) {
    os << obj.data_.toRotationMatrix();
    return os;
}

/// Implementation for the Lie algebra: so(3)

template <std::floating_point T> inline SO3<T> Exp(const SO3Algb<T> &vec) {
    auto const norm = vec.data().norm();
    auto const normedVec = (vec.data() / norm).eval();

    return (Eigen::Matrix<T, 3, 3>::Identity() +
            std::sin(norm) * cross(normedVec) +
            (1.0 - std::cos(norm)) * cross(normedVec) * cross(normedVec))
        .eval();
}

template <std::floating_point T> inline SO3Algb<T> Log(const SO3<T> &R) {
    auto const rotMtx = R.data().toRotationMatrix();
    auto const theta = std::acos(0.5 * (rotMtx.trace() - 1));
    auto const diffR = (rotMtx - rotMtx.transpose()).eval();
    auto const veeDiffR =
        Eigen::Vector<T, 3>{diffR(2, 1), diffR(0, 2), diffR(1, 0)};

    return ((theta / (2 * std::sin(theta))) * veeDiffR).eval();
}

template <std::floating_point T>
inline SO3<T> operator+(const SO3<T> &a, const SO3Algb<T> &b) {
    return a * Exp(b);
};

template <std::floating_point T>
std::ostream &operator<<(std::ostream &os, const SO3Algb<T> &obj) {
    os << obj.data_;
    return os;
}
