#include <gtest/gtest.h>
#include "SE3.hpp"

#include <type_traits>

template <typename T>
constexpr bool is_se3_instantiable = requires { typename LieGroup::SE3<T>; };

template <typename T>
constexpr bool is_se3_algb_instantiable = requires { typename LieGroup::SE3Algb<T>; };


TEST(TestSE3, Type) {
  EXPECT_TRUE(!is_se3_instantiable<int>);
  EXPECT_TRUE(is_se3_instantiable<float>);
  EXPECT_TRUE(is_se3_instantiable<double>);

  EXPECT_TRUE(!is_se3_algb_instantiable<int>);
  EXPECT_TRUE(is_se3_algb_instantiable<float>);
  EXPECT_TRUE(is_se3_algb_instantiable<double>);
}

TEST(TestSE3, Operation) {
    LieGroup::SE3_d object;
    LieGroup::SE3Algb_d delta{0.07 * Eigen::Vector<double, 6>::Ones()};

    LieGroup::SE3_d result = (object + delta);

    std::cout << result << std::endl;
    std::cout << delta << std::endl;
    std::cout << "--------------" << std::endl;
    std::cout << (result - object) << std::endl;
}