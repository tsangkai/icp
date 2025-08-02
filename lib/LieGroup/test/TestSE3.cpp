#include <gtest/gtest.h>
#include "SE3.hpp"

#include <type_traits>

template <typename T>
constexpr bool is_se3_instantiable = requires { typename SE3<T>; };

template <typename T>
constexpr bool is_se3_algb_instantiable = requires { typename SE3Algb<T>; };


TEST(TestSE3, Type) {
  EXPECT_TRUE(!is_se3_instantiable<int>);
  EXPECT_TRUE(is_se3_instantiable<float>);
  EXPECT_TRUE(is_se3_instantiable<double>);

  EXPECT_TRUE(!is_se3_algb_instantiable<int>);
  EXPECT_TRUE(is_se3_algb_instantiable<float>);
  EXPECT_TRUE(is_se3_algb_instantiable<double>);
}

TEST(TestSE3, Operation) {
    SE3_d object;
    SE3Algb_d delta{7 * Eigen::Vector<double, 6>::Ones()};

    std::cout << (object + delta) << std::endl;
    std::cout << delta << std::endl;
}