#include <iostream>
#include <Eigen/Dense>
#include "../include/modern_robotics.h"
#include "gtest/gtest.h"


TEST(MRTest, VecToSO3Test)
{
    Eigen::Vector3d vec(1,2,3);
    Eigen::Matrix3d result(3,3);
    result << 0,-3,2,3,0,-1,-2,1,0;
    EXPECT_EQ(result, mr::VecToso3(vec));
}

TEST(MRTest, JacobianSpaceTest)
{
    Eigen::MatrixXd s_list(6,3);
    s_list << 0,      0,      0,
              0,      1,     -1,
              1,      0,      0,
              0,-0.0711, 0.0711,
              0,      0,      0,
              0,      0,-0.2795;
    Eigen::VectorXd theta(3);
    theta << 1.0472, 1.0472, 1.0472;
    Eigen::MatrixXd result(6,3);
    result << 0,  -0.866,   0.866,
              0,     0.5,    -0.5,
              1,       0,       0,
              0, -0.0355, -0.0855,
              0, -0.0615, -0.1481,
              0,       0, -0.1398;
    Eigen::MatrixXd tmp_result = mr::JacobianSpace(s_list, theta);
    std::cout << tmp_result << std::endl;
    ASSERT_TRUE(mr::JacobianSpace(s_list, theta).isApprox(result,4));
}


TEST(MRTest, JacobianBodyTest)
{
    Eigen::MatrixXd b_list(6,3);
    b_list <<     0,      0,      0,
                  0,      1,     -1,
                  1,      0,      0,
             0.0425,      0,      0,
             0.5515,      0,      0,
                  0,-0.5515, 0.2720;
    Eigen::VectorXd theta(3);
    theta << 0, 0, 1.5708;
    Eigen::MatrixXd result(6,3);
    result << 1,       0,       0,
              0,       1,      -1,
              0,       0,       0,
              0, -0.2795,       0,
         0.2795,       0,       0,
        -0.0425, -0.2720,  0.2720;
    Eigen::MatrixXd tmp_result = mr::JacobianBody(b_list, theta);
    std::cout << tmp_result << std::endl;
    ASSERT_TRUE(mr::JacobianBody(b_list, theta).isApprox(result,4));
}

TEST(MRTest, adTest)
{
    Eigen::VectorXd V(6);
    V << 1, 2, 3, 4, 5, 6;

    Eigen::MatrixXd result(6,6);
    result <<   0, -3,  2,  0,  0,  0,
                3,  0, -1,  0,  0,  0,
               -2,  1,  0,  0,  0,  0,
                0, -6,  5,  0, -3,  2,
                6,  0, -4,  3,  0, -1,
               -5,  4,  0, -2,  1,  0;

    ASSERT_TRUE(mr::ad(V).isApprox(result,4));
}

TEST(MRTest, TransInvTest)
{
  Eigen::MatrixXd input(4, 4);
  input <<   1, 0,  0, 0,
              0, 0, -1, 0,
              0, 1,  0, 3,
              0, 0,  0, 1;
  Eigen::MatrixXd result(4, 4);
  result << 1,  0, 0,  0,
            0,  0, 1, -3,
            0, -1, 0,  0,
            0,  0, 0,  1;

  auto inv = mr::TransInv(input);
  ASSERT_TRUE(inv.isApprox(result, 1));
}

TEST(MRtest, RotInvTest)
{
  Eigen::MatrixXd input(3, 3);
  input <<   0, 0, 1,
             1, 0, 0,
             0, 1, 0;
  Eigen::MatrixXd result(3, 3);
  result << 0, 1, 0,
            0, 0, 1,
            1, 0, 0;

  auto inv = mr::RotInv(input);
  ASSERT_TRUE(inv.isApprox(result, 1));
}
