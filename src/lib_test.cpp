#include <iostream>
#include <Eigen/Dense>
#include "../include/modern_robotics.h"
#include "gtest/gtest.h"

# define M_PI           3.14159265358979323846  /* pi */

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
    // std::cout << tmp_result << std::endl;
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
    // std::cout << tmp_result << std::endl;
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
  ASSERT_TRUE(inv.isApprox(result, 4));
}

TEST(MRTest, RotInvTest)
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
  ASSERT_TRUE(inv.isApprox(result, 4));
}

TEST(MRTest, ScrewToAxisTest)
{
  Eigen::Vector3d q, s;
  q << 3, 0, 1;
  s << 0, 0, 1;
  double h = 2;

  Eigen::VectorXd axis = mr::ScrewToAxis(q, s, h);
  Eigen::VectorXd result(6);
  result << 0, 0, 1, 0, -3, 2;

  ASSERT_TRUE(axis.isApprox(result, 4));
}

TEST(MRTest, FKInBodyTest)
{
  Eigen::MatrixXd M(4, 4);
  M <<  -1, 0,  0, 0,
         0, 1, 0, 6,
         0, 0,  -1, 2,
         0, 0,  0, 1;
  Eigen::MatrixXd Blist(6, 3);
  Blist << 0, 0, 0,
           0, 0, 0,
          -1, 0, 1,
           2, 0, 0,
           0, 1, 0,
           0, 0, 0.1;
  Eigen::VectorXd thetaList(3);
  thetaList << M_PI/2.0, 3, M_PI;

  Eigen::MatrixXd result(4, 4);
  result << 0, 1, 0, -5,
            1, 0, 0,  4,
            0, 0, -1, 1.68584073,
            0, 0,  0,          1;
  Eigen::MatrixXd FKCal = mr::FKinBody(M,Blist,thetaList);

  ASSERT_TRUE(FKCal.isApprox(result, 4));
}

TEST(MRTest, FKInSpaceTest)
{
  Eigen::MatrixXd M(4, 4);
  M <<  -1, 0,  0, 0,
         0, 1, 0, 6,
         0, 0,  -1, 2,
         0, 0,  0, 1;
  Eigen::MatrixXd Slist(6, 3);
  Slist << 0, 0, 0,
           0, 0, 0,
           1, 0, -1,
           4, 0, -6,
           0, 1, 0,
           0, 0, -0.1;
  Eigen::VectorXd thetaList(3);
  thetaList << M_PI/2.0, 3, M_PI;

  Eigen::MatrixXd result(4, 4);
  result << 0, 1, 0, -5,
            1, 0, 0,  4,
            0, 0, -1, 1.68584073,
            0, 0,  0,          1;
  Eigen::MatrixXd FKCal = mr::FKinBody(M,Slist,thetaList);

  ASSERT_TRUE(FKCal.isApprox(result, 4));
}

TEST(MRTest, AxisAng6Test) {
	Eigen::VectorXd input(6);
	Eigen::VectorXd result(7);
	input << 1.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0;
	result << 1.0, 0.0, 0.0, 1.0, 1.0, 2.0, 3.0, 1.0;

	Eigen::VectorXd output = mr::AxisAng6(input);
	ASSERT_TRUE(output.isApprox(result, 4));
}

TEST(MRTest, MatrixLog6Test) {
	Eigen::MatrixXd Tinput(4, 4);
	Eigen::MatrixXd result(4, 4);
	Tinput << 1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 3,
		0, 0, 0, 1;

	result << 0, 0, 0, 0,
		0, 0, -1.57079633, 2.35619449,
		0, 1.57079633, 0, 2.35619449,
		0, 0, 0, 0;

	Eigen::MatrixXd Toutput = mr::MatrixLog6(Tinput);
	ASSERT_TRUE(Toutput.isApprox(result, 4));
}

TEST(MRTest, DistanceToSO3Test) {
	Eigen::Matrix3d input;
	double result = 0.088353;
	input << 1.0, 0.0, 0.0,
		0.0, 0.1, -0.95,
		0.0, 1.0, 0.1;
	ASSERT_DOUBLE_EQ(result, mr::DistanceToSO3(input));
}

TEST(MRTest, DistanceToSE3Test) {
	Eigen::Matrix4d input;
	double result = 0.134931;
	input << 1.0, 0.0, 0.0, 1.2,
		0.0, 0.1, -0.95, 1.5,
		0.0, 1.0, 0.1, -0.9,
		0.0, 0.0, 0.1, 0.98;
	ASSERT_DOUBLE_EQ(result, mr::DistanceToSE3(input));
}

TEST(MRTest, TestIfSO3Test) {
	Eigen::Matrix3d input;
	bool result = false;
	input << 1.0, 0.0, 0.0,
		0.0, 0.1, -0.95,
		0.0, 1.0, 0.1;
	ASSERT_EQ(result, mr::TestIfSO3(input));
}

TEST(MRTest, TestIfSE3Test) {
	Eigen::Matrix4d input;
	bool result = false;
	input << 1.0, 0.0, 0.0, 1.2,
		0.0, 0.1, -0.95, 1.5,
		0.0, 1.0, 0.1, -0.9,
		0.0, 0.0, 0.1, 0.98;
	ASSERT_EQ(result, mr::TestIfSE3(input));
}