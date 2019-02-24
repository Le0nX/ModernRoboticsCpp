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
	input << 1.0, 0.0, 0.0, 1.0, 2.0, 3.0;
	result << 1.0, 0.0, 0.0, 1.0, 2.0, 3.0, 1.0;

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
	EXPECT_NEAR(result, mr::DistanceToSO3(input),3);
}

TEST(MRTest, DistanceToSE3Test) {
	Eigen::Matrix4d input;
	double result = 0.134931;
	input << 1.0, 0.0, 0.0, 1.2,
		0.0, 0.1, -0.95, 1.5,
		0.0, 1.0, 0.1, -0.9,
		0.0, 0.0, 0.1, 0.98;
	EXPECT_NEAR(result, mr::DistanceToSE3(input),3);
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

TEST(MRTest, IKinBodyTest) {
	Eigen::MatrixXd BlistT(3, 6);
	BlistT << 0, 0, -1, 2, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 0.1;
	Eigen::MatrixXd Blist = BlistT.transpose();
	Eigen::Matrix4d M;
	M << -1, 0, 0, 0,
		0, 1, 0, 6,
		0, 0, -1, 2,
		0, 0, 0, 1;
	Eigen::Matrix4d T;
	T << 0, 1, 0, -5,
		1, 0, 0, 4,
		0, 0, -1, 1.6858,
		0, 0, 0, 1;
	Eigen::VectorXd thetalist(3);
	thetalist << 1.5, 2.5, 3;
	double eomg = 0.01;
	double ev = 0.001;
	bool b_result = true;
	Eigen::VectorXd theta_result(3);
	theta_result << 1.57073819, 2.999667, 3.14153913;
	bool iRet = mr::IKinBody(Blist, M, T, thetalist, eomg, ev);
	ASSERT_EQ(b_result, iRet);
	ASSERT_TRUE(thetalist.isApprox(theta_result, 4));
}

TEST(MRTest, IKinSpaceTest) {
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 0, 0, 1, 4, 0, 0,
		0, 0, 0, 0, 1, 0,
		0, 0, -1, -6, 0, -0.1;
	Eigen::MatrixXd Slist = SlistT.transpose();
	Eigen::Matrix4d M;
	M << -1, 0, 0, 0,
		0, 1, 0, 6,
		0, 0, -1, 2,
		0, 0, 0, 1;
	Eigen::Matrix4d T;
	T << 0, 1, 0, -5,
		1, 0, 0, 4,
		0, 0, -1, 1.6858,
		0, 0, 0, 1;
	Eigen::VectorXd thetalist(3);
	thetalist << 1.5, 2.5, 3;
	double eomg = 0.01;
	double ev = 0.001;
	bool b_result = true;
	Eigen::VectorXd theta_result(3);
	theta_result << 1.57073783, 2.99966384, 3.1415342;
	bool iRet = mr::IKinSpace(Slist, M, T, thetalist, eomg, ev);
	ASSERT_EQ(b_result, iRet);
	ASSERT_TRUE(thetalist.isApprox(theta_result, 4));
}

TEST(MRTest, AdjointTest) {
    Eigen::Matrix4d T;
    T << 1, 0, 0, 0,
         0, 0, -1, 0,
         0, 1, 0, 3,
       0, 0, 0, 1;
    Eigen::MatrixXd result(6,6);
    result << 
    1,     0,     0,     0,     0,     0,
    0,     0,    -1,     0,     0,     0,
    0,     1,     0,     0,     0,     0,
    0,     0,     3,     1,     0,     0,
    3,     0,     0,     0,     0,    -1,
    0,     0,     0,     0,     1,     0;

    ASSERT_TRUE(mr::Adjoint(T).isApprox(result, 4));
}

TEST(MRTest, InverseDynamicsTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd dthetalist(3);
  dthetalist << 0.1, 0.2, 0.3;
  Eigen::VectorXd ddthetalist(3);
  ddthetalist << 2, 1.5, 1;
  Eigen::VectorXd g(3);
  g << 0, 0, -9.8;
  Eigen::VectorXd Ftip(6);
  Ftip << 1, 1, 1, 1, 1, 1;

  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;

  Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.089159,
		     0, 0, 0, 1;
  Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
		     0, 1, 0, 0.13585,
		     -1, 0 ,0, 0,
		     0, 0, 0, 1;
  Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
		     0, 1, 0, -0.1197,
		     0, 0, 1, 0.395,
		     0, 0, 0, 1;
  Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.14225,
		     0, 0, 0, 1;

  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1,      0, 1,     0, 
            0, 1, 0, -0.089, 0,     0, 
            0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

  Eigen::VectorXd taulist = mr::InverseDynamics(thetalist, dthetalist, ddthetalist, g, 
                                                Ftip, Mlist, Glist, Slist);
                 
	Eigen::VectorXd result(3);
	result << 74.6962, -33.0677, -3.23057;

  ASSERT_TRUE(taulist.isApprox(result, 4));
}

TEST(MRTest, GravityForcesTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd g(3);
  g << 0, 0, -9.8;

  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;

  Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.089159,
		     0, 0, 0, 1;
  Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
		     0, 1, 0, 0.13585,
		     -1, 0 ,0, 0,
		     0, 0, 0, 1;
  Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
		     0, 1, 0, -0.1197,
		     0, 0, 1, 0.395,
		     0, 0, 0, 1;
  Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.14225,
		     0, 0, 0, 1;

  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1,      0, 1,     0, 
            0, 1, 0, -0.089, 0,     0, 
            0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

  Eigen::VectorXd grav = mr::GravityForces(thetalist, g, Mlist, Glist, Slist);
                 
	Eigen::VectorXd result(3);
	result << 28.4033, -37.6409, -5.4416;

  ASSERT_TRUE(grav.isApprox(result, 4));
}

TEST(MRTest, MassMatrixTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;

  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;

  Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.089159,
		     0, 0, 0, 1;
  Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
		     0, 1, 0, 0.13585,
		     -1, 0 ,0, 0,
		     0, 0, 0, 1;
  Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
		     0, 1, 0, -0.1197,
		     0, 0, 1, 0.395,
		     0, 0, 0, 1;
  Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.14225,
		     0, 0, 0, 1;

  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1,      0, 1,     0, 
            0, 1, 0, -0.089, 0,     0, 
            0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

  Eigen::MatrixXd M = mr::MassMatrix(thetalist, Mlist, Glist, Slist);
                 
	Eigen::MatrixXd result(3,3);
	result << 22.5433,   -0.3071,   -0.0072,
            -0.3071,    1.9685,    0.4322,
            -0.0072,    0.4322,    0.1916;

  ASSERT_TRUE(M.isApprox(result, 4));
}

TEST(MRTest, VelQuadraticForcesTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd dthetalist(3);
  dthetalist << 0.1, 0.2, 0.3;

  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;

  Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.089159,
		     0, 0, 0, 1;
  Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
		     0, 1, 0, 0.13585,
		     -1, 0 ,0, 0,
		     0, 0, 0, 1;
  Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
		     0, 1, 0, -0.1197,
		     0, 0, 1, 0.395,
		     0, 0, 0, 1;
  Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.14225,
		     0, 0, 0, 1;

  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1,      0, 1,     0, 
            0, 1, 0, -0.089, 0,     0, 
            0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

  Eigen::VectorXd c = mr::VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist);
                 
	Eigen::VectorXd result(3);
	result << 0.2645, -0.0551, -0.0069;

  ASSERT_TRUE(c.isApprox(result, 4));
}

TEST(MRTest, EndEffectorForcesTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd Ftip(6);
  Ftip << 1, 1, 1, 1, 1, 1;

  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;

  Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.089159,
		     0, 0, 0, 1;
  Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
		     0, 1, 0, 0.13585,
		     -1, 0 ,0, 0,
		     0, 0, 0, 1;
  Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
		     0, 1, 0, -0.1197,
		     0, 0, 1, 0.395,
		     0, 0, 0, 1;
  Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.14225,
		     0, 0, 0, 1;

  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1,      0, 1,     0, 
            0, 1, 0, -0.089, 0,     0, 
            0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

  Eigen::VectorXd JTFtip = mr::EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);
                 
	Eigen::VectorXd result(3);
	result << 1.4095, 1.8577, 1.3924;

  ASSERT_TRUE(JTFtip.isApprox(result, 4));
}


TEST(MRTest, ForwardDynamicsTest) {
  Eigen::VectorXd thetalist(3);
  thetalist << 0.1, 0.1, 0.1;
  Eigen::VectorXd dthetalist(3);
  dthetalist << 0.1, 0.2, 0.3;
  Eigen::VectorXd taulist(3);
  taulist << 0.5, 0.6, 0.7;
  Eigen::VectorXd g(3);
  g << 0, 0, -9.8;
  Eigen::VectorXd Ftip(6);
  Ftip << 1, 1, 1, 1, 1, 1;

  std::vector<Eigen::MatrixXd> Mlist;
  std::vector<Eigen::MatrixXd> Glist;

  Eigen::Matrix4d M01;
	M01 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.089159,
		     0, 0, 0, 1;
  Eigen::Matrix4d M12;
	M12 << 0, 0, 1, 0.28,
		     0, 1, 0, 0.13585,
		     -1, 0 ,0, 0,
		     0, 0, 0, 1;
  Eigen::Matrix4d M23;
	M23 << 1, 0, 0, 0,
		     0, 1, 0, -0.1197,
		     0, 0, 1, 0.395,
		     0, 0, 0, 1;
  Eigen::Matrix4d M34;
	M34 << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1, 0.14225,
		     0, 0, 0, 1;

  Mlist.push_back(M01);
  Mlist.push_back(M12);
  Mlist.push_back(M23);
  Mlist.push_back(M34);
  
  Eigen::VectorXd G1(6);
  G1 << 0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7;
  Eigen::VectorXd G2(6);
  G2 << 0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393;
  Eigen::VectorXd G3(6);
  G3 << 0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275;

  Glist.push_back(G1.asDiagonal());
  Glist.push_back(G2.asDiagonal());
  Glist.push_back(G3.asDiagonal());
  
	Eigen::MatrixXd SlistT(3, 6);
	SlistT << 1, 0, 1,      0, 1,     0, 
            0, 1, 0, -0.089, 0,     0, 
            0, 1, 0, -0.089, 0, 0.425;
	Eigen::MatrixXd Slist = SlistT.transpose();

  Eigen::VectorXd ddthetalist = mr::ForwardDynamics(thetalist, dthetalist, taulist, g, 
                                                Ftip, Mlist, Glist, Slist);
                 
	Eigen::VectorXd result(3);
	result << -0.9739, 25.5847, -32.9150;

  ASSERT_TRUE(ddthetalist.isApprox(result, 4));
}