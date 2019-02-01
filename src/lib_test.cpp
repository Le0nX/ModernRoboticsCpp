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