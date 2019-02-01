#include "../include/modern_robotics.h"

/*
 * modernRobotics.cpp
 * Adapted from modern_robotics.py provided by modernrobotics.org
 * Provides useful Jacobian and frame representation functions
 */
#include <Eigen/Dense>
#include <cmath>
#include <vector>

namespace mr {
    
/* Function: Find if the value is negligible enough to consider 0
 * Inputs: value to be checked as a double
 * Returns: Boolean of true-ignore or false-can't ignore
 */
bool NearZero(const double val){
    return (std::abs(val) < .000001);
}


/* Function: Returns a normalized version of the input vector
 * Input: Eigen::MatrixXd
 * Output: Eigen::MatrixXd
 * Note: MatrixXd is used instead of VectorXd for the case of row vectors
 * 		Requires a copy
 *		Useful because of the MatrixXd casting
 */
Eigen::MatrixXd Normalize(Eigen::MatrixXd V){
    V.normalize();
    return V;
}


/* Function: Returns the skew symmetric matrix representation of an angular velocity vector
 * Input: Eigen::Vector3d 3x1 angular velocity vector
 * Returns: Eigen::MatrixXd 3x3 skew symmetric matrix
 */
Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg) {
    Eigen::Matrix3d m_ret;
    m_ret << 0, -omg(2), omg(1),
            omg(2), 0, -omg(0),
            -omg(1), omg(0), 0;
    return m_ret;
}


/* Function: Returns angular velocity vector represented by the skew symmetric matrix
 * Inputs: Eigen::MatrixXd 3x3 skew symmetric matrix
 * Returns: Eigen::Vector3d 3x1 angular velocity
 */
Eigen::Vector3d so3ToVec(const Eigen::MatrixXd& so3mat) {
    Eigen::Vector3d v_ret;
    v_ret << so3mat(2,1), so3mat(0,2), so3mat(1,0);
    return v_ret;
}


/* Function: Tranlates an exponential rotation into it's individual components
 * Inputs: Exponential rotation (rotation matrix in terms of a rotation axis
 *				and the angle of rotation)
 * Returns: The axis and angle of rotation as [x, y, z, theta]
 */
Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3){
    Eigen::Vector4d v_ret;
    v_ret << Normalize(expc3), expc3.norm();
    return v_ret;
}


/* Function: Translates an exponential rotation into a rotation matrix
 * Inputs: exponenential representation of a rotation
 * Returns: Rotation matrix
 */
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat){
    Eigen::Vector3d omgtheta = so3ToVec(so3mat);

    Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
    if( NearZero(so3mat.norm()) ) {
        return m_ret;
    }
    else {
        double theta = (AxisAng3(omgtheta))(3);
        Eigen::Matrix3d omgmat = so3mat * (1/theta);
        return m_ret + std::sin(theta) * omgmat + ( (1 - std::cos(theta)) * (omgmat * omgmat));
    }
}


/* Function: Combines a rotation matrix and position vector into a single
 * 				Special Euclidian Group (SE3) homogeneous transformation matrix
 * Inputs: Rotation Matrix (R), Position Vector (p)
 * Returns: Matrix of T = [ [R, p],
 *						    [0, 1] ]
 */
Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p){
    Eigen::MatrixXd m_ret(4,4);
    m_ret << R, p,
            0, 0, 0, 1;
    return m_ret;
}


/* Function: Separates the rotation matrix and position vector from
 *				the transfomation matrix representation
 * Inputs: Homogeneous transformation matrix
 * Returns: std::vector of [rotation matrix, position vector]
 */
std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T){
    std::vector<Eigen::MatrixXd> Rp_ret;
    Eigen::Matrix3d R_ret;
    // Get top left 3x3 corner
    R_ret = T.block<3,3>(0,0);

    Eigen::Vector3d p_ret( T(0,3), T(1,3), T(2,3) );

    Rp_ret.push_back(R_ret);
    Rp_ret.push_back(p_ret);

    return Rp_ret;
}


/* Function: Translates a spatial velocity vector into a transformation matrix
 * Inputs: Spatial velocity vector [angular velocity, linear velocity]
 * Returns: Transformation matrix
 */
Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V){
    // Separate angular (exponential representation) and linear velocities
    Eigen::Vector3d exp( V(0), V(1), V(2) );
    Eigen::Vector3d linear( V(3), V(4), V(5) );

    // Fill in values to the appropriate parts of the transformation matrix
    Eigen::MatrixXd m_ret(4,4);
    m_ret << VecToso3(exp), linear,
            0, 0, 0, 0;

    return m_ret;
}


/* Function: Provides the adjoint representation of a transformation matrix
 *			 Used to change the frame of reference for spatial velocity vectors
 * Inputs: 4x4 Transformation matrix SE(3)
 * Returns: 6x6 Adjoint Representation of the matrix
 */
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T){
    std::vector<Eigen::MatrixXd> R = TransToRp(T);
    Eigen::MatrixXd ad_ret(6,6);
    Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3,3);
    ad_ret << R[0], zeroes,
            VecToso3(R[1]) * R[0], R[0];
    return ad_ret;
}


/* Function: Rotation expanded for screw axis
 * Inputs: se3 matrix representation of exponential coordinates (transformation matrix)
 * Returns: 6x6 Matrix representing the rotation
 */
Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat){
    // Extract the angular velocity vector from the transformation matrix
    Eigen::Matrix3d se3mat_cut = se3mat.block<3,3>(0,0);
    Eigen::Vector3d omgtheta = so3ToVec(se3mat_cut);

    Eigen::MatrixXd m_ret(4,4);

    // If negligible rotation, m_Ret = [[Identity, angular velocty ]]
    //									[	0	 ,		1		   ]]
    if(NearZero(omgtheta.norm())){
        // Reuse previous variables that have our required size
        se3mat_cut = Eigen::MatrixXd::Identity(3,3);
        omgtheta << se3mat(0,3), se3mat(1,3), se3mat(2,3);
        m_ret << se3mat_cut, omgtheta,
                0, 0, 0, 1;
        return m_ret;
    }
        // If not negligible, MR page 105
    else{
        double theta = (AxisAng3(omgtheta))(3);
        Eigen::Matrix3d omgmat = se3mat.block<3,3>(0,0) / theta;
        Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3,3) * theta + (1-std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
        Eigen::Vector3d linear(se3mat(0,3), se3mat(1,3), se3mat(2,3));
        Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
        m_ret << MatrixExp3(se3mat_cut), GThetaV,
                0, 0, 0, 1;
        return m_ret;
    }

}


/* Function: Compute end effector frame (used for current spatial position calculation)
 * Inputs: Home configuration (position and orientation) of end-effector
 *		   The joint screw axes in the space frame when the manipulator
 *             is at the home position
 * 		   A list of joint coordinates.
 * Returns: Transfomation matrix representing the end-effector frame when the joints are
 *				at the specified coordinates
 * Notes: FK means Forward Kinematics
 */
Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList){
    Eigen::MatrixXd T = M;
    for(int i=(thetaList.size()-1); i>-1; i--){
        T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
    }
    return T;
}


/* Function: Gives the space Jacobian
 * Inputs: Screw axis in home position, joint configuration
 * Returns: 6xn Spatial Jacobian
 */
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetaList) {
    Eigen::MatrixXd Js = Slist;
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);
    Eigen::VectorXd sListTemp(Slist.col(0).size());
    for(int i = 1; i < thetaList.size(); i++) {
        sListTemp << Slist.col(i-1) * thetaList(i-1);
        T = T * MatrixExp6(VecTose3(sListTemp));
        // std::cout << "array: " << sListTemp << std::endl;
        Js.col(i) = Adjoint(T) * Slist.col(i);
    }

    return Js;
}

}