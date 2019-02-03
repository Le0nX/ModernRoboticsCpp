#ifndef MODERNROBOTICSCPP_LIBRARY_H
#define MODERNROBOTICSCPP_LIBRARY_H

#include <eigen3/Eigen/Dense>
#include <vector>

/*
 * Function: Find if the value is negligible enough to consider 0
 * Inputs: value to be checked as a double
 * Returns: Boolean of true-ignore or false-can't ignore
 */
bool NearZero(const double);


/*
 * Function: Returns a normalized version of the input vector
 * Input: Eigen::MatrixXd
 * Output: Eigen::MatrixXd
 * Note: MatrixXd is used instead of VectorXd for the case of row vectors
 * 		Requires a copy
 *		Useful because of the MatrixXd casting
 */
Eigen::MatrixXd Normalize(Eigen::MatrixXd);


/*
 * Function: Returns the skew symmetric matrix representation of an angular velocity vector
 * Input: Eigen::Vector3d 3x1 angular velocity vector
 * Returns: Eigen::MatrixXd 3x3 skew symmetric matrix
 */
Eigen::Matrix3d VecToso3(const Eigen::Vector3d&);


/*
 * Function: Returns angular velocity vector represented by the skew symmetric matrix
 * Inputs: Eigen::MatrixXd 3x3 skew symmetric matrix
 * Returns: Eigen::Vector3d 3x1 angular velocity
 */
Eigen::Vector3d so3ToVec(const Eigen::MatrixXd&);


/*
 * Function: Tranlates an exponential rotation into it's individual components
 * Inputs: Exponential rotation (rotation matrix in terms of a rotation axis
 *				and the angle of rotation)
 * Returns: The axis and angle of rotation as [x, y, z, theta]
 */
Eigen::Vector4d AxisAng3(const Eigen::Vector3d&);


/*
 * Function: Translates an exponential rotation into a rotation matrix
 * Inputs: exponenential representation of a rotation
 * Returns: Rotation matrix
 */
Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d&);


/*
 * Function: Combines a rotation matrix and position vector into a single
 * 				Special Euclidian Group (SE3) homogeneous transformation matrix
 * Inputs: Rotation Matrix (R), Position Vector (p)
 * Returns: Matrix of T = [ [R, p],
 *						    [0, 1] ]
 */
Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d&, const Eigen::Vector3d&);


/*
 * Function: Separates the rotation matrix and position vector from
 *				the transfomation matrix representation
 * Inputs: Homogeneous transformation matrix
 * Returns: std::vector of [rotation matrix, position vector]
 */
std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd&);


/*
 * Function: Translates a spatial velocity vector into a transformation matrix
 * Inputs: Spatial velocity vector [angular velocity, linear velocity]
 * Returns: Transformation matrix
 */
Eigen::MatrixXd VecTose3(const Eigen::VectorXd&);


/*
 * Function: Provides the adjoint representation of a transformation matrix
 *			 Used to change the frame of reference for spatial velocity vectors
 * Inputs: 4x4 Transformation matrix SE(3)
 * Returns: 6x6 Adjoint Representation of the matrix
 */
Eigen::MatrixXd Adjoint(const Eigen::MatrixXd&);


/*
 * Function: Rotation expanded for screw axis
 * Inputs: se3 matrix representation of exponential coordinates (transformation matrix)
 * Returns: 6x6 Matrix representing the rotation
 */
Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd&);


/*
 * Function: Compute end effector frame (used for current spatial position calculation)
 * Inputs: Home configuration (position and orientation) of end-effector
 *		   The joint screw axes in the space frame when the manipulator
 *             is at the home position
 * 		   A list of joint coordinates.
 * Returns: Transfomation matrix representing the end-effector frame when the joints are
 *				at the specified coordinates
 * Notes: FK means Forward Kinematics
 */
Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::VectorXd&);


/*
 * Function: Gives the space Jacobian
 * Inputs: Screw axis in home position, joint configuration
 * Returns: 6xn Spatial Jacobian
 */
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd&, const Eigen::MatrixXd&);

#endif /* MODERNROBOTICSCPP_LIBRARY_H */
