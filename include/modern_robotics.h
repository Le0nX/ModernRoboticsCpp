#pragma once

#include <Eigen/Dense>
#include <vector>

namespace mr {

/*
 * Function: Find if the value is negligible enough to consider 0
 * Inputs: value to be checked as a double
 * Returns: Boolean of true-ignore or false-can't ignore
 */
bool NearZero(const double);

/*
 * Function: Calculate the 6x6 matrix [adV] of the given 6-vector
 * Input: Eigen::VectorXd (6x1)
 * Output: Eigen::MatrixXd (6x6)
 * Note: Can be used to calculate the Lie bracket [V1, V2] = [adV1]V2
 */
Eigen::MatrixXd ad(Eigen::VectorXd);


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


/* Function: Computes the matrix logarithm of a rotation matrix
 * Inputs: Rotation matrix
 * Returns: matrix logarithm of a rotation
 */
Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d&);


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


/* Function: Translates a transformation matrix into a spatial velocity vector
 * Inputs: Transformation matrix
 * Returns: Spatial velocity vector [angular velocity, linear velocity]
 */
Eigen::VectorXd se3ToVec(const Eigen::MatrixXd&);


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
 * Function: Computes the matrix logarithm of a homogeneous transformation matrix
 * Inputs: R: Transformation matrix in SE3
 * Returns: The matrix logarithm of R
 */
Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd&);


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
 * Function: Compute end effector frame (used for current body position calculation)
 * Inputs: Home configuration (position and orientation) of end-effector
 *		   The joint screw axes in the body frame when the manipulator
 *             is at the home position
 * 		   A list of joint coordinates.
 * Returns: Transfomation matrix representing the end-effector frame when the joints are
 *				at the specified coordinates
 * Notes: FK means Forward Kinematics
 */
Eigen::MatrixXd FKinBody(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::VectorXd&);


/*
 * Function: Gives the space Jacobian
 * Inputs: Screw axis in home position, joint configuration
 * Returns: 6xn Spatial Jacobian
 */
Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd&, const Eigen::MatrixXd&);


/*
 * Function: Gives the body Jacobian
 * Inputs: Screw axis in BODY position, joint configuration
 * Returns: 6xn Bobdy Jacobian
 */
Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd&, const Eigen::MatrixXd&);


/*
 * Inverts a homogeneous transformation matrix
 * Inputs: A homogeneous transformation Matrix T
 * Returns: The inverse of T
 */
Eigen::MatrixXd TransInv(const Eigen::MatrixXd&);

/*
 * Inverts a rotation matrix
 * Inputs: A rotation matrix  R
 * Returns: The inverse of R
 */
Eigen::MatrixXd RotInv(const Eigen::MatrixXd&);

/*
 * Takes a parametric description of a screw axis and converts it to a
 * normalized screw axis
 * Inputs:
 * q: A point lying on the screw axis
 * s: A unit vector in the direction of the screw axis
 * h: The pitch of the screw axis
 * Returns: A normalized screw axis described by the inputs
 */
Eigen::VectorXd ScrewToAxis(Eigen::Vector3d q, Eigen::Vector3d s, double h);


/*
 * Function: Translates a 6-vector of exponential coordinates into screw
 * axis-angle form
 * Inputs: 
 * expc6: A 6-vector of exponential coordinates for rigid-body motion
          S*theta
 * Returns: The corresponding normalized screw axis S; The distance theta traveled
 * along/about S in form [S, theta]
 * Note: Is it better to return std::map<S, theta>?
 */
Eigen::VectorXd AxisAng6(const Eigen::VectorXd&);


/*
 * Function: Returns projection of one matrix into SO(3)
 * Inputs:
 * M:		A matrix near SO(3) to project to SO(3)
 * Returns: The closest matrix R that is in SO(3)
 * Projects a matrix mat to the closest matrix in SO(3) using singular-value decomposition
 * (see http://hades.mech.northwestern.edu/index.php/Modern_Robotics_Linear_Algebra_Review).
 * This function is only appropriate for matrices close to SO(3).
 */
Eigen::MatrixXd ProjectToSO3(const Eigen::MatrixXd&);


/*
 * Function: Returns projection of one matrix into SE(3)
 * Inputs:
 * M:		A 4x4 matrix near SE(3) to project to SE(3)
 * Returns: The closest matrix T that is in SE(3)
 * Projects a matrix mat to the closest matrix in SO(3) using singular-value decomposition
 * (see http://hades.mech.northwestern.edu/index.php/Modern_Robotics_Linear_Algebra_Review).
 * This function is only appropriate for matrices close to SE(3).
 */
Eigen::MatrixXd ProjectToSE3(const Eigen::MatrixXd&);


/*
 * Function: Returns the Frobenius norm to describe the distance of M from the SO(3) manifold
 * Inputs:
 * M: A 3x3 matrix
 * Outputs:
 *	 the distance from mat to the SO(3) manifold using the following
 * method:
 *  If det(M) <= 0, return a large number.
 *  If det(M) > 0, return norm(M^T*M - I).
 */
double DistanceToSO3(const Eigen::Matrix3d&);


/*
 * Function: Returns the Frobenius norm to describe the distance of mat from the SE(3) manifold
 * Inputs:
 * T: A 4x4 matrix
 * Outputs:
 *	 the distance from T to the SE(3) manifold using the following
 * method:
 *  Compute the determinant of matR, the top 3x3 submatrix of T.
 *  If det(matR) <= 0, return a large number.
 *  If det(matR) > 0, replace the top 3x3 submatrix of mat with matR^T*matR,
 *  and set the first three entries of the fourth column of mat to zero. Then
 *  return norm(T - I).
 */
double DistanceToSE3(const Eigen::Matrix4d&);


/*
 * Function: Returns true if M is close to or on the manifold SO(3)
 * Inputs:
 * M: A 3x3 matrix
 * Outputs:
 *	 true if M is very close to or in SO(3), false otherwise
 */
bool TestIfSO3(const Eigen::Matrix3d&);


/*
 * Function: Returns true if T is close to or on the manifold SE(3)
 * Inputs:
 * M: A 4x4 matrix
 * Outputs:
 *	 true if T is very close to or in SE(3), false otherwise
 */
bool TestIfSE3(const Eigen::Matrix4d&);


/*
 * Function: Computes inverse kinematics in the body frame for an open chain robot
 * Inputs:
 *	Blist: The joint screw axes in the end-effector frame when the
 *         manipulator is at the home position, in the format of a
 *         matrix with axes as the columns
 *	M: The home configuration of the end-effector
 *	T: The desired end-effector configuration Tsd
 *	thetalist[in][out]: An initial guess and result output of joint angles that are close to
 *         satisfying Tsd
 *	emog: A small positive tolerance on the end-effector orientation
 *        error. The returned joint angles must give an end-effector
 *        orientation error less than eomg
 *	ev: A small positive tolerance on the end-effector linear position
 *      error. The returned joint angles must give an end-effector
 *      position error less than ev
 * Outputs:
 *	success: A logical value where TRUE means that the function found
 *           a solution and FALSE means that it ran through the set
 *           number of maximum iterations without finding a solution
 *           within the tolerances eomg and ev.
 *	thetalist[in][out]: Joint angles that achieve T within the specified tolerances,
 */
bool IKinBody(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double);


/*
 * Function: Computes inverse kinematics in the space frame for an open chain robot
 * Inputs:
 *	Slist: The joint screw axes in the space frame when the
 *         manipulator is at the home position, in the format of a
 *         matrix with axes as the columns
 *	M: The home configuration of the end-effector
 *	T: The desired end-effector configuration Tsd
 *	thetalist[in][out]: An initial guess and result output of joint angles that are close to
 *         satisfying Tsd
 *	emog: A small positive tolerance on the end-effector orientation
 *        error. The returned joint angles must give an end-effector
 *        orientation error less than eomg
 *	ev: A small positive tolerance on the end-effector linear position
 *      error. The returned joint angles must give an end-effector
 *      position error less than ev
 * Outputs:
 *	success: A logical value where TRUE means that the function found
 *           a solution and FALSE means that it ran through the set
 *           number of maximum iterations without finding a solution
 *           within the tolerances eomg and ev.
 *	thetalist[in][out]: Joint angles that achieve T within the specified tolerances,
 */
bool IKinSpace(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, Eigen::VectorXd&, double, double);

/* 
 * Function: This function uses forward-backward Newton-Euler iterations to solve the 
 * equation:
 * taulist = Mlist(thetalist) * ddthetalist + c(thetalist, dthetalist) ...
 *           + g(thetalist) + Jtr(thetalist) * Ftip
 * Inputs:
 *  thetalist: n-vector of joint variables
 *  dthetalist: n-vector of joint rates
 *  ddthetalist: n-vector of joint accelerations
 *  g: Gravity vector g
 *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 * 
 * Outputs:
 *  taulist: The n-vector of required joint forces/torques
 * 
 */
Eigen::VectorXd InverseDynamics(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, 
                                   const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, 
                                   const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

/* 
 * Function: This function calls InverseDynamics with Ftip = 0, dthetalist = 0, and 
 *   ddthetalist = 0. The purpose is to calculate one important term in the dynamics equation       
 * Inputs:
 *  thetalist: n-vector of joint variables
 *  g: Gravity vector g
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 * 
 * Outputs:
 *  grav: The 3-vector showing the effect force of gravity to the dynamics
 * 
 */
Eigen::VectorXd GravityForces(const Eigen::VectorXd&, const Eigen::VectorXd&,
                                const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

/* 
 * Function: This function calls InverseDynamics n times, each time passing a 
 * ddthetalist vector with a single element equal to one and all other 
 * inputs set to zero. Each call of InverseDynamics generates a single 
 * column, and these columns are assembled to create the inertia matrix.       
 *
 * Inputs:
 *  thetalist: n-vector of joint variables
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 * 
 * Outputs:
 *  M: The numerical inertia matrix M(thetalist) of an n-joint serial
 *     chain at the given configuration thetalist.
 */
Eigen::MatrixXd MassMatrix(const Eigen::VectorXd&,
                                const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

/* 
 * Function: This function calls InverseDynamics with g = 0, Ftip = 0, and 
 * ddthetalist = 0.      
 *
 * Inputs:
 *  thetalist: n-vector of joint variables
 *  dthetalist: A list of joint rates
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 * 
 * Outputs:
 *  c: The vector c(thetalist,dthetalist) of Coriolis and centripetal
 *     terms for a given thetalist and dthetalist.
 */
Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd&, const Eigen::VectorXd&,
                            const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

/* 
 * Function: This function calls InverseDynamics with g = 0, dthetalist = 0, and 
 * ddthetalist = 0.  
 *
 * Inputs:
 *  thetalist: n-vector of joint variables 
 *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 * 
 * Outputs:
 *  JTFtip: The joint forces and torques required only to create the 
 *     end-effector force Ftip.
 */
Eigen::VectorXd EndEffectorForces(const Eigen::VectorXd&, const Eigen::VectorXd&, 
                            const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);

/* 
 * Function: This function computes ddthetalist by solving:
 * Mlist(thetalist) * ddthetalist = taulist - c(thetalist,dthetalist) 
 *                                  - g(thetalist) - Jtr(thetalist) * Ftip
 * Inputs:
 *  thetalist: n-vector of joint variables
 *  dthetalist: n-vector of joint rates
 *  taulist: An n-vector of joint forces/torques
 *  g: Gravity vector g
 *  Ftip: Spatial force applied by the end-effector expressed in frame {n+1}
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 * 
 * Outputs:
 *  ddthetalist: The resulting joint accelerations
 * 
 */
Eigen::VectorXd ForwardDynamics(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, 
                                   const Eigen::VectorXd&, const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, 
                                   const std::vector<Eigen::MatrixXd>&, const Eigen::MatrixXd&);


/*
 * Function: Compute the joint angles and velocities at the next timestep using
    first order Euler integration
 * Inputs:
 *  thetalist[in]: n-vector of joint variables
 *  dthetalist[in]: n-vector of joint rates
 *	ddthetalist: n-vector of joint accelerations
 *  dt: The timestep delta t
 *
 * Outputs:
 *  thetalist[out]: Vector of joint variables after dt from first order Euler integration
 *  dthetalist[out]: Vector of joint rates after dt from first order Euler integration
 */
void EulerStep(Eigen::VectorXd&, Eigen::VectorXd&, const Eigen::VectorXd&, double);


/*
 * Function: Compute the joint forces/torques required to move the serial chain along the given
 *	trajectory using inverse dynamics
 * Inputs:
 *  thetamat: An N x n matrix of robot joint variables (N: no. of trajecoty time step points; n: no. of robot joints
 *  dthetamat: An N x n matrix of robot joint velocities
 *  ddthetamat: An N x n matrix of robot joint accelerations
 *	g: Gravity vector g
 *	Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (if there are no tip forces
 *			 the user should input a zero matrix)
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *
 * Outputs:
 *  taumat: The N x n matrix of joint forces/torques for the specified trajectory, where each of the N rows is the vector
 *			of joint forces/torques at each time step
 */
Eigen::MatrixXd InverseDynamicsTrajectory(const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&,
	const Eigen::VectorXd&, const Eigen::MatrixXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&,
	const Eigen::MatrixXd&);


/*
 * Function: Compute the motion of a serial chain given an open-loop history of joint forces/torques
 * Inputs:
 *  thetalist: n-vector of initial joint variables
 *  dthetalist: n-vector of initial joint rates
 *  taumat: An N x n matrix of joint forces/torques, where each row is is the joint effort at any time step
 *	g: Gravity vector g
 *	Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (if there are no tip forces
 *			 the user should input a zero matrix)
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *	dt: The timestep between consecutive joint forces/torques
 *	intRes: Integration resolution is the number of times integration (Euler) takes places between each time step.
 *			Must be an integer value greater than or equal to 1
 *
 * Outputs: std::vector of [thetamat, dthetamat]
 *  thetamat: The N x n matrix of joint angles resulting from the specified joint forces/torques
 *  dthetamat: The N x n matrix of joint velocities
 */
std::vector<Eigen::MatrixXd> ForwardDynamicsTrajectory(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::MatrixXd&,
	const Eigen::VectorXd&, const Eigen::MatrixXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&,
	const Eigen::MatrixXd&, double, int);


/*
 * Function: Compute the joint control torques at a particular time instant
 * Inputs:
 *  thetalist: n-vector of joint variables
 *  dthetalist: n-vector of joint rates
 *	eint: n-vector of the time-integral of joint errors
 *	g: Gravity vector g
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *  thetalistd: n-vector of reference joint variables
 *  dthetalistd: n-vector of reference joint rates
 *  ddthetalistd: n-vector of reference joint accelerations
 *	Kp: The feedback proportional gain (identical for each joint)
 *	Ki: The feedback integral gain (identical for each joint)
 *	Kd: The feedback derivative gain (identical for each joint)
 *
 * Outputs:
 *  tau_computed: The vector of joint forces/torques computed by the feedback
 *				  linearizing controller at the current instant
 */
Eigen::VectorXd ComputedTorque(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
	const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&,
	const Eigen::MatrixXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, double, double, double);


/*
 * Function: Compute s(t) for a cubic time scaling
 * Inputs:
 *  Tf: Total time of the motion in seconds from rest to rest
 *  t: The current time t satisfying 0 < t < Tf
 *
 * Outputs:
 *  st: The path parameter corresponding to a third-order
 *      polynomial motion that begins and ends at zero velocity
 */
double CubicTimeScaling(double, double);


/*
 * Function: Compute s(t) for a quintic time scaling
 * Inputs:
 *  Tf: Total time of the motion in seconds from rest to rest
 *  t: The current time t satisfying 0 < t < Tf
 *
 * Outputs:
 *  st: The path parameter corresponding to a fifth-order
 *      polynomial motion that begins and ends at zero velocity
 *	    and zero acceleration
 */
double QuinticTimeScaling(double, double);


/*
 * Function: Compute a straight-line trajectory in joint space
 * Inputs:
 *  thetastart: The initial joint variables
 *  thetaend: The final joint variables
 *  Tf: Total time of the motion in seconds from rest to rest
 *	N: The number of points N > 1 (Start and stop) in the discrete
 *     representation of the trajectory
 *  method: The time-scaling method, where 3 indicates cubic (third-
 *          order polynomial) time scaling and 5 indicates quintic
 *          (fifth-order polynomial) time scaling
 *
 * Outputs:
 *  traj: A trajectory as an N x n matrix, where each row is an n-vector
 *        of joint variables at an instant in time. The first row is
 *        thetastart and the Nth row is thetaend . The elapsed time
 *        between each row is Tf / (N - 1)
 */
Eigen::MatrixXd JointTrajectory(const Eigen::VectorXd&, const Eigen::VectorXd&, double, int, int);


/*
 * Function: Compute a trajectory as a list of N SE(3) matrices corresponding to
 *			 the screw motion about a space screw axis
 * Inputs:
 *  Xstart: The initial end-effector configuration
 *  Xend: The final end-effector configuration
 *  Tf: Total time of the motion in seconds from rest to rest
 *	N: The number of points N > 1 (Start and stop) in the discrete
 *     representation of the trajectory
 *  method: The time-scaling method, where 3 indicates cubic (third-
 *          order polynomial) time scaling and 5 indicates quintic
 *          (fifth-order polynomial) time scaling
 *
 * Outputs:
 *  traj: The discretized trajectory as a list of N matrices in SE(3)
 *        separated in time by Tf/(N-1). The first in the list is Xstart
 *        and the Nth is Xend
 */
std::vector<Eigen::MatrixXd> ScrewTrajectory(const Eigen::MatrixXd&, const Eigen::MatrixXd&, double, int, int);


/*
 * Function: Compute a trajectory as a list of N SE(3) matrices corresponding to
 *			 the origin of the end-effector frame following a straight line
 * Inputs:
 *  Xstart: The initial end-effector configuration
 *  Xend: The final end-effector configuration
 *  Tf: Total time of the motion in seconds from rest to rest
 *	N: The number of points N > 1 (Start and stop) in the discrete
 *     representation of the trajectory
 *  method: The time-scaling method, where 3 indicates cubic (third-
 *          order polynomial) time scaling and 5 indicates quintic
 *          (fifth-order polynomial) time scaling
 *
 * Outputs:
 *  traj: The discretized trajectory as a list of N matrices in SE(3)
 *        separated in time by Tf/(N-1). The first in the list is Xstart
 *        and the Nth is Xend
 * Notes:
 *	This function is similar to ScrewTrajectory, except the origin of the
 *  end-effector frame follows a straight line, decoupled from the rotational
 *  motion.
 */
std::vector<Eigen::MatrixXd> CartesianTrajectory(const Eigen::MatrixXd&, const Eigen::MatrixXd&, double, int, int);


/*
 * Function: Compute the motion of a serial chain given an open-loop history of joint forces/torques
 * Inputs:
 *  thetalist: n-vector of initial joint variables
 *  dthetalist: n-vector of initial joint rates
 *	g: Gravity vector g
 *	Ftipmat: An N x 6 matrix of spatial forces applied by the end-effector (if there are no tip forces
 *			 the user should input a zero matrix)
 *  Mlist: List of link frames {i} relative to {i-1} at the home position
 *  Glist: Spatial inertia matrices Gi of the links
 *  Slist: Screw axes Si of the joints in a space frame, in the format
 *         of a matrix with the screw axes as the columns.
 *  thetamatd: An Nxn matrix of desired joint variables from the reference trajectory
 *  dthetamatd: An Nxn matrix of desired joint velocities
 *  ddthetamatd: An Nxn matrix of desired joint accelerations
 *	gtilde: The gravity vector based on the model of the actual robot (actual values given above)
 *  Mtildelist: The link frame locations based on the model of the actual robot (actual values given above)
 *  Gtildelist: The link spatial inertias based on the model of the actual robot (actual values given above)
 *	Kp: The feedback proportional gain (identical for each joint)
 *	Ki: The feedback integral gain (identical for each joint)
 *	Kd: The feedback derivative gain (identical for each joint)
 *	dt: The timestep between points on the reference trajectory
 *	intRes: Integration resolution is the number of times integration (Euler) takes places between each time step.
 *			Must be an integer value greater than or equal to 1
 *
 * Outputs: std::vector of [taumat, thetamat]
 *  taumat: An Nxn matrix of the controllers commanded joint forces/ torques, where each row of n forces/torques
 *			  corresponds to a single time instant
 *  thetamat: The N x n matrix of actual joint angles
 */
std::vector<Eigen::MatrixXd> SimulateControl(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
	const Eigen::MatrixXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&,
	const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&, const Eigen::MatrixXd&,
	const Eigen::VectorXd&, const std::vector<Eigen::MatrixXd>&, const std::vector<Eigen::MatrixXd>&,
	double, double, double, double, int);

}
