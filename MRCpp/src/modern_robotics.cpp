#include "../include/modern_robotics.h"

/*
 * modernRobotics.cpp
 * Adapted from modern_robotics.py provided by modernrobotics.org
 * Provides useful Jacobian and frame representation functions
 */
#include <Eigen/Dense>
#include <cmath>
#include <vector>

# define M_PI           3.14159265358979323846  /* pi */

namespace mr {

	/* Function: Find if the value is negligible enough to consider 0
	 * Inputs: value to be checked as a double
	 * Returns: Boolean of true-ignore or false-can't ignore
	 */
	bool NearZero(const double val) {
		return (std::abs(val) < .000001);
	}

	/*
	 * Function: Calculate the 6x6 matrix [adV] of the given 6-vector
	 * Input: Eigen::VectorXd (6x1)
	 * Output: Eigen::MatrixXd (6x6)
	 * Note: Can be used to calculate the Lie bracket [V1, V2] = [adV1]V2
	 */
	Eigen::MatrixXd ad(Eigen::VectorXd V) {
		Eigen::Matrix3d omgmat = VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

		Eigen::MatrixXd result(6, 6);
		result.topLeftCorner<3, 3>() = omgmat;
		result.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero(3, 3);
		result.bottomLeftCorner<3, 3>() = VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
		result.bottomRightCorner<3, 3>() = omgmat;
		return result;
	}

	/* Function: Returns a normalized version of the input vector
	 * Input: Eigen::MatrixXd
	 * Output: Eigen::MatrixXd
	 * Note: MatrixXd is used instead of VectorXd for the case of row vectors
	 * 		Requires a copy
	 *		Useful because of the MatrixXd casting
	 */
	Eigen::MatrixXd Normalize(Eigen::MatrixXd V) {
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
		v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
		return v_ret;
	}


	/* Function: Translates an exponential rotation into it's individual components
	 * Inputs: Exponential rotation (rotation matrix in terms of a rotation axis
	 *				and the angle of rotation)
	 * Returns: The axis and angle of rotation as [x, y, z, theta]
	 */
	Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3) {
		Eigen::Vector4d v_ret;
		v_ret << Normalize(expc3), expc3.norm();
		return v_ret;
	}


	/* Function: Translates an exponential rotation into a rotation matrix
	 * Inputs: exponenential representation of a rotation
	 * Returns: Rotation matrix
	 */
	Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat) {
		Eigen::Vector3d omgtheta = so3ToVec(so3mat);

		Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
		if (NearZero(so3mat.norm())) {
			return m_ret;
		}
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = so3mat * (1 / theta);
			return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
		}
	}


	/* Function: Computes the matrix logarithm of a rotation matrix
	 * Inputs: Rotation matrix
	 * Returns: matrix logarithm of a rotation
	 */
	Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R) {
		double acosinput = (R.trace() - 1) / 2.0;
		Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d omg;
			if (!NearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!NearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}
	}

	/* Function: Combines a rotation matrix and position vector into a single
	 * 				Special Euclidian Group (SE3) homogeneous transformation matrix
	 * Inputs: Rotation Matrix (R), Position Vector (p)
	 * Returns: Matrix of T = [ [R, p],
	 *						    [0, 1] ]
	 */
	Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p) {
		Eigen::MatrixXd m_ret(4, 4);
		m_ret << R, p,
			0, 0, 0, 1;
		return m_ret;
	}


	/* Function: Separates the rotation matrix and position vector from
	 *				the transfomation matrix representation
	 * Inputs: Homogeneous transformation matrix
	 * Returns: std::vector of [rotation matrix, position vector]
	 */
	std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T) {
		std::vector<Eigen::MatrixXd> Rp_ret;
		Eigen::Matrix3d R_ret;
		// Get top left 3x3 corner
		R_ret = T.block<3, 3>(0, 0);

		Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

		Rp_ret.push_back(R_ret);
		Rp_ret.push_back(p_ret);

		return Rp_ret;
	}


	/* Function: Translates a spatial velocity vector into a transformation matrix
	 * Inputs: Spatial velocity vector [angular velocity, linear velocity]
	 * Returns: Transformation matrix
	 */
	Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V) {
		// Separate angular (exponential representation) and linear velocities
		Eigen::Vector3d exp(V(0), V(1), V(2));
		Eigen::Vector3d linear(V(3), V(4), V(5));

		// Fill in values to the appropriate parts of the transformation matrix
		Eigen::MatrixXd m_ret(4, 4);
		m_ret << VecToso3(exp), linear,
			0, 0, 0, 0;

		return m_ret;
	}


	/* Function: Translates a transformation matrix into a spatial velocity vector
	 * Inputs: Transformation matrix
	 * Returns: Spatial velocity vector [angular velocity, linear velocity]
	 */
	Eigen::VectorXd se3ToVec(const Eigen::MatrixXd& T) {
		Eigen::VectorXd m_ret(6);
		m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

		return m_ret;
	}


	/* Function: Provides the adjoint representation of a transformation matrix
	 *			 Used to change the frame of reference for spatial velocity vectors
	 * Inputs: 4x4 Transformation matrix SE(3)
	 * Returns: 6x6 Adjoint Representation of the matrix
	 */
	Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T) {
		std::vector<Eigen::MatrixXd> R = TransToRp(T);
		Eigen::MatrixXd ad_ret(6, 6);
		Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
		ad_ret << R[0], zeroes,
			VecToso3(R[1]) * R[0], R[0];
		return ad_ret;
	}


	/* Function: Rotation expanded for screw axis
	 * Inputs: se3 matrix representation of exponential coordinates (transformation matrix)
	 * Returns: 6x6 Matrix representing the rotation
	 */
	Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat) {
		// Extract the angular velocity vector from the transformation matrix
		Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
		Eigen::Vector3d omgtheta = so3ToVec(se3mat_cut);

		Eigen::MatrixXd m_ret(4, 4);

		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if (NearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
			omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
			m_ret << se3mat_cut, omgtheta,
				0, 0, 0, 1;
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
			Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
			m_ret << MatrixExp3(se3mat_cut), GThetaV,
				0, 0, 0, 1;
			return m_ret;
		}

	}

	Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T) {
		Eigen::MatrixXd m_ret(4, 4);
		auto rp = mr::TransToRp(T);
		Eigen::Matrix3d omgmat = MatrixLog3(rp.at(0));
		Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
		if (omgmat.isApprox(zeros3d)) {
			m_ret << zeros3d, rp.at(1),
				0, 0, 0, 0;
		}
		else {
			double theta = std::acos((rp.at(0).trace() - 1) / 2.0);
			Eigen::Matrix3d logExpand1 = Eigen::MatrixXd::Identity(3, 3) - omgmat / 2.0;
			Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*rp.at(1),
				0, 0, 0, 0;
		}
		return m_ret;
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
	Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList) {
		Eigen::MatrixXd T = M;
		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}
		return T;
	}

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
	Eigen::MatrixXd FKinBody(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Blist, const Eigen::VectorXd& thetaList) {
		Eigen::MatrixXd T = M;
		for (int i = 0; i < thetaList.size(); i++) {
			T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
		}
		return T;
	}


	/* Function: Gives the space Jacobian
	 * Inputs: Screw axis in home position, joint configuration
	 * Returns: 6xn Spatial Jacobian
	 */
	Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetaList) {
		Eigen::MatrixXd Js = Slist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd sListTemp(Slist.col(0).size());
		for (int i = 1; i < thetaList.size(); i++) {
			sListTemp << Slist.col(i - 1) * thetaList(i - 1);
			T = T * MatrixExp6(VecTose3(sListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Js.col(i) = Adjoint(T) * Slist.col(i);
		}

		return Js;
	}

	/*
	 * Function: Gives the body Jacobian
	 * Inputs: Screw axis in BODY position, joint configuration
	 * Returns: 6xn Bobdy Jacobian
	 */
	Eigen::MatrixXd JacobianBody(const Eigen::MatrixXd& Blist, const Eigen::MatrixXd& thetaList) {
		Eigen::MatrixXd Jb = Blist;
		Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
		Eigen::VectorXd bListTemp(Blist.col(0).size());
		for (int i = thetaList.size() - 2; i >= 0; i--) {
			bListTemp << Blist.col(i + 1) * thetaList(i + 1);
			T = T * MatrixExp6(VecTose3(-1 * bListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Jb.col(i) = Adjoint(T) * Blist.col(i);
		}
		return Jb;
	}

	Eigen::MatrixXd TransInv(const Eigen::MatrixXd& transform) {
		auto rp = mr::TransToRp(transform);
		auto Rt = rp.at(0).transpose();
		auto t = -(Rt * rp.at(1));
		Eigen::MatrixXd inv = Eigen::MatrixXd::Zero(4, 4);
		inv.block(0, 0, 3, 3) = Rt;
		inv.block(0, 3, 3, 1) = t;
		inv(3, 3) = 1;
		return inv;
	}

	Eigen::MatrixXd RotInv(const Eigen::MatrixXd& rotMatrix) {
		return rotMatrix.transpose();
	}

	Eigen::VectorXd ScrewToAxis(Eigen::Vector3d q, Eigen::Vector3d s, double h) {
		Eigen::VectorXd axis(6);
		axis.segment(0, 3) = s;
		axis.segment(3, 3) = q.cross(s) + (h * s);
		return axis;
	}

	Eigen::VectorXd AxisAng6(const Eigen::VectorXd& expc6) {
		Eigen::VectorXd v_ret(7);
		double theta = Eigen::Vector3d(expc6(0), expc6(1), expc6(2)).norm();
		if (NearZero(theta))
			theta = Eigen::Vector3d(expc6(3), expc6(4), expc6(5)).norm();
		v_ret << expc6 / theta, theta;
		return v_ret;
	}

	Eigen::MatrixXd ProjectToSO3(const Eigen::MatrixXd& M) {
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::MatrixXd R = svd.matrixU() * svd.matrixV().transpose();
		if (R.determinant() < 0)
			// In this case the result may be far from M; reverse sign of 3rd column
			R.col(2) *= -1;
		return R;
	}

	Eigen::MatrixXd ProjectToSE3(const Eigen::MatrixXd& M) {
		Eigen::Matrix3d R = M.block<3, 3>(0, 0);
		Eigen::Vector3d t = M.block<3, 1>(0, 3);
		Eigen::MatrixXd T = RpToTrans(ProjectToSO3(R), t);
		return T;
	}

	double DistanceToSO3(const Eigen::Matrix3d& M) {
		if (M.determinant() > 0)
			return (M.transpose() * M - Eigen::Matrix3d::Identity()).norm();
		else
			return 1.0e9;
	}

	double DistanceToSE3(const Eigen::Matrix4d& T) {
		Eigen::Matrix3d matR = T.block<3, 3>(0, 0);
		if (matR.determinant() > 0) {
			Eigen::Matrix4d m_ret;
			m_ret << matR.transpose()*matR, Eigen::Vector3d::Zero(3),
				T.row(3);
			m_ret = m_ret - Eigen::Matrix4d::Identity();
			return m_ret.norm();
		}
		else
			return 1.0e9;
	}

	bool TestIfSO3(const Eigen::Matrix3d& M) {
		return std::abs(DistanceToSO3(M)) < 1e-3;
	}

	bool TestIfSE3(const Eigen::Matrix4d& T) {
		return std::abs(DistanceToSE3(T)) < 1e-3;
	}
	bool IKinBody(const Eigen::MatrixXd& Blist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T,
		Eigen::VectorXd& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		Eigen::MatrixXd Tfk = FKinBody(M, Blist, thetalist);
		Eigen::MatrixXd Tdiff = TransInv(Tfk)*T;
		Eigen::VectorXd Vb = se3ToVec(MatrixLog6(Tdiff));
		Eigen::Vector3d angular(Vb(0), Vb(1), Vb(2));
		Eigen::Vector3d linear(Vb(3), Vb(4), Vb(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Eigen::MatrixXd Jb;
		while (err && i < maxiterations) {
			Jb = JacobianBody(Blist, thetalist);
			thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
			i += 1;
			// iterate
			Tfk = FKinBody(M, Blist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vb = se3ToVec(MatrixLog6(Tdiff));
			angular = Eigen::Vector3d(Vb(0), Vb(1), Vb(2));
			linear = Eigen::Vector3d(Vb(3), Vb(4), Vb(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}

	bool IKinSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& M, const Eigen::MatrixXd& T,
		Eigen::VectorXd& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		Eigen::MatrixXd Tfk = FKinSpace(M, Slist, thetalist);
		Eigen::MatrixXd Tdiff = TransInv(Tfk)*T;
		Eigen::VectorXd Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
		Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
		Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Eigen::MatrixXd Js;
		while (err && i < maxiterations) {
			Js = JacobianSpace(Slist, thetalist);
			thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
			i += 1;
			// iterate
			Tfk = FKinSpace(M, Slist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
			angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
			linear = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}

	Eigen::VectorXd InverseDynamics(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const Eigen::VectorXd& ddthetalist,
		const Eigen::VectorXd& g, const Eigen::VectorXd& Ftip, const std::vector<Eigen::MatrixXd>& Mlist, const std::vector<Eigen::MatrixXd>& Glist,
		const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::MatrixXd Mi = Eigen::MatrixXd::Identity(4, 4);
		Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(6, n);
		std::vector<Eigen::MatrixXd> AdTi(n + 1);
		Eigen::MatrixXd Vi = Eigen::MatrixXd::Zero(6, n + 1);
		Eigen::MatrixXd Vdi = Eigen::MatrixXd::Zero(6, n + 1);
		Vdi.col(0) << 0, 0, 0, -g;
		AdTi[n] = Adjoint(TransInv(Mlist[n]));
		Eigen::VectorXd Fi = Ftip;
		Eigen::VectorXd taulist = Eigen::VectorXd::Zero(n);
		Eigen::MatrixXd Mt;

		for (int i = 0; i < n; ++i) {
			Mi *= Mlist[i];
			Ai.col(i) = Adjoint(TransInv(Mi))*Slist.col(i);
			Mt = VecTose3(Ai.col(i)* -1 * thetalist(i));
			AdTi[i] = Adjoint(MatrixExp6(Mt)*TransInv(Mlist[i]));
			Vi.col(i + 1) = AdTi[i] * Vi.col(i) + Ai.col(i)*dthetalist(i);
			Vdi.col(i + 1) = AdTi[i] * Vdi.col(i) + Ai.col(i)*ddthetalist(i) + ad(Vi.col(i + 1))*Ai.col(i)*dthetalist(i);
		}
		for (int i = n - 1; i > -1; --i) {
			Fi = AdTi[i + 1].transpose() * Fi + Glist[i] * Vdi.col(i + 1) - ad(Vi.col(i + 1)).transpose() *(Glist[i] * Vi.col(i + 1));
			taulist(i) = Fi.transpose() * Ai.col(i);
		}
		return taulist;
	}

	Eigen::MatrixXd MassMatrix(const Eigen::VectorXd& thetalist, const std::vector<Eigen::MatrixXd>& Mlist, 
		const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::MatrixXd M = Eigen::MatrixXd::Zero(n, n);
		Eigen::VectorXd dthetalist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd ddthetalist;
		Eigen::VectorXd g(3); g << 0, 0, 0;
		Eigen::VectorXd Ftip(6); Ftip << 0, 0, 0, 0, 0, 0;
		for (int i = 0; i < n; ++i) {
			ddthetalist = Eigen::VectorXd::Zero(n);
			ddthetalist(i) = 1;
			M.col(i) = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist);
		}
		return M;
	}
	Eigen::VectorXd VelQuadraticForces(const Eigen::VectorXd& thetalist, const Eigen::VectorXd& dthetalist, const std::vector<Eigen::MatrixXd>& Mlist,
		const std::vector<Eigen::MatrixXd>& Glist, const Eigen::MatrixXd& Slist) {
		int n = thetalist.size();
		Eigen::VectorXd ddthetalist = Eigen::VectorXd::Zero(n);
		Eigen::VectorXd g(3); g << 0, 0, 0;
		Eigen::VectorXd Ftip(6); Ftip << 0, 0, 0, 0, 0, 0;
		Eigen::VectorXd vqForces = InverseDynamics(thetalist, dthetalist, ddthetalist, g, Ftip, Mlist, Glist, Slist);
		return vqForces;
	}
}
