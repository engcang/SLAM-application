#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; 
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

//4. Change the MTK_bUILD_MANIFOLD based on user's lid_num.
// If user uses two LiDARs, only offset_R_0, offset_R_1, offset_T_0 and offset_T_1 are needed.
MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_0))
((SO3, offset_R_1))
((vect3, offset_T_0))
((vect3, offset_T_1))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
);

/*** For UrbanNav Dataset (Case: LiDAR 2)***/
/*MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))
((SO3, rot))
((SO3, offset_R_0))
((SO3, offset_R_1))
((vect3, offset_T_0))
((vect3, offset_T_1))
((vect3, vel))
((vect3, bg))
((vect3, ba))
((S2, grav))
);*/

MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, acc))
((vect3, gyro))
);

MTK_BUILD_MANIFOLD(process_noise_ikfom,
((vect3, ng))
((vect3, na))
((vect3, nbg))
((vect3, nba))
);

MTK::get_cov<process_noise_ikfom>::type process_noise_cov()
{
	MTK::get_cov<process_noise_ikfom>::type cov = MTK::get_cov<process_noise_ikfom>::type::Zero();
	MTK::setDiagonal<process_noise_ikfom, vect3, 0>(cov, &process_noise_ikfom::ng, 0.0001);// 0.03
	MTK::setDiagonal<process_noise_ikfom, vect3, 3>(cov, &process_noise_ikfom::na, 0.0001); // *dt 0.01 0.01 * dt * dt 0.05
	MTK::setDiagonal<process_noise_ikfom, vect3, 6>(cov, &process_noise_ikfom::nbg, 0.00001); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
	MTK::setDiagonal<process_noise_ikfom, vect3, 9>(cov, &process_noise_ikfom::nba, 0.00001);   //0.001 0.05 0.0001/out 0.01
	return cov;
}

//double L_offset_to_I[3] = {0.04165, 0.02926, -0.0284}; // Avia 
//vect3 Lidar_offset_to_IMU(L_offset_to_I, 3);
Eigen::MatrixXd get_f(state_ikfom &s, const input_ikfom &in, int &lid_num)
{
	int size_row = 18 + 6 * lid_num;
	Eigen::MatrixXd res = Eigen::MatrixXd::Zero(size_row, 1);
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	vect3 a_inertial = s.rot * (in.acc-s.ba); 
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];
		res(i + 3) =  omega[i]; 
		res(i + 6 * (lid_num + 1)) = a_inertial[i] + s.grav[i]; 
	}
	return res;
}

Eigen::MatrixXd df_dx(state_ikfom &s, const input_ikfom &in, int &lid_num)
{
	int size_row = 18 + 6 * lid_num;
	int size_col = size_row - 1;
	Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(size_row, size_col);
	cov.template block<3, 3>(0, 6 * (lid_num + 1)) = Eigen::Matrix3d::Identity();
	vect3 acc_;
	in.acc.boxminus(acc_, s.ba);
	vect3 omega;
	in.gyro.boxminus(omega, s.bg);
	cov.template block<3, 3>(6 * (lid_num + 1), 3) = -s.rot.toRotationMatrix()*MTK::hat(acc_);
	cov.template block<3, 3>(6 * (lid_num + 1), 6 * (lid_num + 2)) = -s.rot.toRotationMatrix();
	Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
	Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
	s.S2_Mx(grav_matrix, vec, 6 * (lid_num + 2) + 3);
	cov.template block<3, 2>(6 * (lid_num + 1), 6 * (lid_num + 2) + 3) =  grav_matrix; 
	cov.template block<3, 3>(3, 6 * (lid_num + 1) + 3) = -Eigen::Matrix3d::Identity(); 
	return cov;
}


Eigen::MatrixXd df_dw(state_ikfom &s, const input_ikfom &in, int &lid_num)
{
	int size_row = 18 + 6 * lid_num;
	Eigen::MatrixXd cov = Eigen::MatrixXd::Zero(size_row, 12);
	cov.template block<3, 3>(6 * (lid_num + 1), 3) = -s.rot.toRotationMatrix();
	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(6 * (lid_num + 1) + 3, 6) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(6 * (lid_num + 2), 9) = Eigen::Matrix3d::Identity();
	return cov;
}

vect3 SO3ToEuler(const SO3 &orient) 
{
	Eigen::Matrix<double, 3, 1> _ang;
	Eigen::Vector4d q_data = orient.coeffs().transpose();
	//scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
	double sqw = q_data[3]*q_data[3];
	double sqx = q_data[0]*q_data[0];
	double sqy = q_data[1]*q_data[1];
	double sqz = q_data[2]*q_data[2];
	double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
	double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

	if (test > 0.49999*unit) { // singularity at north pole
	
		_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
	if (test < -0.49999*unit) { // singularity at south pole
		_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
		double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
		vect3 euler_ang(temp, 3);
		return euler_ang;
	}
		
	_ang <<
			std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
			std::asin (2*test/unit),
			std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
	vect3 euler_ang(temp, 3);
		// euler_ang[0] = roll, euler_ang[1] = pitch, euler_ang[2] = yaw
	return euler_ang;
}

#endif