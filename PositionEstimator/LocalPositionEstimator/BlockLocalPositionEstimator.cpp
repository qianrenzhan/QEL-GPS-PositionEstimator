#include <iostream>
#include "BlockLocalPositionEstimator.h"
#include "../gyro/JY901.h"
#pragma warning(disable:4244)

using namespace std;


// required standard deviation of estimate for estimator to publish data
static const int 		EST_STDDEV_XY_VALID = 2.0; // 2.0 m
static const int 		EST_STDDEV_Z_VALID = 2.0; // 2.0 m
static const int 		EST_STDDEV_TZ_VALID = 2.0; // 2.0 m

static const float P_MAX = 1.0e6f; // max allowed value in state covariance
static const float LAND_RATE = 10.0f; // rate of land detector correction

BlockLocalPositionEstimator::BlockLocalPositionEstimator() :
	// reference altitudes
	_altOrigin(0),
	_altOriginInitialized(false),
	_baroAltOrigin(0),
	_gpsAltOrigin(0),

	_estimatorInitialized(0),

	// kf matrices
	_x(), _u(), _P(), _R_att(), _eul()
{
	// initialize A, B,  P, x, u
	_x.setZero();
	_u.setZero();
	initSS();
	_estimatorInitialized = true;
	baroInit();
	gpsInit();
	
	cout << "lpe初始化完成..." << endl;
}

BlockLocalPositionEstimator::~BlockLocalPositionEstimator()
{
}

BlockLocalPositionEstimator BlockLocalPositionEstimator::operator=(const BlockLocalPositionEstimator &)
{
	return BlockLocalPositionEstimator();
}

Vector<float, BlockLocalPositionEstimator::n_x> BlockLocalPositionEstimator::dynamics(
	float t,
	const Vector<float, BlockLocalPositionEstimator::n_x> &x,
	const Vector<float, BlockLocalPositionEstimator::n_u> &u)
{
	return _A * x + _B * u;
}

void BlockLocalPositionEstimator::update()
{
	bool gpsUpdated = 1;
	bool baroUpdated = 0;

	// do prediction
	predict();

	// sensor corrections/ initializations
	if (gpsUpdated) 
	{
		gpsCorrect();
	}
	if (baroUpdated) 
	{
		baroCorrect();
	}

	publishLocalPos();
	//cout << "lpe updata!"<<endl;
}


void BlockLocalPositionEstimator::initP()
{
	_P.setZero();
	// initialize to twice valid condition
	_P(X_x, X_x) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	_P(X_y, X_y) = 2 * EST_STDDEV_XY_VALID * EST_STDDEV_XY_VALID;
	_P(X_z, X_z) = 2 * EST_STDDEV_Z_VALID * EST_STDDEV_Z_VALID;
	//=======================================================
	//_P(X_vx, X_vx) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	//_P(X_vy, X_vy) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	//// use vxy thresh for vz init as well
	//_P(X_vz, X_vz) = 2 * _vxy_pub_thresh.get() * _vxy_pub_thresh.get();
	//=======================================================
	_P(X_vx, X_vx) = 2 * 0.3f * 0.3f;
	_P(X_vy, X_vy) = 2 * 0.3f * 0.3f;
	// use vxy thresh for vz init as well
	_P(X_vz, X_vz) = 2 * 0.3f * 0.3f;
	// initialize bias uncertainty to small values to keep them stable
	_P(X_bx, X_bx) = 1e-6f;
	_P(X_by, X_by) = 1e-6f;
	_P(X_bz, X_bz) = 1e-6f;
	_P(X_tz, X_tz) = 2 * EST_STDDEV_TZ_VALID * EST_STDDEV_TZ_VALID;
}

void BlockLocalPositionEstimator::initSS()
{
	initP();

	// dynamics matrix
	_A.setZero();
	// derivative of position is velocity
	_A(X_x, X_vx) = 1;
	_A(X_y, X_vy) = 1;
	_A(X_z, X_vz) = 1;

	// input matrix
	_B.setZero();
	_B(X_vx, U_ax) = 1;
	_B(X_vy, U_ay) = 1;
	_B(X_vz, U_az) = 1;

	// update components that depend on current state
	updateSSStates();
	updateSSParams();
}

void BlockLocalPositionEstimator::updateSSStates()
{
	// derivative of velocity is accelerometer acceleration
	// (in input matrix) - bias (in body frame)
	_A(X_vx, X_bx) = -_R_att(0, 0);
	_A(X_vx, X_by) = -_R_att(0, 1);
	_A(X_vx, X_bz) = -_R_att(0, 2);

	_A(X_vy, X_bx) = -_R_att(1, 0);
	_A(X_vy, X_by) = -_R_att(1, 1);
	_A(X_vy, X_bz) = -_R_att(1, 2);

	_A(X_vz, X_bx) = -_R_att(2, 0);
	_A(X_vz, X_by) = -_R_att(2, 1);
	_A(X_vz, X_bz) = -_R_att(2, 2);
}

void BlockLocalPositionEstimator::updateSSParams()
{
	// input noise covariance matrix
	_R.setZero();
	//_R(U_ax, U_ax) = _accel_xy_stddev.get() * _accel_xy_stddev.get();   LPE_ACC_XY
	_R(U_ax, U_ax) = 0.012f * 0.012f;
	//_R(U_ay, U_ay) = _accel_xy_stddev.get() * _accel_xy_stddev.get();   LPE_ACC_XY
	_R(U_ay, U_ay) = 0.012f * 0.012f;
	//_R(U_az, U_az) = _accel_z_stddev.get() * _accel_z_stddev.get();     LPE_ACC_Z
	_R(U_az, U_az) = 0.02f * 0.02f;

	// process noise power matrix
	_Q.setZero();
	//float pn_p_sq = _pn_p_noise_density.get() * _pn_p_noise_density.get();
	float pn_p_sq = 0.1f * 0.1f;
	//float pn_v_sq = _pn_v_noise_density.get() * _pn_v_noise_density.get();
	float pn_v_sq = 0.1f * 0.1f;
	_Q(X_x, X_x) = pn_p_sq;
	_Q(X_y, X_y) = pn_p_sq;
	_Q(X_z, X_z) = pn_p_sq;
	_Q(X_vx, X_vx) = pn_v_sq;
	_Q(X_vy, X_vy) = pn_v_sq;
	_Q(X_vz, X_vz) = pn_v_sq;

	// technically, the noise is in the body frame,
	// but the components are all the same, so
	// ignoring for now
	//float pn_b_sq = _pn_b_noise_density.get() * _pn_b_noise_density.get();
	float pn_b_sq = 0.001f * 0.001f;
	_Q(X_bx, X_bx) = pn_b_sq;
	_Q(X_by, X_by) = pn_b_sq;
	_Q(X_bz, X_bz) = pn_b_sq;

	// terrain random walk noise ((m/s)/sqrt(hz)), scales with velocity
	/*float pn_t_noise_density =
		_pn_t_noise_density.get() + (_t_max_grade.get() / 100.0f) * sqrtf(_x(X_vx) * _x(X_vx) + _x(X_vy) * _x(X_vy));*/

	float pn_t_noise_density = 0.001f + (1.0 / 100.0f) * sqrtf(_x(X_vx) * _x(X_vx) + _x(X_vy) * _x(X_vy));
	_Q(X_tz, X_tz) = pn_t_noise_density * pn_t_noise_density;
}

void BlockLocalPositionEstimator::predict()
{
	// get acceleration
	float acc[3],angle[3];
	//把模块的x方向朝北，z轴置0
	angle[0] = JY901.stcAngle.Angle[0] / 32768 * 3.1415;
	angle[1] = -JY901.stcAngle.Angle[1] / 32768 * 3.1415;
	//angle[2] = JY901.stcAngle.Angle[2] / 32768 * 3.1415;
	angle[2] = 0;

	matrix::Euler<float> eul(angle[0],angle[1],angle[2]);
	matrix::Quaternion<float> q(eul);
	//q.from_euler(JY901.stcAngle.Angle[0] / 32768 * 180,JY901.stcAngle.Angle[1] / 32768 * 180,JY901.stcAngle.Angle[2] / 32768 * 180);
	//from_euler(JY901.stcAngle.Angle[0] / 32768 * 180,JY901.stcAngle.Angle[1] / 32768 * 180,JY901.stcAngle.Angle[2] / 32768 * 180);              //当前四元数     ================================
	_eul = matrix::Euler<float>(q);
	_R_att = matrix::Dcm<float>(q);

	acc[0] = (float)JY901.stcAcc.a[0]/32768*16*9.81;
	acc[1] = -(float)JY901.stcAcc.a[1]/32768*16*9.81;
	acc[2] = -(float)JY901.stcAcc.a[2]/32768*16*9.81;


	Vector3f a(acc[0], acc[1], acc[2]);                                  //当前加速度     ================================
	// note, bias is removed in dynamics function
	_u = _R_att * a;
	_u(U_az) += 9.81f; // add g

	// update state space based on new states
	updateSSStates();

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = getDt();
	Vector<float, n_x> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, n_x> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	//// don't integrate position if no valid xy data
	//if (!(_estimatorInitialized & EST_XY))  
	//{
	//	dx(X_x) = 0;
	//	dx(X_vx) = 0;
	//	dx(X_y) = 0;
	//	dx(X_vy) = 0;
	//}

	//// don't integrate z if no valid z data
	//if (!(_estimatorInitialized & EST_Z))  
	//{
	//	dx(X_z) = 0;
	//}

	//// don't integrate tz if no valid tz data
	//if (!(_estimatorInitialized & EST_TZ))  
	//{
	//	dx(X_tz) = 0;
	//}

	// saturate bias
	float bx = dx(X_bx) + _x(X_bx);
	float by = dx(X_by) + _x(X_by);
	float bz = dx(X_bz) + _x(X_bz);

	if (std::abs(bx) > BIAS_MAX) 
	{
		bx = BIAS_MAX * bx / std::abs(bx);
		dx(X_bx) = bx - _x(X_bx);
	}

	if (std::abs(by) > BIAS_MAX) {
		by = BIAS_MAX * by / std::abs(by);
		dx(X_by) = by - _x(X_by);
	}

	if (std::abs(bz) > BIAS_MAX) {
		bz = BIAS_MAX * bz / std::abs(bz);
		dx(X_bz) = bz - _x(X_bz);
	}

	// propagate
	_x += dx;
	Matrix<float, n_x, n_x> dP = (_A * _P + _P * _A.transpose() +
				      _B * _R * _B.transpose() + _Q) * getDt();

	// covariance propagation logic
	for (int i = 0; i < n_x; i++) 
	{
		if (_P(i, i) > P_MAX) 
		{
			// if diagonal element greater than max, stop propagating
			dP(i, i) = 0;

			for (int j = 0; j < n_x; j++) 
			{
				dP(i, j) = 0;
				dP(j, i) = 0;
			}
		}
	}
	_P += dP;
	//_xLowPass.update(_x);
	//_aglLowPass.update(agl());
}

void BlockLocalPositionEstimator::publishLocalPos()
{
	//const Vector<float, n_x> &xLP = _xLowPass.getState();

	//_pub_lpos.get().x = xLP(X_x); 	// north
	//_pub_lpos.get().y = xLP(X_y);  	// east
	//_pub_lpos.get().z = xLP(X_z); 	// down

	//_pub_lpos.get().vx = xLP(X_vx); // north
	//_pub_lpos.get().vy = xLP(X_vy); // east
	//_pub_lpos.get().vz = xLP(X_vz); // down
}

