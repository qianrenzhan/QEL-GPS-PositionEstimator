#pragma once
#include "../geo/geo.h"
#include "../matrix/math.hpp"


using namespace matrix;

static const float DELAY_MAX = 0.5f; // seconds
static const float HIST_STEP = 0.05f; // 20 hz
static const float BIAS_MAX = 1e-1f;
static const size_t HIST_LEN = 10; // DELAY_MAX / HIST_STEP;
static const size_t N_DIST_SUBS = 4;

// for fault detection
// chi squared distribution, false alarm probability 0.0001
// see fault_table.py
// note skip 0 index so we can use degree of freedom as index
static const float BETA_TABLE[7] = {0,
				    8.82050518214f,
				    12.094592431f,
				    13.9876612368f,
				    16.0875642296f,
				    17.8797700658f,
				    19.6465647819f,
				   };

class BlockLocalPositionEstimator       //: public control::SuperBlock
{
// dynamics:
//
//	x(+) = A * x(-) + B * u(+)
//	y_i = C_i*x
//
// kalman filter
//
//	E[xx'] = P
//	E[uu'] = W
//	E[y_iy_i'] = R_i
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//
//
// input:
// 	ax, ay, az (acceleration NED)
//
// states:
// 	px, py, pz , ( position NED, m)
// 	vx, vy, vz ( vel NED, m/s),
// 	bx, by, bz ( accel bias, m/s^2)
// 	tz (terrain altitude, ASL, m)
//
// measurements:
//
// 	sonar: pz (measured d*cos(phi)*cos(theta))
//
// 	baro: pz
//
// 	flow: vx, vy (flow is in body x, y frame)
//
// 	gps: px, py, pz, vx, vy, vz (flow is in body x, y frame)
//
// 	lidar: pz (actual measured d*cos(phi)*cos(theta))
//
// 	vision: px, py, pz, vx, vy, vz
//
// 	mocap: px, py, pz
//
// 	land (detects when landed)): pz (always measures agl = 0)
//
public:

	// constants
	enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};
	enum {U_ax = 0, U_ay, U_az, n_u};
	enum {Y_baro_z = 0, n_y_baro};
	enum {Y_lidar_z = 0, n_y_lidar};
	enum {Y_flow_vx = 0, Y_flow_vy, n_y_flow};
	enum {Y_sonar_z = 0, n_y_sonar};
	enum {Y_gps_x = 0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz, n_y_gps};
	enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
	enum {Y_mocap_x = 0, Y_mocap_y, Y_mocap_z, n_y_mocap};
	enum {Y_land_vx = 0, Y_land_vy, Y_land_agl, n_y_land};
	enum {POLL_FLOW = 0, POLL_SENSORS, POLL_PARAM, n_poll};
	enum {
		FUSE_GPS = 1 << 0,
		FUSE_FLOW = 1 << 1,
		FUSE_VIS_POS = 1 << 2,
		FUSE_VIS_YAW = 1 << 3,
		FUSE_LAND = 1 << 4,
		FUSE_PUB_AGL_Z = 1 << 5,
		FUSE_FLOW_GYRO_COMP = 1 << 6,
		FUSE_BARO = 1 << 7,
	};

	enum sensor_t {
		SENSOR_BARO = 1 << 0,
		SENSOR_GPS = 1 << 1,
		SENSOR_LIDAR = 1 << 2,
		SENSOR_FLOW = 1 << 3,
		SENSOR_SONAR = 1 << 4,
		SENSOR_VISION = 1 << 5,
		SENSOR_MOCAP = 1 << 6,
		SENSOR_LAND = 1 << 7,
	};

	enum estimate_t {
		EST_XY = 1 << 0,
		EST_Z = 1 << 1,
		EST_TZ = 1 << 2,
	};

	// public methods
	BlockLocalPositionEstimator();
	void update();
	inline float getx(){return _x(0);}
	inline float gety(){return _x(1);}
	inline float getz(){return _x(2);}
	inline float get_ua1(){return _u(0);}
	inline float get_ua2(){return _u(1);}
	inline float get_ua3(){return _u(2);}

	virtual ~BlockLocalPositionEstimator();

private:
	// prevent copy and assignment
	BlockLocalPositionEstimator(const BlockLocalPositionEstimator &);
	BlockLocalPositionEstimator operator=(const BlockLocalPositionEstimator &);

	// methods
	// ----------------------------
	//
	Vector<float, n_x> dynamics(
		float t,
		const Vector<float, n_x> &x,
		const Vector<float, n_u> &u);
	void initP();
	void initSS();
	void updateSSStates();
	void updateSSParams();

	// predict the next state
	void predict();

	void publishLocalPos();

	// baro
	int  baroMeasure(Vector<float, n_y_baro> &y);
	void baroCorrect();
	void baroInit();

	// gps
	int  gpsMeasure(Vector<double, n_y_gps> &y);
	void gpsCorrect();
	void gpsInit();


	// misc
	inline float agl() { return _x(X_tz) - _x(X_z); }
	inline double getDt() { return 0.5; }

	// map projection
	struct map_projection_reference_s _map_ref;

	// reference altitudes
	float _altOrigin;
	bool _altOriginInitialized;
	float _baroAltOrigin;
	float _gpsAltOrigin;

	unsigned char _estimatorInitialized;

	// state space
	Vector<float, n_x>  _x; // state vector
	Vector<float, n_u>  _u; // input vector
	Matrix<float, n_x, n_x>  _P; // state covariance matrix

	matrix::Dcm<float> _R_att;
	Vector3f _eul;

	Matrix<float, n_x, n_x>  _A; // dynamics matrix
	Matrix<float, n_x, n_u>  _B; // input matrix
	Matrix<float, n_u, n_u>  _R; // input covariance
	Matrix<float, n_x, n_x>  _Q; // process noise covariance
};
