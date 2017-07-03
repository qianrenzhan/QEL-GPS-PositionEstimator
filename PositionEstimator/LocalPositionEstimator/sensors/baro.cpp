#include "../BlockLocalPositionEstimator.h"
#include "../../matrix/math.hpp"

void BlockLocalPositionEstimator::baroInit()
{
	_baroAltOrigin = 2;		//========================   baro_alt_meter
	if (!_altOriginInitialized) 
	{
		_altOriginInitialized = true;
		_altOrigin = _baroAltOrigin;
	}
}

int BlockLocalPositionEstimator::baroMeasure(Vector<float, n_y_baro> &y)
{
	//measure
	y.setZero();
	y(0) = 2;                  //========================   baro_alt_meter
	return 1;
}

void BlockLocalPositionEstimator::baroCorrect()
{
	// measure
	Vector<float, n_y_baro> y;

	if (baroMeasure(y) != 1) { return; }

	// subtract baro origin alt
	y -= _baroAltOrigin;

	// baro measurement matrix
	Matrix<float, n_y_baro, n_x> C;
	C.setZero();
	C(Y_baro_z, X_z) = -1; // measured altitude, negative down dir.

	Matrix<float, n_y_baro, n_y_baro> R;
	R.setZero();
	//R(0, 0) = _baro_stddev.get() * _baro_stddev.get();
	R(0, 0) = 300000 * 300000;

	// residual
	Matrix<float, n_y_baro, n_y_baro> S_I =
		inv<float, n_y_baro>((C * _P * C.transpose()) + R);
	Vector<float, n_y_baro> r = y - (C * _x);


	// kalman filter correction always
	Matrix<float, n_x, n_y_baro> K = _P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	_P -= K * C * _P;
}

