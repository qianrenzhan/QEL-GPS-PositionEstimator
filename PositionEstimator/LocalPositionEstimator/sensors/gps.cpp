#include <fstream>
#include "../BlockLocalPositionEstimator.h"
#include <windows.h> 
#include "../../gyro/JY901.h"
using namespace std;
#pragma warning(disable:4244)

float  testtemp = 116;

//写文件
ofstream fout2;

void BlockLocalPositionEstimator::gpsInit()
{
	fout2.open("gpsint.txt");           //不能有空格

	cout<<"正在接收gps数据...";
	cout<<"这个地方会卡住，因为没有GPS数据，可以把gps.cpp文件中第19行到23行注释掉"<<endl;
	//while(JY901.stcLonLat.lLat <0.01)
	//{
	//	cout<<".";
	//	Sleep(1000);
	//}
	cout<<endl;
	cout<<"GPS找到数据,正在进行初始值记录...";
	int count = 20;
	double gpsLatsum = 0;
	double gpsLonsum = 0;
	float gpsAltsum = 0;
	while(count--)
	{	
		cout<<".";
		gpsLatsum += JY901.stcLonLat.lLat/100000.0;				//============================纬度
		gpsLonsum += JY901.stcLonLat.lLon/100000.0;				//============================经度
		gpsAltsum += JY901.stcGPSV.sGPSHeight/10.0;					//============================高度
		Sleep(150);
	}
	cout<<endl;
	double gpsLat = gpsLatsum/20.0;
	double gpsLon = gpsLonsum/20.0;
	float gpsAlt = gpsAltsum/20.0;
	cout<<"GPS初始值记录有效，分别是"<<gpsLat<<" "<<gpsLon<<" "<<gpsAlt<<endl;
	_gpsAltOrigin = gpsAlt;// + _x(X_z);

	fout2<<gpsLat<<" "<< gpsLon<<" "<<gpsAlt<<endl;

	double gpsLatOrigin = 0;
	double gpsLonOrigin = 0;
	// reproject at current coordinates
	map_projection_init(&_map_ref, gpsLat, gpsLon);
	// find origin
	map_projection_reproject(&_map_ref, -_x(X_x), -_x(X_y), &gpsLatOrigin, &gpsLonOrigin);
	// reinit origin
	map_projection_init(&_map_ref, gpsLatOrigin, gpsLonOrigin);

	fout2<<gpsLatOrigin<<" "<< gpsLonOrigin<<endl;
	fout2<<flush;
	fout2.close();
	// always override alt origin on first GPS to fix
	// possible baro offset in global altitude at init
	_altOrigin = _gpsAltOrigin;
	_altOriginInitialized = true;

	//cout<<"GPS初始化完毕，是否继续？"<<endl;
	char x00 = 'n';
	while(x00!='y')
	{
		cout<<"GPS初始化完毕，是否继续(y/n)？"<<endl;
		cin>>x00;
	}
}

int BlockLocalPositionEstimator::gpsMeasure(Vector<double, n_y_gps> &y)
{
	// gps measurement
	y.setZero();
	y(0) = (double)JY901.stcLonLat.lLat/100000.0;//_sub_gps.get().lat * 1e-7;          //============================纬度
	y(1) = (double)JY901.stcLonLat.lLon/100000.0;//_sub_gps.get().lon * 1e-7;          //============================精度
	y(2) = (double)JY901.stcGPSV.sGPSHeight/10.0;//_sub_gps.get().alt * 1e-3;     //============================高度
	y(3) = 0;//_sub_gps.get().vel_n_m_s;     //============================北
	y(4) = 0;//_sub_gps.get().vel_e_m_s;     //============================东
	y(5) = 0;//_sub_gps.get().vel_d_m_s;     //============================地

	return 1;
}

void BlockLocalPositionEstimator::gpsCorrect()
{
	// measure
	Vector<double, n_y_gps> y_global;

	if (gpsMeasure(y_global) != 1) { return; }

	// gps measurement in local frame
	double  lat = y_global(0);
	double  lon = y_global(1);
	float  alt = y_global(2);
	float px = 0;
	float py = 0;
	float pz = -(alt - _gpsAltOrigin);
	map_projection_project(&_map_ref, lat, lon, &px, &py);
	Vector<float, 6> y;
	y.setZero();
	y(0) = px;
	y(1) = py;
	y(2) = pz;
	y(3) = y_global(3);
	y(4) = y_global(4);
	y(5) = y_global(5);

	// gps measurement matrix, measures position and velocity
	Matrix<float, n_y_gps, n_x> C;
	C.setZero();
	C(Y_gps_x, X_x) = 1;
	C(Y_gps_y, X_y) = 1;
	C(Y_gps_z, X_z) = 1;
	C(Y_gps_vx, X_vx) = 1;
	C(Y_gps_vy, X_vy) = 1;
	C(Y_gps_vz, X_vz) = 1;

	// gps covariance matrix
	SquareMatrix<float, n_y_gps> R;
	R.setZero();

	// default to parameter, use gps cov if provided
	//float var_xy = _gps_xy_stddev.get() * _gps_xy_stddev.get();
	float var_xy = 1.0 * 1.0;
	//float var_z = _gps_z_stddev.get() * _gps_z_stddev.get();
	float var_z = 3.0 * 3.0;
	//float var_vxy = _gps_vxy_stddev.get() * _gps_vxy_stddev.get();
	float var_vxy = 0.25 * 0.25;
	//float var_vz = _gps_vz_stddev.get() * _gps_vz_stddev.get();
	float var_vz = 0.25 * 0.25;

	R(0, 0) = var_xy;
	R(1, 1) = var_xy;
	R(2, 2) = var_z;
	R(3, 3) = var_vxy;
	R(4, 4) = var_vxy;
	R(5, 5) = var_vz;

	// get delayed x
	unsigned char  i_hist = 0;

	//if (getDelayPeriods(_gps_delay.get(), &i_hist)  < 0) { return; }

	//Vector<float, n_x> x0 = _xDelay.get(i_hist);
	Vector<float, n_x> x0 = _x;

	// residual
	Vector<float, n_y_gps> r = y - C * x0;


	Matrix<float, n_y_gps, n_y_gps> S_I = inv<float, 6>(C * _P * C.transpose() + R);

	// kalman filter correction always for GPS
	Matrix<float, n_x, n_y_gps> K = _P * C.transpose() * S_I;
	Vector<float, n_x> dx = K * r;
	_x += dx;
	_P -= K * C * _P;
}
