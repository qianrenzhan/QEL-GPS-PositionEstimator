#include <iostream>
#include<fstream>
#include "LocalPositionEstimator/BlockLocalPositionEstimator.h"
#include <windows.h> 
#include "gyro/stdafx.h"
#include "gyro/Com.h"
#include "gyro/JY901.h"
using namespace std;


DWORD WINAPI mavThreadRead(LPVOID lpParam);
DWORD WINAPI mavThreadWrite(LPVOID lpParam);