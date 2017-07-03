/*
���ߣ���ΰ��
ʱ�䣺2017��7��1��
���ܣ��������������ݺ�GPS���ݵ��ںϣ��õ�λ�����ꡣ
*/
#include"main.h"

//��ʱ�����߳�
HANDLE hTimer = NULL;
HANDLE hThreadRead = NULL;
HANDLE hThreadWrite = NULL;
LARGE_INTEGER liDueTime;
int totalNum = 0;

//����
char chrBuffer[2000];
unsigned short usLength = 0;
unsigned long ulBaund = 9600, ulComNo = 4;
signed char cResult = 1;

//д�ļ�
ofstream fout;

//�ж�ѭ������
bool isrun = 0;
bool isinit = 0;

//����ʱ��
int testtime = 50;

int main()
{
	
	hTimer = CreateWaitableTimer(NULL, TRUE, "TestWaitableTimer");
	if (hTimer)
	{
		printf("��ʱ������\r\n");
	}
	liDueTime.QuadPart = -10000000;     //1s
	SetWaitableTimer(hTimer, &liDueTime, 0, NULL, NULL, 0);

	hThreadRead = CreateThread(NULL, 0,
		mavThreadRead, NULL, 0, NULL);
	hThreadWrite = CreateThread(NULL, 0,
		mavThreadWrite, NULL, 0, NULL);
	
	while(1);
	return 0;
}

DWORD WINAPI mavThreadRead(LPVOID lpParam)
{
	cout<<"�����Ǵ��ڳ�ʼ��..."<<endl;
	cout<<"����ط��Ῠס����Ϊû�д������ӣ���������һ�������豸��Ȼ����main.cpp���޸ĵ�18�еĴ��ں�"<<endl;
	while (cResult != 0)
	{
		cout<<".";
		cResult = OpenCOMDevice(ulComNo, ulBaund);
		Sleep(1000);
	}
	cout<<endl;
	cout<<"�����ǳ�ʼ�����..."<<endl;

	isinit =  1;
	while (1)
	{
		usLength = CollectUARTData(ulComNo, chrBuffer);
		if (usLength > 0)
		{
			JY901.CopeSerialData(chrBuffer, usLength);
		}

		//system("cls");

		//printf("Time:20%d-%d-%d %d:%d:%.3f\r\n",(short)JY901.stcTime.ucYear,(short)JY901.stcTime.ucMonth,
		//	(short)JY901.stcTime.ucDay,(short)JY901.stcTime.ucHour,(short)JY901.stcTime.ucMinute,(float)JY901.stcTime.ucSecond+(float)JY901.stcTime.usMiliSecond/1000);

		//printf("Acc:%.3f %.3f %.3f\r\n", (float)JY901.stcAcc.a[0] / 32768 * 16, (float)JY901.stcAcc.a[1] / 32768 * 16, (float)JY901.stcAcc.a[2] / 32768 * 16);

		//printf("Gyro:%.3f %.3f %.3f\r\n", (float)JY901.stcGyro.w[0] / 32768 * 2000, (float)JY901.stcGyro.w[1] / 32768 * 2000, (float)JY901.stcGyro.w[2] / 32768 * 2000);

		//printf("Angle:%.3f %.3f %.3f\r\n", (float)JY901.stcAngle.Angle[0] / 32768 * 180, (float)JY901.stcAngle.Angle[1] / 32768 * 180, (float)JY901.stcAngle.Angle[2] / 32768 * 180);

		//printf("Pressure:%lx Height%.2f\r\n",JY901.stcPress.lPressure,(float)JY901.stcPress.lAltitude/100);

		//printf("DStatus:%d %d %d %d\r\n",JY901.stcDStatus.sDStatus[0],JY901.stcDStatus.sDStatus[1],JY901.stcDStatus.sDStatus[2],JY901.stcDStatus.sDStatus[3]);

		//printf("Longitude:%ldDeg%.5fm Lattitude:%ldDeg%.5fm\r\n",JY901.stcLonLat.lLon/10000000,(double)(JY901.stcLonLat.lLon % 10000000)/1e5,JY901.stcLonLat.lLat/10000000,(double)(JY901.stcLonLat.lLat % 10000000)/1e5);

		//printf("GPSHeight:%.1fm GPSYaw:%.1fDeg GPSV:%.3fkm/h\r\n\r\n",(float)JY901.stcGPSV.sGPSHeight/10,(float)JY901.stcGPSV.sGPSYaw/10,(float)JY901.stcGPSV.lGPSVelocity/1000);
		
		Sleep(50);      //20hz
	}
	return 0;
}


DWORD WINAPI mavThreadWrite(LPVOID lpParam)
{
	while(!isinit);

	//λ�ù�����
	BlockLocalPositionEstimator lpe;       //���캯���н��г�ʼ��

	time_t rawtime;
	char filenametemp[15];
	cout<<"������Ҫ�������ݵ��ļ�����";
	cin>>filenametemp;
	strcat(filenametemp,".txt");
	char* filename = filenametemp;
	fout.open(filename);           //�����пո�
	isrun = 1;
	while (isrun)
	{
		if (WaitForSingleObject(hTimer, INFINITE) != WAIT_OBJECT_0)
		{
			printf("1�붨ʱ��������\r\n");
		}
		else
		{
			time(&rawtime);
			liDueTime.QuadPart = -500000;     //0.05s                -5000000����0.5s��      //20hz
			SetWaitableTimer(hTimer, &liDueTime, 0, NULL, NULL, 0);
			totalNum++;
			cout << totalNum << endl;
			if(totalNum<testtime*50)           //100s    5000������
			{
				lpe.update();
			}
			//cout << "at "<<ctime(&rawtime) << endl;
			float x,y,z;
			float a1,a2,a3;
			x = lpe.getx();
			y = lpe.gety();
			z = lpe.getz();
			a1 = lpe.get_ua1();
			a2 = lpe.get_ua2();
			a3 = lpe.get_ua3();

			//system("cls");
			//printf("Pos:%.3f %.3f %.3f\r\n", x, y, z);
			if(totalNum<testtime*50)
			{
				//�������γ�ȣ�GPS�߶ȣ����ٶȵ�ֵ���˲������
				fout.precision(10);
				fout<<(double)JY901.stcLonLat.lLat/100000.0<<" "<< (double)JY901.stcLonLat.lLon/100000.0<<" "<<(double)JY901.stcGPSV.sGPSHeight/10.0<<" ";
				fout<<(double)JY901.stcAngle.Angle[0] / 32768 * 3.1415<<" "<<-(double)JY901.stcAngle.Angle[1] / 32768.0 * 3.1415<<" "<<(double)JY901.stcAngle.Angle[2] / 32768.0 * 3.1415<<" ";
				fout<<(float)JY901.stcAcc.a[0] / 32768.0 * 16<<" "<<-(float)JY901.stcAcc.a[1] / 32768.0 * 16<<" "<<-(float)JY901.stcAcc.a[2] / 32768.0 * 16<<" ";
				fout<<x<<" "<<y<<" "<<z<<endl;
			}
			else if(totalNum ==testtime*50)
			{
				fout<<flush;
				fout.close();
				printf("�ļ�������\r\n");
			}
		}
	}
	return 0;
}