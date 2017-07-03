#include "stdafx.h"
#include "Com.h"
#include "JY901.h"

int main(void)
{
	char chrBuffer[2000];
	unsigned short usLength = 0;
	unsigned long ulBaund = 9600, ulComNo = 3;
	signed char cResult = 1;
	printf("Please enter the serial number:\r\nCom = ");
	scanf("%ld", &ulComNo);
	printf("Please enter the baud rate : (9600¡¢115200 or else )\r\nBaud = ");
	scanf("%ld", &ulBaund);
	printf("Wait for the serial port to open...\r\n");
	while (cResult != 0)
	{
		cResult = OpenCOMDevice(ulComNo, ulBaund);
	}

	while (1)
	{
		usLength = CollectUARTData(ulComNo, chrBuffer);
		if (usLength > 0)
		{
			JY901.CopeSerialData(chrBuffer, usLength);
		}

		system("cls");

		printf("Acc:%.3f %.3f %.3f\r\n", (float)JY901.stcAcc.a[0] / 32768 * 16, (float)JY901.stcAcc.a[1] / 32768 * 16, (float)JY901.stcAcc.a[2] / 32768 * 16);

		printf("Gyro:%.3f %.3f %.3f\r\n", (float)JY901.stcGyro.w[0] / 32768 * 2000, (float)JY901.stcGyro.w[1] / 32768 * 2000, (float)JY901.stcGyro.w[2] / 32768 * 2000);

		printf("Angle:%.3f %.3f %.3f\r\n", (float)JY901.stcAngle.Angle[0] / 32768 * 180, (float)JY901.stcAngle.Angle[1] / 32768 * 180, (float)JY901.stcAngle.Angle[2] / 32768 * 180);

		Sleep(100);
	}
	return 0;
}

