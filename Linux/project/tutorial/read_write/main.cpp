#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"


using namespace Robot;

int main()
{
	printf( "\n===== Read/Write Tutorial for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxArbotixPro linux_arbotixpro("/dev/ttyUSB0");
	ArbotixPro arbotixpro(&linux_arbotixpro);
	if (arbotixpro.Connect() == false)
		{
			printf("Fail to connect Arbotix Pro!\n");
			return 0;
		}
	/////////////////////////////////////////////////////////////////////


	return 0;
}
