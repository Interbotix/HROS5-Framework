#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"



using namespace Robot;

int main()
{
	printf( "\n===== UM7 Demo for DARwIn =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxUM7 linux_um7("/dev/ttyUSB1");
	UM7 um7(&linux_um7);
	if (um7.Connect() == false)
		{
			printf("Fail to connect UM7!\n");
			return 0;
		}
	else
		{
			printf("Success! UM7 connected.");
		}
	/////////////////////////////////////////////////////////////////////

	return 0;
}
