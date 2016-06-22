#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"



using namespace Robot;

int main()
{
	printf( "\n===== UM7 Demo for DARwIn =====\n\n");

	//////////////////////////// Framework Initialize ////////////////////////////
	LinuxUM7 	linux_um7( "/dev/um7" );
	UM7 		um7( &linux_um7 );

	if ( um7.Connect() == false )
	{
		printf( "Fail to connect UM7!\n" );
		return -1;
	}
	else
	{
		printf( "Success! UM7 connected.\n" );
	}
	//////////////////////////////////////////////////////////////////////////////

	while ( true )
	{
		if ( um7.request_angles() == true )
		{
			printf( "UM7 Pitch: %0.2f, Roll: %0.2f, Yaw: %0.2f\n", um7.pitch, um7.roll, um7.yaw );
		}
		usleep( 100000 );
	}
	
	return 0;
}
