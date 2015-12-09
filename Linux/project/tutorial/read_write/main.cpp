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

	int value, value1 = 0;
	arbotixpro.WriteWord(JointData::ID_R_SHOULDER_PITCH, MXDXL::P_TORQUE_ENABLE, 0, 0);
	arbotixpro.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MXDXL::P_TORQUE_ENABLE, 0, 0);
	arbotixpro.WriteWord(JointData::ID_R_ELBOW,          MXDXL::P_TORQUE_ENABLE, 0, 0);

	arbotixpro.WriteByte(JointData::ID_L_SHOULDER_PITCH, MXDXL::P_P_GAIN, 1, 0);
	arbotixpro.WriteByte(JointData::ID_L_SHOULDER_ROLL,  MXDXL::P_P_GAIN, 1, 0);
	arbotixpro.WriteByte(JointData::ID_L_ELBOW,          MXDXL::P_P_GAIN, 1, 0);

	while (1)
		{
			printf("\r");

			printf("GFB:");
			if (arbotixpro.ReadWord(ArbotixPro::P_GYRO_Y_L, &value, 0) == ArbotixPro::SUCCESS)
				printf("%3d", value);
			else
				printf("---");

			printf(" GRL:");
			if (arbotixpro.ReadWord(ArbotixPro::P_GYRO_X_L, &value, 0) == ArbotixPro::SUCCESS)
				printf("%3d", value);
			else
				printf("---");

			printf(" AFB:");
			if (arbotixpro.ReadWord(ArbotixPro::P_ACCEL_Y_L, &value, 0) == ArbotixPro::SUCCESS)
				printf("%3d", value);
			else
				printf("----");

			printf(" ARL:");
			if (arbotixpro.ReadWord(ArbotixPro::P_ACCEL_X_L, &value, 0) == ArbotixPro::SUCCESS)
				printf("%3d", value);
			else
				printf("----");

			printf(" BTN:");
			if (arbotixpro.ReadWord(ArbotixPro::P_BUTTON, &value, 0) == ArbotixPro::SUCCESS)
				printf("%1d", value);
			else
				printf("----");

			printf(" ID[%d]:", JointData::ID_R_SHOULDER_PITCH);
			if (arbotixpro.ReadWord(JointData::ID_R_SHOULDER_PITCH, MXDXL::P_PRESENT_POSITION_L, &value, 0) == ArbotixPro::SUCCESS)
				{
					printf("%4d", value);
					arbotixpro.WriteWord(JointData::ID_L_SHOULDER_PITCH, MXDXL::P_GOAL_POSITION_L, MXDXL::GetMirrorValue(value), 0);
				}
			else
				printf("----");

			printf(" ID[%d]:", JointData::ID_R_SHOULDER_ROLL);
			if (arbotixpro.ReadWord(JointData::ID_R_SHOULDER_ROLL, MXDXL::P_PRESENT_POSITION_L, &value, 0) == ArbotixPro::SUCCESS)
				{
					printf("%4d", value);
					arbotixpro.WriteWord(JointData::ID_L_SHOULDER_ROLL, MXDXL::P_GOAL_POSITION_L, MXDXL::GetMirrorValue(value), 0);
				}
			else
				printf("----");

			printf(" ID[%d]:", JointData::ID_R_ELBOW);
			if (arbotixpro.ReadWord(JointData::ID_R_ELBOW, MXDXL::P_PRESENT_POSITION_L, &value, 0) == ArbotixPro::SUCCESS)
				{
					printf("%4d", value);
					arbotixpro.WriteWord(JointData::ID_L_ELBOW, MXDXL::P_GOAL_POSITION_L, MXDXL::GetMirrorValue(value), 0);
				}
			else
				printf("----");

			if (arbotixpro.ReadWord(ArbotixPro::P_LED_HEAD_L, &value, 0) == ArbotixPro::SUCCESS)
				{
					if (value == 0x7FFF)
						value = 0;
					else
						value++;

					arbotixpro.WriteWord(ArbotixPro::P_LED_HEAD_L, value, 0);
				}

			//if(arbotixpro.ReadWord(ArbotixPro::P_LED_EYE_L, &value, 0) == ArbotixPro::SUCCESS)
			{
				//if(value1 == 0)
				//	value = 0x7FFF;
				//else
				//	value1++;
				//printf(" :Eye LED %02x",value);
				arbotixpro.WriteWord(ArbotixPro::P_LED_EYE_L, arbotixpro.MakeColor(14, 14, 0), 0);
			}

			usleep(50000);
		}

	return 0;
}
