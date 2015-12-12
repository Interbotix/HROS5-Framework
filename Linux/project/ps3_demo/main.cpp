/*
* main.cpp
*
*  Created on: 2011. 1. 4.
*      Author: robotis
*/

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>

#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "StatusCheck.h"


#define MOTION_FILE_PATH    ((char *)"../../../Data/motion_4096.bin")

#define INI_FILE_PATH       ((char *)"../../../Data/config.ini")

#define M_INI	((char *)"../../../Data/configSMOOTH.ini")


#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxArbotixPro linux_arbotixpro(U2D_DEV_NAME0);
ArbotixPro arbotixpro(&linux_arbotixpro);
int GetCurrentPosition(ArbotixPro &arbotixpro);
////////////////////////////////////////////
Action::PAGE Page;
Action::STEP Step;
////////////////////////////////////////////
int change_current_dir()
{
	char exepath[1024] = {0};
	int status = 0;
	if (readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
		status = chdir(dirname(exepath));
	return status;
}

int main(int argc, char *argv[])
{
	int trackerSel;
	change_current_dir();
	StatusCheck::m_cur_mode = INITIAL;
	minIni* ini = new minIni(INI_FILE_PATH);
	minIni* ini1 = new minIni(M_INI);
	StatusCheck::m_ini = ini;
	StatusCheck::m_ini1 = ini1;
	StatusCheck::m_power_state = 0;

	//////////////////// Framework Initialize ////////////////////////////
	printf("Framework initialized.\n");

	if (MotionManager::GetInstance()->Initialize(&arbotixpro) == false)
		{
			linux_arbotixpro.SetPortName(U2D_DEV_NAME1);
			if (MotionManager::GetInstance()->Initialize(&arbotixpro) == false)
				{
					printf("Fail to initialize Motion Manager!\n");
					return 0;
				}
		}
	printf("MotionManager initialized.\n");

	Walking::GetInstance()->LoadINISettings(ini);
	printf("Walking configuration loaded.\n");
	usleep(100);
	MotionManager::GetInstance()->LoadINISettings(ini);
	//printf("Offset configuration loaded.");

	MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
	MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
	printf("Action, Head, & Walking MotionModules loaded.\n");

	LinuxMotionTimer linuxMotionTimer;
	linuxMotionTimer.Initialize(MotionManager::GetInstance());
	linuxMotionTimer.Start();
	/////////////////////////////////////////////////////////////////////


	int firm_ver = 0, retry = 0;
	//important but allow a few retries
	while (arbotixpro.ReadByte(JointData::ID_HEAD_PAN, MXDXL::P_VERSION, &firm_ver, 0)  != ArbotixPro::SUCCESS)
		{
			fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
			retry++;
			if (retry >= 3) exit(1); // if we can't do it after 3 attempts its not going to work.
		}

	if (0 < firm_ver && firm_ver < 40)
		{
			Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
			printf("RME Motion File Loaded.\n");
		}
	else
		{
			fprintf(stderr, "Wrong firmware version %d!! \n\n", JointData::ID_HEAD_PAN);
			exit(0);
		}

	//copy page ////////////////
	if (argc > 1 && strcmp(argv[1], "-copy") == 0)
		{
			printf("Page copy -- uses files motion_src.bin and motion_dest.bin\n");
			if (Action::GetInstance()->LoadFile((char *)"../../../Data/motion_src.bin") == false)
				{
					printf("Unable to open source file\n");
					exit(1);
				}
			int k;
			void *page1;

			page1 = malloc(sizeof(Robot::Action::PAGE));
			printf("Page to load:");
			if (scanf("%d", &k) != EOF)
				{
					if (Action::GetInstance()->LoadPage(k, (Robot::Action::PAGE *)page1) == false)
						{
							printf("Unable to load page %d\n", k);
							exit(1);
						}
					if (Action::GetInstance()->LoadFile((char *)"../../../Data/motion_dest.bin") == false)
						{
							printf("Unable to open destination file\n");
							exit(1);
						}
					if (Action::GetInstance()->SavePage(k, (Robot::Action::PAGE *)page1) == false)
						{
							printf("Unable to save page %d\n", k);
							exit(1);
						}
					printf("Completed successfully.\n");
					exit(0);
				}
		}

	Walking::GetInstance()->LoadINISettings(ini);
	MotionManager::GetInstance()->LoadINISettings(ini);

	Walking::GetInstance()->m_Joint.SetEnableBody(false);
	Head::GetInstance()->m_Joint.SetEnableBody(false);
	Action::GetInstance()->m_Joint.SetEnableBody(true);
	MotionManager::GetInstance()->SetEnable(true);




	if (PS3Controller_Start() == 0)
		{
			printf("PS3 controller not installed.\n");
		}

	//determine current position
	StatusCheck::m_cur_mode = GetCurrentPosition(arbotixpro);
	//LinuxActionScript::PlayMP3("../../../Data/mp3/ready.mp3");
	if ((argc > 1 && strcmp(argv[1], "-off") == 0) || (StatusCheck::m_cur_mode == SITTING))
		{
			StatusCheck::m_power_state = 0;
			arbotixpro.DXLPowerOn(false);
			printf("Robot detected in sitting position, powering down Dynamixels.\n");

			//for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
			//	arbotixpro.WriteByte(id, MXDXL::P_TORQUE_ENABLE, 0, 0);
		}
	else
		{
			printf("Robot detected in Standing or Walk Ready Position.\n");
			printf("Sitting Down. Dynamixels remain powered on.\n");
			StatusCheck::m_power_state = 1;
			Action::GetInstance()->Start(15);
			StatusCheck::m_cur_mode = SITTING;
			while (Action::GetInstance()->IsRunning()) usleep(8 * 1000);
		}
	while (1)
		{
			StatusCheck::Check(arbotixpro);

			if (StatusCheck::m_is_started == 0)
				continue;
		}

	return 0;
}

int GetCurrentPosition(ArbotixPro &arbotixpro)
{
	int m = StatusCheck::m_cur_mode, p, j, pos[31];
	int dMaxAngle1, dMaxAngle2, dMaxAngle3;
	double dAngle;
	int rl[6] = { JointData::ID_R_ANKLE_ROLL, JointData::ID_R_ANKLE_PITCH, JointData::ID_R_KNEE, JointData::ID_R_HIP_PITCH, JointData::ID_R_HIP_ROLL, JointData::ID_R_HIP_YAW };
	int ll[6] = { JointData::ID_L_ANKLE_ROLL, JointData::ID_L_ANKLE_PITCH, JointData::ID_L_KNEE, JointData::ID_L_HIP_PITCH, JointData::ID_L_HIP_ROLL, JointData::ID_L_HIP_YAW };

	for (p = 0; p < 31; p++)
		{
			pos[p]	= -1;
		}
	for (p = 0; p < 6; p++)
		{
			if (arbotixpro.ReadWord(rl[p], MXDXL::P_PRESENT_POSITION_L, &pos[rl[p]], 0) != ArbotixPro::SUCCESS)
				{
					printf("Failed to read position %d", rl[p]);
				}
			if (arbotixpro.ReadWord(ll[p], MXDXL::P_PRESENT_POSITION_L, &pos[ll[p]], 0) != ArbotixPro::SUCCESS)
				{
					printf("Failed to read position %d", ll[p]);
				}
		}
	// compare to a couple poses
	// first sitting - page 15
	Action::GetInstance()->LoadPage(15, &Page);
	j = Page.header.stepnum - 1;
	dMaxAngle1 = dMaxAngle2 = dMaxAngle3 = 0;
	for (p = 0; p < 6; p++)
		{
			dAngle = abs(MXDXL::Value2Angle(pos[rl[p]]) - MXDXL::Value2Angle(Page.step[j].position[rl[p]]));
			if (dAngle > dMaxAngle1)
				dMaxAngle1 = dAngle;
			dAngle = abs(MXDXL::Value2Angle(pos[ll[p]]) - MXDXL::Value2Angle(Page.step[j].position[ll[p]]));
			if (dAngle > dMaxAngle1)
				dMaxAngle1 = dAngle;
		}
	// Standing - page 1
	Action::GetInstance()->LoadPage(1, &Page);
	j = Page.header.stepnum - 1;
	for (int p = 0; p < 6; p++)
		{
			dAngle = abs(MXDXL::Value2Angle(pos[rl[p]]) - MXDXL::Value2Angle(Page.step[j].position[rl[p]]));
			if (dAngle > dMaxAngle2)
				dMaxAngle2 = dAngle;
			dAngle = abs(MXDXL::Value2Angle(pos[ll[p]]) - MXDXL::Value2Angle(Page.step[j].position[ll[p]]));
			if (dAngle > dMaxAngle2)
				dMaxAngle2 = dAngle;
		}
	// walkready - page 9
	Action::GetInstance()->LoadPage(9, &Page);
	j = Page.header.stepnum - 1;
	for (int p = 0; p < 6; p++)
		{
			dAngle = abs(MXDXL::Value2Angle(pos[rl[p]]) - MXDXL::Value2Angle(Page.step[j].position[rl[p]]));
			if (dAngle > dMaxAngle3)
				dMaxAngle3 = dAngle;
			dAngle = abs(MXDXL::Value2Angle(pos[ll[p]]) - MXDXL::Value2Angle(Page.step[j].position[ll[p]]));
			if (dAngle > dMaxAngle3)
				dMaxAngle3 = dAngle;
		}
	if (dMaxAngle1 < 20 && dMaxAngle1 < dMaxAngle2 && dMaxAngle1 < dMaxAngle3)
		m = Robot::SITTING;
	if (dMaxAngle2 < 20 && dMaxAngle2 < dMaxAngle1 && dMaxAngle2 < dMaxAngle3)
		m = Robot::ACTION;
	if (dMaxAngle3 < 20 && dMaxAngle3 < dMaxAngle1 && dMaxAngle3 < dMaxAngle2)
		m = Robot::WALK_READY;
	printf("Sitting = %d, Standing = %d, Walk Ready = %d\n", dMaxAngle1, dMaxAngle2, dMaxAngle3);
	printf("Robot is %s\n", m == Robot::ACTION ? "Standing" : m == Robot::WALK_READY ? "Walk Ready" : m == Robot::SITTING ? "Sitting" : "None");
	return m;
}
