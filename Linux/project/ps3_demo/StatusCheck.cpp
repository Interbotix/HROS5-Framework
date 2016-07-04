/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom, Farrell Robotics
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "StatusCheck.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "MotionStatus.h"
#include "MotionManager.h"
#include "LinuxActionScript.h"
#include "PS3Controller.h"
#include "PS3BallFollower.h"
#include "LineFollower.h"
#include "RobotFollower.h"
#include "LinuxDARwIn.h"

using namespace Robot;

int StatusCheck::m_cur_mode     = INITIAL;
int StatusCheck::m_old_btn      = 0;
int StatusCheck::m_is_started   = 0;
int StatusCheck::m_current_walk_speed = FAST_WALK;
int StatusCheck::m_power_state;

bool bLJState = false, bRJState = false;

const char* StatusCheck::SCRIPT_FILE_PATH_TRIANGLE = "action_scripts/Triangle.asc";
const char* StatusCheck::SCRIPT_FILE_PATH_CIRCLE = "action_scripts/Circle.asc";
const char* StatusCheck::SCRIPT_FILE_PATH_CROSS = "action_scripts/Cross.asc";
const char* StatusCheck::SCRIPT_FILE_PATH_SQUARE = "action_scripts/Square.asc";
const char* StatusCheck::SCRIPT_FILE_PATH_R1 = "action_scripts/R1.asc";
const char* StatusCheck::SCRIPT_FILE_PATH_R2 = "action_scripts/R2.asc";
const char* StatusCheck::SCRIPT_FILE_PATH_L1 = "action_scripts/L1.asc";
const char* StatusCheck::SCRIPT_FILE_PATH_L2 = "action_scripts/L2.asc";
const char* StatusCheck::SCRIPT_FILE_PATH_SELECT = "action_scripts/SelectButton.asc";
const char* StatusCheck::SCRIPT_FILE_PATH_START = "action_scripts/StartButton.asc";

minIni* StatusCheck::m_ini;
minIni* StatusCheck::m_ini1;

//#define Southpaw

int StatusCheck::DeadBand( int p_deadband, int p_value )
{
	if ( p_value > 0 )
	{
		if ( p_value - p_deadband > 0 )
		{
			return p_value - p_deadband;
		}
		else return 0;
	}
	else if ( p_value < 0 )
	{
		if ( p_value + p_deadband < 0 )
		{
			return p_value + p_deadband;
		} else return 0;
	}
	return 0;
}

void StatusCheck::Check(ArbotixPro &arbotixpro)
{
	int value = 0;

//////////////////////////////////////////////////////////////////////////////////////
// System Standby Toggle
//////////////////////////////////////////////////////////////////////////////////////
	if (PS3.key.PS != 0)
		{
			if (Walking::GetInstance()->IsRunning() == true)
				{
					Walking::GetInstance()->Stop();
					while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);
				}
			if (ToggleRobotStandby() == 1)
				{
					//LinuxActionScript::PlayMP3("../../../Data/mp3/standby.mp3");
				}
			// wait for key release
			while (PS3.key.PS != 0) usleep(8000);
		}
	if (robotInStandby == 1) return;



//////////////////////////////////////////////////////////////////////////////////////
// IMU AUTO GETUP ROUTINE
//////////////////////////////////////////////////////////////////////////////////////

	if (MotionStatus::FALLEN != STANDUP && (m_cur_mode == WALKING || m_cur_mode == WALK_READY) && m_is_started == 1)
		{
			Walking::GetInstance()->Stop();
			while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);

			Action::GetInstance()->m_Joint.SetEnableBody(true, true);

			if (MotionStatus::FALLEN == FORWARD)
			{
				usleep(1000);
				Action::GetInstance()->Start(70);   // FORWARD GETUP 10
				printf( "Robot has fallen forward.\n");
			}
			else if (MotionStatus::FALLEN == BACKWARD)
			{
				usleep(1000);
				Action::GetInstance()->Start(75);   // BACKWARD GETUP 11
				printf( "Robot has fallen backward.\n");
			}
			while (Action::GetInstance()->IsRunning() == 1) usleep(8000);
			// Go back to Walk Ready
			mWalkReady(arbotixpro);
		}


//////////////////////////////////////////////////////////////////////////////////////
// Start Walk Ready
//////////////////////////////////////////////////////////////////////////////////////
	if (PS3.key.Triangle != 0)
		{
			mWalkReady(arbotixpro);
			while (PS3.key.Triangle != 0) usleep(8000);
		}

//////////////////////////////////////////////////////////////////////////////////////
// Shut it down, sit down.
//////////////////////////////////////////////////////////////////////////////////////
	if (PS3.key.Cross != 0)
		{
			if (m_cur_mode == SITTING)
				{
					if (m_power_state == 1)
						{
							MotionManager::GetInstance()->Reinitialize();
							MotionManager::GetInstance()->SetEnable(false);
							arbotixpro.DXLPowerOn(false);
							printf( "Robot Dynamixel Power Disabled.\n");
							m_power_state = 0;
							m_is_started = 0;
						}
					else
						{
//							MotionManager::GetInstance()->Reinitialize();
//							MotionManager::GetInstance()->SetEnable(false);
							arbotixpro.DXLPowerOn(true);
							printf( "Robot Dynamixel Power Enabled.\n");
							m_power_state = 1;
							m_is_started = 1;
						}
				}
			else
				{
					if (Walking::GetInstance()->IsRunning() == true)
						{
							Walking::GetInstance()->Stop();
							while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);
						}
					m_is_started    = 0;
					m_power_state = 1;
					m_cur_mode      = SITTING;
					LinuxActionScript::m_stop = 1;
					Walking::GetInstance()->m_Joint.SetEnableBody(false);
					Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true);
					while (Action::GetInstance()->Start(15) == false) usleep(8000);
					while (Action::GetInstance()->IsRunning() == true) usleep(8000);
					Head::GetInstance()->m_Joint.SetEnableHeadOnly(true);
				}
			while (PS3.key.Cross != 0) usleep(8000);
		}


//////////////////////////////////////////////////////////////////////////////////////
// Action Script Button Assignment
//////////////////////////////////////////////////////////////////////////////////////

	//Make sure we aren't walking currently.
	if (m_cur_mode == WALK_READY || m_cur_mode == ACTION)
		{


//////////////////////////////////////////////////////////////////////////////////////
// Square Button
//////////////////////////////////////////////////////////////////////////////////////
			if (PS3.key.Square != 0)
				{
					mAction(SCRIPT_FILE_PATH_SQUARE);
					while (PS3.key.Square != 0) usleep(8000);
				}

//////////////////////////////////////////////////////////////////////////////////////
// Circle Button
//////////////////////////////////////////////////////////////////////////////////////
			if (PS3.key.Circle != 0)
				{
					mAction(SCRIPT_FILE_PATH_CIRCLE);
					while (PS3.key.Circle != 0) usleep(8000);
				}

//////////////////////////////////////////////////////////////////////////////////////
// R1 Button
//////////////////////////////////////////////////////////////////////////////////////
			if (PS3.key.R1 != 0)
				{
					mAction(SCRIPT_FILE_PATH_R1);
					while (PS3.key.R1 != 0) usleep(8000);
				}

//////////////////////////////////////////////////////////////////////////////////////
// R2 Button
//////////////////////////////////////////////////////////////////////////////////////
			if (PS3.key.R2 != 0)
				{
					mAction(SCRIPT_FILE_PATH_R2);
					while (PS3.key.R2 != 0) usleep(8000);
				}

//////////////////////////////////////////////////////////////////////////////////////
// L1 Button
//////////////////////////////////////////////////////////////////////////////////////
			if (PS3.key.L1 != 0)
				{
					mAction(SCRIPT_FILE_PATH_L1);
					while (PS3.key.L1 != 0) usleep(8000);
				}

//////////////////////////////////////////////////////////////////////////////////////
// L2 Button
//////////////////////////////////////////////////////////////////////////////////////
			if (PS3.key.L2 != 0)
				{
					mAction(SCRIPT_FILE_PATH_L2);
					while (PS3.key.L2 != 0) usleep(8000);
				}


//////////////////////////////////////////////////////////////////////////////////////
// Start Button
//////////////////////////////////////////////////////////////////////////////////////
			if (PS3.key.Start != 0)
				{
					mAction(SCRIPT_FILE_PATH_START);
					while (PS3.key.Select != 0) usleep(8000);
				}

//////////////////////////////////////////////////////////////////////////////////////
// Select Button
//////////////////////////////////////////////////////////////////////////////////////
			if (PS3.key.Select != 0)
				{
					mAction(SCRIPT_FILE_PATH_SELECT);
					while (PS3.key.Select != 0) usleep(8000);
				}
		}

//////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////
// START WALKING GAIT ENGINE
//////////////////////////////////////////////////////////////////////////////////////
	if (Walking::GetInstance()->IsRunning() == false && PS3.key.Up != 0)
		{
			if (m_cur_mode == WALK_READY)
				{
					printf("\r");
					fprintf(stderr, "Starting Walking Gait.\n");
					m_cur_mode = WALKING;
					Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
					Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
					Walking::GetInstance()->X_MOVE_AMPLITUDE = 0;
					Walking::GetInstance()->Y_MOVE_AMPLITUDE = 0;
					Walking::GetInstance()->A_MOVE_AMPLITUDE = 0;
					Walking::GetInstance()->Start();
				}
		}

//////////////////////////////////////////////////////////////////////////////////////
// STOP WALKING GAIT ENGINE
//////////////////////////////////////////////////////////////////////////////////////

	if (Walking::GetInstance()->IsRunning() == true && PS3.key.Down != 0)
		{
			printf("\r");
			fprintf(stderr, "Stopping Walking Gait.\n");
			m_cur_mode = WALK_READY;
			Walking::GetInstance()->Stop();
			while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);
		}
//////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////
// PS3 R/C Control code
//////////////////////////////////////////////////////////////////////////////////////


	if (Walking::GetInstance()->IsRunning() == true)
		{
			int rx = 128, ry = 128, rxx = 128; 
			int dead_band = 7;
			double FBStep = 0, RLTurn = 0, RLStep = 0, xd, yd, xxd;
			static double speedAdjSum = 0;

#ifdef Southpaw
			rx = -(PS3.key.RJoyX - 128);
			ry = -(PS3.key.RJoyY - 128);
#else
			rx = -(PS3.key.LJoyX - 128);
			ry = -(PS3.key.LJoyY - 128);
			rxx = (PS3.key.RJoyX -128);
#endif

			rx = DeadBand( dead_band, rx );
			ry = DeadBand( dead_band, ry );
			rxx = DeadBand( dead_band, rxx);
//			fprintf(stderr, " (X:%d Y:%d RXX:%d)\n", rx, ry, rxx);

			if (abs(rx) > 0 || abs(ry) > 0 || abs(rxx) > 0 )
				{
					xd = (double)rx / 256;
					yd = (double)ry / 256;
					xxd = (double)rxx / 256;
//					RLTurn = 60 * xd;
					RLTurn = -10 * xxd;

					FBStep = 70 * yd;
					RLStep = 80 * xxd;
//				fprintf(stderr, " (xxd:%.1f)\n", xxd);
//				Walking::GetInstance()->HIP_PITCH_OFFSET = Walking::GetInstance()->HIP_PITCH_OFFSET_START + yd / 2;
					if (FBStep < 0)
						{
							FBStep = 45 * yd;
						}
					speedAdjSum += yd;

					if (speedAdjSum > Walking::GetInstance()->UPPER_VELADJ_LIMIT)
						{
							speedAdjSum = Walking::GetInstance()->UPPER_VELADJ_LIMIT;
						}

					else if (speedAdjSum < Walking::GetInstance()->LOWER_VELADJ_LIMIT)
						{
							speedAdjSum = Walking::GetInstance()->LOWER_VELADJ_LIMIT;
						}
					else
						{
							speedAdjSum = 0;
						}
				}
			Walking::GetInstance()->speedAdj = speedAdjSum;
//			Walking::GetInstance()->X_OFFSET = Walking::GetInstance()->X_OFFSET_START - speedAdjSum;
//			double hip_offset = Walking::GetInstance()->HIP_PITCH_OFFSET;
//			fprintf(stderr, " (hip offset:%.1f)\n", hip_offset);
			Walking::GetInstance()->X_MOVE_AMPLITUDE = FBStep;
			Walking::GetInstance()->Y_MOVE_AMPLITUDE = RLStep;
			Walking::GetInstance()->A_MOVE_AMPLITUDE = RLTurn;
			fprintf(stderr, " (FB:%.1f RLTurn:%.1f RLStep:%.1f)\n", FBStep, RLTurn, RLStep);
		}
	else //things only done in auto mode
		{
		}
//////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////
// PS3 Head Control
//////////////////////////////////////////////////////////////////////////////////////
/*
	if ((PS3BallFollower::GetInstance()->bHeadAuto == false && (m_cur_mode == WALK_READY || m_cur_mode == SITTING || m_cur_mode == WALKING)) )
		{
			int lx = 128, ly = 128;
			int dead_band = 7;
			double pan, tilt;
			pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
			tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
			Point2D pos = Point2D(pan, tilt);

#ifdef Southpaw
			lx = -(PS3.key.LJoyX - 128);
			ly = -(PS3.key.LJoyY - 128);
#else
			lx = -(PS3.key.RJoyX - 128);
			ly = -(PS3.key.RJoyY - 128);
#endif

			lx = DeadBand( dead_band, lx );
			ly = DeadBand( dead_band, ly );

			if (abs(lx) > 0 || abs(ly) > 0)
			{
				pos.X = pan + 0.2 * Camera::VIEW_V_ANGLE * lx / 256;
				pos.Y = tilt + 0.2 * Camera::VIEW_H_ANGLE * ly / 256;

				Head::GetInstance()->MoveByAngle(pos.X, pos.Y);
				//Head::GetInstance()->MoveTracking(pos);
			}
		}
*/


//////////////////////////////////////////////////////////////////////////////////////


	/*
	//////////////////////////////////////////////////////////////////////////////////////
		// toggle head from auto to not

		if(PS3.key.LeftHat != 0)
			{
			if(bLJState == true)
				{
				bLJState = false;
				if(m_cur_mode == SOCCER)
					PS3BallFollower::GetInstance()->bHeadAuto = false;
				else if(m_cur_mode == SITTING)
					LineFollower::GetInstance()->bHeadAuto = false;
				else if(m_cur_mode == LINE_FOLLOWING)
					LineFollower::GetInstance()->bHeadAuto = false;
				else if(m_cur_mode == ROBOT_FOLLOWING)
					RobotFollower::GetInstance()->bHeadAuto = false;

				//double pan,tilt;
				//pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);
				//tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
				//Head::GetInstance()->MoveByAngle(pan,tilt);
				}
			else
				{
				bLJState = true;
				if(m_cur_mode == SOCCER)
					{
					if(PS3BallFollower::GetInstance()->bFullAuto == true)
						PS3BallFollower::GetInstance()->bHeadAuto = true;
					else
						PS3BallFollower::GetInstance()->bHeadAuto = false;
					}
				else if(m_cur_mode == SITTING)
					{
					LineFollower::GetInstance()->bHeadAuto = true;
					}
				else if(m_cur_mode == LINE_FOLLOWING)
					{
					if(LineFollower::GetInstance()->bFullAuto == true)
						LineFollower::GetInstance()->bHeadAuto = true;
					else
						LineFollower::GetInstance()->bHeadAuto = false;
					}
				else if(m_cur_mode == ROBOT_FOLLOWING)
					{
					if(RobotFollower::GetInstance()->bFullAuto == true)
						RobotFollower::GetInstance()->bHeadAuto = true;
					else
						RobotFollower::GetInstance()->bHeadAuto = false;
					}
				}
			PS3Vibrate();
			// wait for key release
			while(PS3.key.LeftHat != 0) usleep(8000);
			}

	*/

}

void StatusCheck::mWalkReady(ArbotixPro &arbotixpro)
{
	if (LinuxActionScript::m_is_running == 0)
		{
			if (m_is_started == 0)
				{
					arbotixpro.DXLPowerOn(true);
				}
			if (Walking::GetInstance()->IsRunning() == true)
				{
					Walking::GetInstance()->Stop();
					while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);
				}
			int lastMode = m_cur_mode;
			m_cur_mode = WALK_READY;
			printf("Robot is in WALK_READY state.\n");
			MotionManager::GetInstance()->Reinitialize();
			MotionManager::GetInstance()->SetEnable(true);
			m_is_started = 1;
			bLJState = bRJState = false;
			Head::GetInstance()->m_Joint.SetEnableBody(false);
			Walking::GetInstance()->m_Joint.SetEnableBody(false);
			Action::GetInstance()->m_Joint.SetEnableBody(true);

			Action::GetInstance()->Start(9); //9 WALK READY STANCE
			while (Action::GetInstance()->IsRunning() == true) usleep(8000);
			usleep(500);
			Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true);
			Action::GetInstance()->m_Joint.SetEnableBody(false);
			usleep(100);
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true);
		}
}

void StatusCheck::mAction(const char* scriptfilepath)
{
	if (LinuxActionScript::m_is_running == 0)
		{
			m_cur_mode = ACTION;
			printf("Robot is in ACTION state.\n");
			LinuxActionScript::m_stop = 0;
			m_is_started = 1;
			Head::GetInstance()->m_Joint.SetEnableBody(false);
			Walking::GetInstance()->m_Joint.SetEnableBody(false);
			Action::GetInstance()->m_Joint.SetEnableBody(true);
			LinuxActionScript::ScriptStart(scriptfilepath);
			while (Action::GetInstance()->IsRunning() == true) usleep(8000);
		}
}

void StatusCheck::mPlay(int motion_page, int mode, int wait)
{
	Walking::GetInstance()->Stop();
	while (Walking::GetInstance()->IsRunning() == 1) usleep(8000);
	m_cur_mode = mode;
	MotionManager::GetInstance()->Reinitialize();
	MotionManager::GetInstance()->SetEnable(true);
	m_is_started = 1;

	Action::GetInstance()->m_Joint.SetEnableBody(true, true);

	Action::GetInstance()->Start(motion_page);
	if (wait == WAIT)
		{
			while (Action::GetInstance()->IsRunning() == true) usleep(8000);
			// if (mode != SITTING && mode != STAIRS)
			// 	{
			// 		Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			// 		Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			// 	}
		}
	return;
}
