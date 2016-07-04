/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom, Farrell Robotics, A. Dresner
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
#include "Camera.h"

using namespace Robot;

int StatusCheck::m_cur_mode     = READY;
int StatusCheck::m_old_btn      = 0;
int StatusCheck::m_is_started   = 0;
int StatusCheck::m_current_walk_speed = FAST_WALK;

bool bLJState=false,bRJState=false;
minIni* StatusCheck::m_ini;
minIni* StatusCheck::m_ini1;

void StatusCheck::mPlay(int motion_page, int mode, int wait)
{
  Walking::GetInstance()->Stop();
	while(Walking::GetInstance()->IsRunning() == 1) usleep(8000);
	m_cur_mode = mode;      
	MotionManager::GetInstance()->Reinitialize();
  MotionManager::GetInstance()->SetEnable(true);
  m_is_started = 1;
  
  Action::GetInstance()->m_Joint.SetEnableBody(true, true);

  Action::GetInstance()->Start(motion_page);
  if(wait == WAIT)
		{
		while(Action::GetInstance()->IsRunning() == true) usleep(8000);
	
		if(mode != SITTING && mode != STAIRS)
			{
			Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
			Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			}
		}
	return;		
}



void StatusCheck::Check(ArbotixPro &arbotixpro)
{
	int value=0;
	//toggle stnadby		
      if(m_cur_mode == SOCCER)
        {
				MotionManager::GetInstance()->ResetGyroCalibration();
				while(1)
					{
					if(MotionManager::GetInstance()->GetCalibrationStatus() == 1)
						{
						LinuxActionScript::PlayMP3("../../../Data/mp3/Sensor calibration complete.mp3");
						break;
						}
					else if(MotionManager::GetInstance()->GetCalibrationStatus() == -1)
						{
						LinuxActionScript::PlayMP3Wait("../../../Data/mp3/Sensor calibration fail.mp3");
						MotionManager::GetInstance()->ResetGyroCalibration();
						}
					usleep(8000);
					}      
        MotionManager::GetInstance()->Reinitialize();
        MotionManager::GetInstance()->SetEnable(true);
        m_is_started = 1;
        LinuxActionScript::PlayMP3("../../../Data/mp3/Start soccer demonstration.mp3");

        Action::GetInstance()->m_Joint.SetEnableBody(true, true);

        Action::GetInstance()->Start(9);
        while(Action::GetInstance()->IsRunning() == true) usleep(8000);

        Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
				usleep(50000);
        Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
				Head::GetInstance()->SetTopLimitAngle(Head::GetInstance()->m_TopLimit_soccer);
				PS3BallFollower::GetInstance()->bFullAuto = true;
				PS3BallFollower::GetInstance()->bHeadAuto = true;
        }
    
}
