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
#include "VisionMode.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    ((char *)"../../../Data/motion_1024.bin")
#else
#define MOTION_FILE_PATH    ((char *)"../../../Data/motion_4096.bin")
#endif
#define INI_FILE_PATH       ((char *)"../../../Data/config.ini")

#define M_INI	((char *)"../../../Data/slow-walk.ini")
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);
int GetCurrentPosition(CM730 &cm730);
////////////////////////////////////////////
Action::PAGE Page;
Action::STEP Step;
////////////////////////////////////////////
int change_current_dir()
{
    char exepath[1024] = {0};
    int status = 0;
		if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        status = chdir(dirname(exepath));
		return status;
}

int main(int argc, char *argv[])
{
		int trackerSel;    
		
		change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH); 
		minIni* ini1 = new minIni(M_INI); 
		StatusCheck::m_ini = ini;
		StatusCheck::m_ini1 = ini1;
		Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
        
    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini
		Head::GetInstance()->LoadINISettings(ini);
    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    BallTracker tracker = BallTracker();
    //tracker.LoadINISettings(ini);
    httpd::ball_finder = &tracker.finder;
		trackerSel = 0;

		//PS3BallFollower follower = PS3BallFollower();
    BallTracker robot_tracker = BallTracker();
		robot_tracker.LoadINISettings(ini,"Find Color");
		
    BallTracker line_tracker = BallTracker();
		line_tracker.LoadINISettings(ini,"WHITE_LINE");
		httpd::line_finder = &line_tracker.finder;

		ColorFinder* red_finder = new ColorFinder(0, 15, 45, 100, 0, 0.3, 50.0);
    red_finder->LoadINISettings(ini, "RED");
    httpd::red_finder = red_finder;

    ColorFinder* yellow_finder = new ColorFinder(60, 15, 45, 100, 0, 0.3, 50.0);
    yellow_finder->LoadINISettings(ini, "YELLOW");
    httpd::yellow_finder = yellow_finder;

    ColorFinder* blue_finder = new ColorFinder(225, 15, 45, 100, 0, 0.3, 50.0);
    blue_finder->LoadINISettings(ini, "BLUE");
    httpd::blue_finder = blue_finder;

    httpd::ini = ini;

    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);
	usleep(100);
    MotionManager::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    //MotionManager::GetInstance()->StartThread();
    //LinuxMotionTimer::Initialize(MotionManager::GetInstance());
    LinuxMotionTimer linuxMotionTimer;
		linuxMotionTimer.Initialize(MotionManager::GetInstance());
		linuxMotionTimer.Start();
   /////////////////////////////////////////////////////////////////////
//	MotionManager::GetInstance()->LoadINISettings(ini);

    int firm_ver = 0,retry=0;
    //important but allow a few retries
		while(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        retry++;
				if(retry >=3) exit(1);// if we can't do it after 3 attempts its not going to work.
    }

    if(0 < firm_ver && firm_ver < 27)
    {
#ifdef MX28_1024
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
        fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
        exit(0);
#endif
    }
    else if(27 <= firm_ver)
    {
#ifdef MX28_1024
        fprintf(stderr, "MX-28's firmware is not support 1024 resolution!! \n");
        fprintf(stderr, "Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
        exit(0);
#else
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#endif
    }
    else
        exit(0);

		//conversion! ////////////////
		/*
		Action::GetInstance()->LoadFile("../../../Data/motion.bin");
		int j,k,p,a;
		double f;		
		for(k=0;k<Action::MAXNUM_PAGE;k++)
			{
			Action::GetInstance()->LoadPage(k, &Page);
			for(j=0;j<Action::MAXNUM_STEP;j++)
				{
				for(p=0;p<31;p++)
					{
					a = Page.step[j].position[p];
					if(a < 1024)
						{
						f = ((a-512)*10)/3+2048;						
						a = (int)f;						
						if(a<0) a =0;
						if(a>4095) a = 4095;						
						Page.step[j].position[p] = a;						
						}						
					}				
				}
			Action::GetInstance()->SavePage(k, &Page);
			}
		exit(0);
		*/
		//copy page ////////////////
		if(argc>1 && strcmp(argv[1],"-copy")==0)
			{
			printf("Page copy -- uses files motion_src.bin and motion_dest.bin\n");
			if(Action::GetInstance()->LoadFile((char *)"../../../Data/motion_src.bin") == false)
				{
				printf("Unable to open source file\n");
				exit(1);
				}
			int k;
			void *page1;

			page1 = malloc(sizeof(Robot::Action::PAGE));
			printf("Page to load:");
			if(scanf("%d",&k) != EOF)
				{
				if(Action::GetInstance()->LoadPage(k, (Robot::Action::PAGE *)page1) == false)
					{
					printf("Unable to load page %d\n",k);
					exit(1);
					}
				if(Action::GetInstance()->LoadFile((char *)"../../../Data/motion_dest.bin") == false)
					{
					printf("Unable to open destination file\n");
					exit(1);
					}
				if(Action::GetInstance()->SavePage(k, (Robot::Action::PAGE *)page1) == false)
					{
					printf("Unable to save page %d\n",k);
					exit(1);
					}
				printf("Completed successfully.\n");
				exit(0);
				}
			}
		/////////////////////////////
/*
    Walking::GetInstance()->m_Joint.SetEnableBody(true,true);
    MotionManager::GetInstance()->SetEnable(true);

		Walking::GetInstance()->LoadINISettings(m_ini);                  

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL);

    PS3Controller_Start();
		LinuxActionScript::PlayMP3("../../../Data/mp3/ready.mp3");
    Action::GetInstance()->Start(15);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);
*/
		Walking::GetInstance()->LoadINISettings(ini);   
MotionManager::GetInstance()->LoadINISettings(ini); 

    Walking::GetInstance()->m_Joint.SetEnableBody(false);
    Head::GetInstance()->m_Joint.SetEnableBody(false);
    Action::GetInstance()->m_Joint.SetEnableBody(true);
    MotionManager::GetInstance()->SetEnable(true);
              

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x02, NULL);

    if(PS3Controller_Start() == 0)
			printf("PS3 controller not installed.\n");
		cm730.WriteWord(CM730::P_LED_HEAD_L, cm730.MakeColor(1,1,1),0);
		//determine current position
		StatusCheck::m_cur_mode = GetCurrentPosition(cm730);
		//LinuxActionScript::PlayMP3("../../../Data/mp3/ready.mp3");
		if((argc>1 && strcmp(argv[1],"-off")==0) || (StatusCheck::m_cur_mode == SITTING))
			{
			cm730.DXLPowerOn(false);
			//for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
			//	cm730.WriteByte(id, MX28::P_TORQUE_ENABLE, 0, 0);
			}
		else
			{
			Action::GetInstance()->Start(15);
			while(Action::GetInstance()->IsRunning()) usleep(8*1000);
			}
    while(1)
			{
      StatusCheck::Check(cm730);

      Point2D ball_pos, red_pos, yellow_pos, blue_pos, line_pos;

      LinuxCamera::GetInstance()->CaptureFrame();
      memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, 	LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

      if(StatusCheck::m_cur_mode == READY || StatusCheck::m_cur_mode == VISION)
        {
        ball_pos = tracker.finder.GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        red_pos = red_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        yellow_pos = yellow_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        blue_pos = blue_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        line_pos = line_tracker.finder.GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

				unsigned char r, g, b;
				for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
					{
					r = 0; g = 0; b = 0;				
					if(tracker.finder.m_result->m_ImageData[i] == 1)
						{
						r = 255;
						g = 128;
						b = 0;
						}
					if(red_finder->m_result->m_ImageData[i] == 1)
						{
						if(tracker.finder.m_result->m_ImageData[i] == 1)
							{
							r = 0;
							g = 255;
							b = 0;
							}
						else
							{
							r = 255;
							g = 0;
							b = 0;
							}
						}
					if(yellow_finder->m_result->m_ImageData[i] == 1)
						{
						if(tracker.finder.m_result->m_ImageData[i] == 1)
							{
							r = 0;
							g = 255;
							b = 0;
							}
						else
							{
							r = 255;
							g = 255;
							b = 0;
							}
						}
					if(blue_finder->m_result->m_ImageData[i] == 1)
						{
						if(tracker.finder.m_result->m_ImageData[i] == 1)
							{
							r = 0;
							g = 255;
							b = 0;
							}
						else
							{
							r = 0;
							g = 0;
							b = 255;
							}
						}
					if(line_tracker.finder.m_result->m_ImageData[i] == 1)
						{
						r = 255;
						g = 0;
						b = 255;
						}

					if(r > 0 || g > 0 || b > 0)
						{
						rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 0] = r;
						rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 1] = g;
						rgb_output->m_ImageData[i * rgb_output->m_PixelSize + 2] = b;
						}
					}
        }
      else if((StatusCheck::m_cur_mode == SOCCER || StatusCheck::m_cur_mode == SITTING) && PS3BallFollower::GetInstance()->bHeadAuto == true)
        {
        tracker.bMasked = false;
        tracker.Process(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);

        for(int i = 0,i1 = 0; i < rgb_output->m_NumberOfPixels; i++)
          {
          if(tracker.finder.m_result->m_ImageData[i] == 1)
            {
            rgb_output->m_ImageData[i1++] = 255;
            rgb_output->m_ImageData[i1++] = 128;
            rgb_output->m_ImageData[i1++] = 0;
            }
          }
        }
      else if(StatusCheck::m_cur_mode == LINE_FOLLOWING && LineFollower::GetInstance()->bHeadAuto == true)
        {
        line_tracker.bMasked = true;
				line_tracker.m_maskLeft = 80;
				line_tracker.m_maskRight =240;
				line_tracker.m_maskTop = 20;
				line_tracker.m_maskBottom = 200;
				line_tracker.Process(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        }
      else if(StatusCheck::m_cur_mode == ROBOT_FOLLOWING && RobotFollower::GetInstance()->bHeadAuto == true)
        {
        robot_tracker.bMasked = false;
				line_tracker.m_maskLeft = 0;
				line_tracker.m_maskRight = 320;
				line_tracker.m_maskTop = 0;
				line_tracker.m_maskBottom = 240;
				robot_tracker.Process(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
	      }


      streamer->send_image(rgb_output);
			if(StatusCheck::m_is_started == 0)
        continue;
							
      switch(StatusCheck::m_cur_mode)
        {
        case READY:
            break;
				case SITTING:
				case SOCCER:
					if(Action::GetInstance()->IsRunning() == 0 && (PS3BallFollower::GetInstance()->bFullAuto == true || PS3BallFollower::GetInstance()->bHeadAuto == true))
						{
						if(PS3.key.Right != 0) 
							{
							trackerSel++;
							if(trackerSel>3)  trackerSel = 0;
							switch(trackerSel)
								{
								case 0:
									tracker.LoadINISettings(ini);
									LinuxActionScript::PlayMP3("../../../Data/mp3/Red.mp3");
									break;								
								case 1:
									tracker.LoadINISettings(ini,"Yellow");
									LinuxActionScript::PlayMP3("../../../Data/mp3/Yellow.mp3");
									break;								
								case 2:
									tracker.LoadINISettings(ini,"Blue");
									LinuxActionScript::PlayMP3("../../../Data/mp3/Purple.mp3");
									break;								
								case 3:
									tracker.LoadINISettings(ini,"Red");
									LinuxActionScript::PlayMP3("../../../Data/mp3/Orange.mp3");
									break;								
								}							
							while(PS3.key.Right != 0) usleep(2000);
							}
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true,true);
						if(StatusCheck::m_cur_mode == SOCCER)
							{
							Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true,true);
							}
						//Action::GetInstance()->m_Joint.SetEnableBody(false);
						PS3BallFollower::GetInstance()->Process(tracker.ball_position);
						int value=0;
						if(PS3BallFollower::GetInstance()->bTracking == true)
								{
								switch(trackerSel)
									{
									case 0://Red
										value = cm730.MakeColor(31,0,0);
										break;
									case 1://Yellow
										value = cm730.MakeColor(31,26,0);
										break;
									case 2://Purple
										value = cm730.MakeColor(31,0,24);
										break;
									case 3://Orange
										value = cm730.MakeColor(31,7,0);
										break;
									}
								}
						else
							value = cm730.MakeColor(1,1,1);
						if(PS3BallFollower::GetInstance()->bHeadAuto == false)
							value = cm730.MakeColor(0,8,0);
						cm730.WriteWordDelayed(CM730::P_LED_HEAD_L, value);
						if(StatusCheck::m_cur_mode == SOCCER)
							{
							if(PS3BallFollower::GetInstance()->KickBall != 0)
								{
								Head::GetInstance()->m_Joint.SetEnableHeadOnly(true,true);
								Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true,true);
								//Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true);
								//int value = cm730.MakeColor(255,0,0);	
	    						//cm730.WriteWord(CM730::P_LED_EYE_L, value, 0);        
								if(PS3BallFollower::GetInstance()->bHeadAuto == true)
									{                
									Action::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
									int m = (int)(3*((float)rand()/RAND_MAX));
									int n1;
									if(PS3BallFollower::GetInstance()->KickBall == -1)
										{
					 					if(m<2)
											n1 = 12;
										else
											n1 = 70; 
										Action::GetInstance()->Start(n1);   // 12 RIGHT KICK
										fprintf(stderr, "RightKick! \n");
										}
									else if(PS3BallFollower::GetInstance()->KickBall == 1)
										{
					 					if(m<2)
											n1 = 13;
										else
											n1 = 71; 
										Action::GetInstance()->Start(n1);   // 13 LEFT KICK
										fprintf(stderr, "LeftKick! \n");
										}
									//value = cm730.MakeColor(200,0,200);
									//cm730.WriteWord(CM730::P_LED_EYE_L, value, 0);        
									}								
								}
							}
						}
            break;
				case LINE_FOLLOWING:
					if(Action::GetInstance()->IsRunning() == 0 && LineFollower::GetInstance()->bFullAuto == true)
						{
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true,true);
						Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true,true);
						//Action::GetInstance()->m_Joint.SetEnableBody(false);

						LineFollower::GetInstance()->Process(line_tracker);
						}
						break;
				case ROBOT_FOLLOWING:
					if(Action::GetInstance()->IsRunning() == 0 && RobotFollower::GetInstance()->bFullAuto == true)
						{
						Head::GetInstance()->m_Joint.SetEnableHeadOnly(true,true);
						Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true,true);
						//Action::GetInstance()->m_Joint.SetEnableBody(false);

						RobotFollower::GetInstance()->Process(robot_tracker);
						}
						break;
				case MOTION:
            if(LinuxActionScript::m_is_running == 0)
                LinuxActionScript::ScriptStart(SCRIPT_FILE_PATH);
            break;
        case VISION:
            int detected_color = 0;
            detected_color |= (red_pos.X == -1)? 0 : VisionMode::RED;
            detected_color |= (yellow_pos.X == -1)? 0 : VisionMode::YELLOW;
            detected_color |= (blue_pos.X == -1)? 0 : VisionMode::BLUE;

            if(Action::GetInstance()->IsRunning() == 0)
                VisionMode::Play(detected_color);
            break;
        }
    }

    return 0;
}

int GetCurrentPosition(CM730 &cm730)
{
	int m=Robot::READY,p,j,pos[31];
	int dMaxAngle1,dMaxAngle2,dMaxAngle3;
	double dAngle;
	int rl[6] = { JointData::ID_R_ANKLE_ROLL,JointData::ID_R_ANKLE_PITCH,JointData::ID_R_KNEE,JointData::ID_R_HIP_PITCH,JointData::ID_R_HIP_ROLL,JointData::ID_R_HIP_YAW };
	int ll[6] = { JointData::ID_L_ANKLE_ROLL,JointData::ID_L_ANKLE_PITCH,JointData::ID_L_KNEE,JointData::ID_L_HIP_PITCH,JointData::ID_L_HIP_ROLL,JointData::ID_L_HIP_YAW };

	for(p=0;p<31;p++) 
		{
		pos[p]	= -1;
		}
	for(p=0; p<6; p++)
		{
		if(cm730.ReadWord(rl[p], MX28::P_PRESENT_POSITION_L, &pos[rl[p]], 0) != CM730::SUCCESS)
			{
			printf("Failed to read position %d",rl[p]);
			}
		if(cm730.ReadWord(ll[p], MX28::P_PRESENT_POSITION_L, &pos[ll[p]], 0) != CM730::SUCCESS)
			{
			printf("Failed to read position %d",ll[p]);
			}
		}
	// compare to a couple poses
	// first sitting - page 48
	Action::GetInstance()->LoadPage(48, &Page);
	j = Page.header.stepnum-1;
	dMaxAngle1 = dMaxAngle2 = dMaxAngle3 = 0;
	for(p=0;p<6;p++)
		{
		dAngle = abs(MX28::Value2Angle(pos[rl[p]]) - MX28::Value2Angle(Page.step[j].position[rl[p]]));
		if(dAngle > dMaxAngle1)
			dMaxAngle1 = dAngle;
		dAngle = abs(MX28::Value2Angle(pos[ll[p]]) - MX28::Value2Angle(Page.step[j].position[ll[p]]));
		if(dAngle > dMaxAngle1)
			dMaxAngle1 = dAngle;
		}				
	// squating - page 15
	Action::GetInstance()->LoadPage(15, &Page);
	j = Page.header.stepnum-1;
	for(int p=0;p<6;p++)
		{
		dAngle = abs(MX28::Value2Angle(pos[rl[p]]) - MX28::Value2Angle(Page.step[j].position[rl[p]]));
		if(dAngle > dMaxAngle2)
			dMaxAngle2 = dAngle;
		dAngle = abs(MX28::Value2Angle(pos[ll[p]]) - MX28::Value2Angle(Page.step[j].position[ll[p]]));
		if(dAngle > dMaxAngle2)
			dMaxAngle2 = dAngle;
		}				
	// walkready - page 9
	Action::GetInstance()->LoadPage(9, &Page);
	j = Page.header.stepnum-1;
	for(int p=0;p<6;p++)
		{
		dAngle = abs(MX28::Value2Angle(pos[rl[p]]) - MX28::Value2Angle(Page.step[j].position[rl[p]]));
		if(dAngle > dMaxAngle3)
			dMaxAngle3 = dAngle;
		dAngle = abs(MX28::Value2Angle(pos[ll[p]]) - MX28::Value2Angle(Page.step[j].position[ll[p]]));
		if(dAngle > dMaxAngle3)
			dMaxAngle3 = dAngle;
		}				
	if(dMaxAngle1<20 && dMaxAngle1<dMaxAngle2 && dMaxAngle1<dMaxAngle3)
		m = Robot::SITTING;
	if(dMaxAngle2<20 && dMaxAngle2<dMaxAngle1 && dMaxAngle2<dMaxAngle3)
		m = Robot::READY;
	if(dMaxAngle3<20 && dMaxAngle3<dMaxAngle1 && dMaxAngle3<dMaxAngle2)
		m = Robot::SOCCER;
	printf("Sitting = %d, Squating = %d, Standing = %d\n",dMaxAngle1,dMaxAngle2,dMaxAngle3);
	printf("Robot is %s\n",m==Robot::READY?"Ready":m==Robot::SOCCER?"Soccer":m==Robot::SITTING?"Sitting":"None");
	return m;
}
