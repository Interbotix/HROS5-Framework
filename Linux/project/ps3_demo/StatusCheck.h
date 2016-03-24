/*
 * StatusCheck.h
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#ifndef STATUSCHECK_H_
#define STATUSCHECK_H_

#include "ArbotixPro.h"
#include "minIni.h"


namespace Robot
{
	enum
	{
		INITIAL,
		SITTING,
		SOCCER,
		WALK_READY,
		WALKING,
		ACTION,
		VISION,
		MAX_MODE
	};

	enum
	{
		BTN_MODE = 1,
		BTN_START = 2
	};

	enum
	{
		FAST_WALK,
		MEDIUM_WALK,
		SLOW_WALK
	};

	enum
	{
		WAIT,
		DONT_WAIT
	};

	class StatusCheck
	{
		private:
			static int m_old_btn;
			static void mPlay(int motion_page, int mode = SOCCER, int wait = WAIT);
			static void mWalkReady(ArbotixPro &arbotixpro);
			static void mAction(const char* scriptfilepath);
			static const char* SCRIPT_FILE_PATH_TRIANGLE;
			static const char* SCRIPT_FILE_PATH_CIRCLE;
			static const char* SCRIPT_FILE_PATH_CROSS;
			static const char* SCRIPT_FILE_PATH_SQUARE;
			static const char* SCRIPT_FILE_PATH_R1;
			static const char* SCRIPT_FILE_PATH_R2;
			static const char* SCRIPT_FILE_PATH_L1;
			static const char* SCRIPT_FILE_PATH_L2;
			static const char* SCRIPT_FILE_PATH_SELECT;
			static const char* SCRIPT_FILE_PATH_START;

		public:
			static int m_cur_mode;
			static int m_is_started;
			static int m_current_walk_speed;
			static int m_power_state;
			static minIni* m_ini;
			static minIni* m_ini1;
			static void Check(ArbotixPro &arbotixpro);
			static int DeadBand( int p_deadband, int p_value );
	};
}
#endif /* STATUSCHECK_H_ */