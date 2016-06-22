/*
 *   LinuxUM7.h
 *
 *   Author: ROBOTIS, Interbotix Labs
 *
 */

#ifndef _LINUX_UM7_H_
#define _LINUX_UM7_H_

#include <semaphore.h>
#include "UM7.h"


namespace Robot
{
	class LinuxUM7 : public PlatformUM7
	{
		private:
			int m_Socket_fd;
			double m_PacketStartTime;
			double m_PacketWaitTime;
			double m_UpdateStartTime;
			double m_UpdateWaitTime;
			double m_ByteTransferTime;
			char m_PortName[20];

			sem_t m_LowSemID;
			sem_t m_MidSemID;
			sem_t m_HighSemID;

			double GetCurrentTime();

		public:
			bool DEBUG_PRINT;

			LinuxUM7(const char* name);
			~LinuxUM7();

			void SetPortName(const char* name);
			const char* GetPortName()		{ return (const char*)m_PortName; }

			///////////////// Platform Porting //////////////////////
			bool OpenPort();
			void ClosePort();
			void ClearPort();
			int WritePort(unsigned char* packet, int numPacket);
			int ReadPort(unsigned char* packet, int numPacket);
			void FlushPort();

			void LowPriorityWait();
			void MidPriorityWait();
			void HighPriorityWait();
			void LowPriorityRelease();
			void MidPriorityRelease();
			void HighPriorityRelease();

			void SetPacketTimeout(int lenPacket);
			bool IsPacketTimeout();
			double GetPacketTime();
			void SetUpdateTimeout(int msec);
			bool IsUpdateTimeout();
			double GetUpdateTime();

			virtual void Sleep(int Miliseconds);
			////////////////////////////////////////////////////////
	};
}

#endif