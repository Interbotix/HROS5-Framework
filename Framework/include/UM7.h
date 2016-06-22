/*
 *   UM7.h
 *
 *   Author: ROBOTIS, Interbotix
 *
 */

#ifndef UM7_H
#define UM7_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

namespace Robot
{

	class PlatformUM7
	{
		public:
			/////////// Need to implement below methods (Platform porting) //////////////
			// Port control
			virtual bool OpenPort() = 0;
			virtual void ClosePort() = 0;
			virtual void ClearPort() = 0;
			virtual int WritePort(unsigned char* packet, int numPacket) = 0;
			virtual int ReadPort(unsigned char* packet, int numPacket) = 0;
			virtual void FlushPort() = 0;

			// Using semaphore
			virtual void LowPriorityWait() = 0;
			virtual void MidPriorityWait() = 0;
			virtual void HighPriorityWait() = 0;
			virtual void LowPriorityRelease() = 0;
			virtual void MidPriorityRelease() = 0;
			virtual void HighPriorityRelease() = 0;

			// Using timeout
			virtual void SetPacketTimeout(int lenPacket) = 0;
			virtual bool IsPacketTimeout() = 0;
			virtual double GetPacketTime() = 0;
			virtual void SetUpdateTimeout(int msec) = 0;
			virtual bool IsUpdateTimeout() = 0;
			virtual double GetUpdateTime() = 0;

			virtual void Sleep(int Miliseconds) = 0;
			//////////////////////////////////////////////////////////////////////////////
	};

	class UM7
	{

		public:
			double roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;

			//UM7();

			bool encode(unsigned char c);
			UM7(PlatformUM7 *platform);
			~UM7();

			bool Connect();
			void Disconnect();

			void configure_sensor();
			bool request_angles();

		private:
			int state = 0;

			enum {STATE_ZERO, STATE_S, STATE_SN, STATE_SNP, STATE_PT, STATE_DATA, STATE_CHK1, STATE_CHK0};

			unsigned char packet_type;
			unsigned char address;
			bool packet_is_batch;
			unsigned char batch_length;
			bool packet_has_data;
			unsigned char data[30];
			unsigned char data_length;
			unsigned char data_index;

			unsigned char checksum1;		// First byte of checksum
			unsigned char checksum0;		// Second byte of checksum

			unsigned short checksum10;			// Checksum received from packet
			unsigned short computed_checksum;	// Checksum computed from bytes received

			bool checksum(void);

			void save(void);

			PlatformUM7 *m_Platform;



	};

}

#endif