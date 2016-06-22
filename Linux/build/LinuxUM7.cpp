/*
 *   LinuxUM7.cpp
 *
 *   Author: ROBOTIS, Interbotix Labs
 *
 */
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include "LinuxUM7.h"

using namespace Robot;


LinuxUM7::LinuxUM7(const char* name)
{
	DEBUG_PRINT = false;
	m_Socket_fd = -1;
	m_PacketStartTime = 0;
	m_PacketWaitTime = 0;
	m_UpdateStartTime = 0;
	m_UpdateWaitTime = 0;
	m_ByteTransferTime = 0;

	sem_init(&m_LowSemID, 0, 1);
	sem_init(&m_MidSemID, 0, 1);
	sem_init(&m_HighSemID, 0, 1);

	SetPortName(name);
}

LinuxUM7::~LinuxUM7()
{
	ClosePort();
}

void LinuxUM7::SetPortName(const char* name)
{
	strcpy(m_PortName, name);
}

bool LinuxUM7::OpenPort()
{
	struct termios newtio;
	struct serial_struct serinfo;
	double baudrate = 115200.0; //bps (115200)

	ClosePort();

	if (DEBUG_PRINT == true)
		printf("\n%s open ", m_PortName);

	if ((m_Socket_fd = open(m_PortName, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
		goto UART_OPEN_ERROR;

	if (DEBUG_PRINT == true)
		printf("success!\n");

	// You must set 38400bps!
	memset(&newtio, 0, sizeof(newtio));
	newtio.c_cflag      = B38400 | CS8 | CLOCAL | CREAD;
	newtio.c_iflag      = IGNPAR;
	newtio.c_oflag      = 0;
	newtio.c_lflag      = 0;
	newtio.c_cc[VTIME]  = 0;
	newtio.c_cc[VMIN]   = 0;
	tcsetattr(m_Socket_fd, TCSANOW, &newtio);

	if (DEBUG_PRINT == true)
		printf("Set %.1fbps ", baudrate);

	// Set non-standard baudrate
	if (ioctl(m_Socket_fd, TIOCGSERIAL, &serinfo) < 0)
		goto UART_OPEN_ERROR;

	serinfo.flags &= ~ASYNC_SPD_MASK;
	serinfo.flags |= ASYNC_SPD_CUST;
	serinfo.custom_divisor = serinfo.baud_base / baudrate;

	if (ioctl(m_Socket_fd, TIOCSSERIAL, &serinfo) < 0)
		{
			if (DEBUG_PRINT == true)
				printf("failed!\n");
			goto UART_OPEN_ERROR;
		}

	if (DEBUG_PRINT == true)
		printf("success!\n");

	tcflush(m_Socket_fd, TCIFLUSH);

	m_ByteTransferTime = (1000.0 / baudrate) * 12.0;

	return true;

UART_OPEN_ERROR:
	if (DEBUG_PRINT == true)
		printf("failed!\n");
	ClosePort();
	return false;
}


void LinuxUM7::ClosePort()
{
	if (m_Socket_fd != -1)
		close(m_Socket_fd);
	m_Socket_fd = -1;
}

void LinuxUM7::FlushPort()
{
	if (m_Socket_fd != -1)
		tcdrain(m_Socket_fd);
}

void LinuxUM7::ClearPort()
{
	tcflush(m_Socket_fd, TCIFLUSH);
}

int LinuxUM7::WritePort(unsigned char* packet, int numPacket)
{
	return write(m_Socket_fd, packet, numPacket);
}

int LinuxUM7::ReadPort(unsigned char* packet, int numPacket)
{
	return read(m_Socket_fd, packet, numPacket);
}

void LinuxUM7::LowPriorityWait()
{
	sem_wait(&m_LowSemID);
}

void LinuxUM7::MidPriorityWait()
{
	sem_wait(&m_MidSemID);
}

void LinuxUM7::HighPriorityWait()
{
	sem_wait(&m_HighSemID);
}

void LinuxUM7::LowPriorityRelease()
{
	sem_post(&m_LowSemID);
}

void LinuxUM7::MidPriorityRelease()
{
	sem_post(&m_MidSemID);
}

void LinuxUM7::HighPriorityRelease()
{
	sem_post(&m_HighSemID);
}

double LinuxUM7::GetCurrentTime()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);

	return ((double)tv.tv_sec * 1000.0 + (double)tv.tv_usec / 1000.0);
}

void LinuxUM7::SetPacketTimeout(int lenPacket)
{
	m_PacketStartTime = GetCurrentTime();
	m_PacketWaitTime = m_ByteTransferTime * (double)lenPacket + 5.0;
}

bool LinuxUM7::IsPacketTimeout()
{
	if (GetPacketTime() > m_PacketWaitTime)
		return true;

	return false;
}

double LinuxUM7::GetPacketTime()
{
	double time;

	time = GetCurrentTime() - m_PacketStartTime;
	if (time < 0.0)
		m_PacketStartTime = GetCurrentTime();

	return time;
}

void LinuxUM7::SetUpdateTimeout(int msec)
{
	m_UpdateStartTime = GetCurrentTime();
	m_UpdateWaitTime = msec;
}

bool LinuxUM7::IsUpdateTimeout()
{
	if (GetUpdateTime() > m_UpdateWaitTime)
		return true;

	return false;
}

double LinuxUM7::GetUpdateTime()
{
	double time;

	time = GetCurrentTime() - m_UpdateStartTime;
	if (time < 0.0)
		m_UpdateStartTime = GetCurrentTime();

	return time;
}

void LinuxUM7::Sleep(int Miliseconds)
{
	double time = GetCurrentTime();

	do
		{
			usleep(Miliseconds * 1000);
		}
	while (GetCurrentTime() - time < (double)Miliseconds);
}
