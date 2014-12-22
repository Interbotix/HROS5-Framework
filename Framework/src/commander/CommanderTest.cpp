#include "SerialInputCommander.h"
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

//hello.cpp
//int fd;
//FILE *f;

//extern char szDevice;
extern Commander command;
//Commander command = Commander();

int main()
{
    command.ControlTest();
}
