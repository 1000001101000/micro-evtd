/*
* Micro Event Daemon for Linkstation Pro+Live/Kuro/Terastation ARM series
*
* Written by Bob Perry (2007-2009) lb-source@users.sourceforge.net
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <termios.h>
#include <unistd.h>
#include <sys/param.h>
#include <sys/mount.h>
#include <stdio.h>
#include <string.h>
#include <linux/serial.h>
#include <stdlib.h>
#include <sys/sem.h>
#include <sys/file.h>

// Some global variables with default values
const char strTitle[]="Micro Event Daemon for LSP/LSL/Kuro/TS";
const char strVersion[]="3.4";
char strRevision[]="$Revision$";
const char micro_devicesV2[][15] = {
                             "/dev/ttyS1",
                             "/dev/ttyS3",
                             "/dev/ttyS2",
			     "/dev/ttyS0",
                         };

const char micro_devicesV3[][15] = {
                             "/dev/ttyUSB0",
                             "/dev/ttyS0",
                             "/dev/ttyS1",
                         };

const char micro_lock[]="/var/lock/micro-evtd";
const char strokTest[] = ",=\n";
char* pDelayProcesses = NULL;
char debug=0;
char iNotQuiet=1;
int i_FileDescriptor = 0;
int i_Poll = 0;
int mutexId=0;
key_t mutex;
int m_fd = 0;
int resourceLock_fd = 0;
#ifdef TEST
  long override_time=0;
#endif

static void open_serialV2(void);
static void open_serialV3(void);
static void close_serial(void);
static void resetV2(void);
static int writeUARTV2(int, unsigned char*);

/**
************************************************************************
*
*  function    : open_serialV2()
*
*  description : Gain access to the micro interface.
*
*  arguments   : (in)	void
*
*  returns     : 		void
************************************************************************
*/
static void open_serialV2(void)
{
	struct termios newtio;
	unsigned char testmsg[] = {0x00,0x03,0xfd};
	unsigned char testbuf[4];
	int j=0,tmp=0,tmplen=0;
	struct timeval tmptime;
	fd_set tmpFS;

	//count of ports in the ports list
	int portcnt = sizeof(micro_devicesV2)/sizeof(micro_devicesV2[0]);

	for (j = 0; j < portcnt; j++)
	{
		//if port does not exist, just move on
		if( access( micro_devicesV2[j], F_OK ) == -1 )
		{
			continue;
		}

		//see if we can open the port
		i_FileDescriptor = open(micro_devicesV2[j], O_RDWR);
		//if we can't move on
		if(i_FileDescriptor < 0)
		{
                	continue;
        	}
		ioctl(i_FileDescriptor, TCFLSH, 2);
                /* Clear data structures */
                memset(&newtio, 0, sizeof(newtio));

                newtio.c_iflag =IGNBRK;
                newtio.c_cflag = PARENB | CLOCAL | CREAD | CSTOPB | CS8 | B38400;
                newtio.c_lflag &= (~ICANON);
                newtio.c_cc[VMIN] = 0;
                newtio.c_cc[VTIME] = 100;

                /* Update tty settings */
                ioctl(i_FileDescriptor, TCSETS, &newtio);
                ioctl(i_FileDescriptor, TCFLSH, 2);

		//send a test command, striped down for speed/simplicity
		tmp = write(i_FileDescriptor, testmsg, 3);
		tmptime.tv_usec = 50000;
                tmptime.tv_sec = 0;
                FD_ZERO(&tmpFS);
                FD_SET(i_FileDescriptor, &tmpFS);
		usleep(50000);
		tmp = select(i_FileDescriptor + 1, &tmpFS, NULL, NULL, &tmptime);

		//if no bytes are returned, move on
		if (tmp != 1)
		{
			continue;
		}

		tmplen = read(i_FileDescriptor, testbuf, 4);

		//finally, confirm response corresponds to the sent command
		if (tmplen == 4 && testbuf[01] == 0x03)
		{
			return;
		}

		close_serial();
	}
}

static void open_serialV3(void)
{
        struct termios newtio;
        unsigned char testmsg[] = "VER_GET\r";
        unsigned char testbuf[64];
        int j=0,tmp=0,tmplen=0;
        struct timeval tmptime;
        fd_set tmpFS;

        //count of ports in the ports list
        int portcnt = sizeof(micro_devicesV2)/sizeof(micro_devicesV2[0]);

        for (j = 0; j < portcnt; j++)
        {
		//if port does not exist, just move on
                if( access( micro_devicesV3[j], F_OK ) == -1 )
                {
                        continue;
                }

                //see if we can open the port
                i_FileDescriptor = open(micro_devicesV3[j], O_RDWR);
                //if we can't move on
                if(i_FileDescriptor < 0)
                {
                        continue;
                }
                ioctl(i_FileDescriptor, TCFLSH, 2);
                /* Clear data structures */
                memset(&newtio, 0, sizeof(newtio));

                newtio.c_iflag =IGNBRK;
                newtio.c_cflag = CLOCAL | CREAD | CSTOPB | CS8 | B57600;
                newtio.c_lflag &= (~ICANON);
                newtio.c_cc[VMIN] = 0;
                newtio.c_cc[VTIME] = 100;

                /* Update tty settings */
                ioctl(i_FileDescriptor, TCSETS, &newtio);
                ioctl(i_FileDescriptor, TCFLSH, 2);

                //send a test command, striped down for speed/simplicity
                tmp = write(i_FileDescriptor, testmsg, sizeof(testmsg)-1);
                tmptime.tv_usec = 50000;
                tmptime.tv_sec = 0;
                FD_ZERO(&tmpFS);
                FD_SET(i_FileDescriptor, &tmpFS);
                usleep(350000);
                tmp = select(i_FileDescriptor + 1, &tmpFS, NULL, NULL, &tmptime);

                //if no bytes are returned, move on
                if (tmp != 1)
                {
                        continue;
                }

		tmplen = read(i_FileDescriptor, testbuf, 64);
		testbuf[tmplen] = '\0';

                //finally, confirm response corresponds to the sent command
                if (tmplen > 1)
                {
                        return;
                }

                close_serial();
        }
}


/**
************************************************************************
*
*  function    : close_serial()
*
*  description : Release our serial port handle along with the resource
*				 lock.
*
*  arguments   : (in)	void
*
*  returns     : 		void
************************************************************************
*/
static void close_serial(void)
{
	union semun {
		int val;
		struct semid_ds *buf;
		ushort *array;
	} arg;

	if (i_FileDescriptor > 0) {
		/* Close port and invalidate our pointer */
		close(i_FileDescriptor);
	}

	// Close memory handle too
	if (m_fd > 0)
		close(m_fd);

	/* Close flock handles if we have one */
	if (resourceLock_fd > 0)
		close(resourceLock_fd);

	// Remove it mutex if we have one
	if (mutexId >0)
		semctl(mutexId, 0, IPC_RMID, arg);

	// Free memory
	if (pDelayProcesses)
		free(pDelayProcesses);
}

/**
************************************************************************
*
*  function    : resetV2()
*
*  description : Demand micro reset.
*
*  arguments   : (in)	void
*
*  returns     : 		void
************************************************************************
*/
static void resetV2(void)
{
	char buf[40]={0xFF,};
	int i;
	write(i_FileDescriptor, &buf, sizeof(buf));
	// Give micro some time
	usleep(400);
	for (i=0;i<2;i++) {
		read(i_FileDescriptor, buf, sizeof(buf));
	}

	usleep(500);
}

/**
************************************************************************
*
*  function    : lockMutex()
*
*  description : Lock/un-lock the micro interface resource.
*
*  arguments   : (in)	char				= lock flag.
*
*  returns     : 		void
************************************************************************
*/
static void lockMutex(char iLock) {
	// Handle mutex locking/un-locking here
	struct sembuf mutexKey = {0, -1, 0};  // set to allocate resource
	// Mutex valid?
	if (mutexId > 0) {
		// Lock resource
		if (!iLock)
			mutexKey.sem_op = 1; // free resource

		semop(mutexId, &mutexKey, 1);
	}

	// Drop back to flocks
	else
		flock(resourceLock_fd, (iLock) ? LOCK_EX : LOCK_UN);
}

/**
************************************************************************
*
*  function    : writeUARTV2()
*
*  description : Send the supplied transmit buffer to the micro.  We
*				 calculate and populate the checksum and return the
*				 micro response.
*
*  arguments   : (in)	int					= number of characters
*						unsigned char*		= pointer to transmit buffer
*
*  returns     : 		int					= micro return
************************************************************************
*/
static int writeUARTV2(int n, unsigned char* output)
{
	unsigned char txcksum = 0;
	unsigned char rxcksum = 0;
	unsigned char rbuf[35];
	unsigned char tbuf[35];

	unsigned char txlen=0, txmode=0, txcmd=0, rxmode=0, rxcmd=0, rxchk=0;

	char retries = 2;
	int i = 0;
	fd_set fReadFS;
	struct timeval tt_TimeoutPoll;
	int iReturn = -1;
	int iResult;

	/* We got data to send? */
	if (n >0) {
		/* Calculate the checksum and populate the output buffer */
		for (i=0;i<n;i++) {
			txcksum -= tbuf[i] = output[i];
		}

		tbuf[n] = txcksum;
	}
        /*********** or else what? *********/
	//min should probably be 2 to be a valid command
	//could check max length while at it
	//generally validate the format

	lockMutex(1);

	while (retries-- > 0)
	{
		int len = -1;

		/* Send data */
		iResult = write(i_FileDescriptor, tbuf, n+1);

		//if writing to serial port appears to have failed
		if (iResult < n+1)
		{
			printf("%s: %d\n", "serial write failed",n);
			resetV2();
			continue;
		}

		tt_TimeoutPoll.tv_usec = 500000;
		tt_TimeoutPoll.tv_sec = 0;

		FD_ZERO(&fReadFS);
		FD_SET(i_FileDescriptor, &fReadFS);

		usleep(100000);
		/* Wait for a max of 500ms for write response */
		iResult = select(i_FileDescriptor + 1, &fReadFS, NULL, NULL, &tt_TimeoutPoll);
		/* We did not time-out or error? No, then get data*/
		if (iResult !=0) {
			/* Ignore data errors */
			len = read(i_FileDescriptor, rbuf, sizeof(rbuf));
		}

		txlen  = n+1;
                txmode = tbuf[0];
                txcmd  = tbuf[1];

		if (debug)
                {
                        printf("sent:\n");
                        printf("raw:  ");
                        for (i=0;i<txlen;i++)
                                printf("%02x ", tbuf[i]);
                        printf("\n");
                        printf("data: ");
                        for (i=2;i<txlen-1;i++)
                                printf("%02x ", tbuf[i]);
                        printf("\n");
                        printf("length: %d mode: %02x cmd: %02x checksum: %02x \n\n",txlen,txmode,txcmd,txcksum);

		}

		/* Too little data? Yes, its an error */
		//mode + cmd + status + checksum
		if (len < 4)
		{
			printf("%s: %d\n", "no response",n);
                        resetV2();
                        continue;
		}

		/* We received some data */
		// Calculate data sum for validation
		for (i=0;i<len;i++)
			rxcksum -= rbuf[i];

		// Process if data valid
		if (rxcksum !=0)
		{
			printf("%s: %d\n", "invalid checksum received",rxcksum);
                        resetV2();
                        continue;
		}

		//if we got this far the micon responded, no need to retry
		retries = 0;

		//ideally we'll swap this for the error code returned by the micon
		//I'm not currently aware how to distinguish between an error code
		//and a valid 1 byte message that happens to be in the same range.
		//for now 0=successful response -1=max retries exceeded.
		iReturn = 0;

		// Check if returned command matches sent command
		if (rbuf[1] != output[1])
		{
			iReturn = 99;
			printf("%s: %d\n", "invalid command",output[1]);
			break;
		}

		rxmode = rbuf[0];
		rxcmd  = rbuf[1];
		rxchk  = rbuf[len-1];

		if (debug)
		{
			printf("response:\n");
			printf("raw:  ");
			for (i=0;i<len;i++)
				printf("%02x ", rbuf[i]);
			printf("\n");
			printf("data: ");
			for (i=2;i<len-1;i++)
				printf("%02x ", rbuf[i]);
			printf("\n");
			printf("length: %d mode: %02x cmd: %02x checksum: %02x(%d) \n\n",len,rxmode,rxcmd,rxchk,rxcksum);
		}

		//if response larger than min, data returned
		if (iNotQuiet)
		{
			//if command is known to return a string, zero terminate and print as string
			//add some validation in case of invalid chars?
			//for now rely on errors being one char and strings always being longer
			if (rbuf[1] >= 0x80 && len > 5)
			{
				rbuf[len-1]=0;
				printf("%s\n",rbuf + 2);
			}
			else
			{
				//iterate through data bytes and print
				for (i=2;i<len-1;i++)
					printf("%d", rbuf[i]);
				printf("\n");
			}
		}

	}

	lockMutex(0);
	return iReturn;
}



static int writeUARTV3(char* msg)
{
	char inbuf[32];
	int msglen = strlen(msg);
	int tmp,tmplen = 0;
	struct timeval tmptime;
        fd_set tmpFS;

	lockMutex(1);
	msg[msglen] = '\r';//remove the ternminator and add new line to make micon happy

	tmp = write(i_FileDescriptor, msg, msglen);
	msg[msglen] = '\0'; //restore terminator when done


	if (tmp != msglen)
	{
		return(99);
	}

        tmptime.tv_usec = 50000;
        tmptime.tv_sec = 0;
        FD_ZERO(&tmpFS);
        FD_SET(i_FileDescriptor, &tmpFS);
        usleep(350000);
        tmp = select(i_FileDescriptor + 1, &tmpFS, NULL, NULL, &tmptime);

	if (tmp != 1)
        {
                return(99);
        }

        tmplen = read(i_FileDescriptor, inbuf, 32);
        inbuf[tmplen] = '\0';

	if (inbuf[tmplen-1] != 0x0a)
	{
		return(99);
	}

	if (iNotQuiet)
        {
		printf("%s",inbuf);
	}

        lockMutex(0);
        return 0;
}

/**
************************************************************************
*
*  function    : main()
*
*  description : Processes input demands.
*
*  arguments   : (in)	int				= number of commands
*  						char*			= Pointer to supplied tokens
*
*  returns     : 		void
*
************************************************************************
*/
int main(int argc, char *argv[])
{
	int iLen;
	int i = 0;
	char *thisarg;
	unsigned char uiMessage[36] = {0,};
	char* pos = NULL;

	//remove cmd from args list
	argc--;
	argv++;

	// Ensure un-buffered output
	setvbuf(stdout, (char*)NULL, _IONBF, 0);

	// Parse any options
	while (argc >= 1 && '-' == **argv) {
		thisarg = *argv;
		thisarg++;
		switch (*thisarg) {
#ifdef TEST
		case 't':
			argc--;
			argv++;
			override_time = atoi(*argv);
			break;
#endif
		case 'v':
			--argc;
			printf("%s %s (%s)\n", strTitle, strVersion, strRevision);
			exit(0);
			break;
		case 'd':
                        --argc;
                        debug = 1;
                        break;
		case 'q':
			--argc;
			iNotQuiet = 0;
			break;
		case 's':
			argc--;
			argv++;
			// Grab the mutex, -1 will indicate no server.  Use a local lock flag
			// so we can maintain a lock on the resource whilst processing any
			// batched commands
			if (mutex >0)
				mutexId = semget(mutex, 1, 0666);
			else {
				resourceLock_fd = open(micro_lock, O_WRONLY|O_CREAT, 0700);
			}

			if (thisarg[1] == '3')
			{
				open_serialV3();
				pos = strtok(*argv, ",");
				while (pos != 0)
				{
					i = writeUARTV3(pos);
					pos = strtok(NULL, ",");
				};
				close_serial();
				exit(i);
                        	break;
			}
			else
			{
			// Allocate device
			open_serialV2();
			// Loop through batched commands
			// is there any validation to do at this step? invalid chars?
			pos = strtok(*argv, ", ");
			while (pos != 0) {
				// Get command length
				iLen = strlen(pos)/2;
				//if iLen not even it's invalid...
				// convert each set of two hex chars to corresponding byte
				for (i=0;i<iLen;i++)
					sscanf(pos+2*i, "%2hhx", &uiMessage[i]);
					//what does this do with non-hex chars?

				// Push it out and return result
				i = writeUARTV2(iLen, uiMessage);

				//current logic only returns non-zero for com failure
				//assume all future cmds will fail and just exit.
				if (i != 0)
				{
					close_serial();
					exit(i);
				}
				pos = strtok(NULL, ", ");
			};
			}
			exit(0);
			break;
		}

		argc--;
		argv++;
	}

	close_serial();

	return 0;
}

