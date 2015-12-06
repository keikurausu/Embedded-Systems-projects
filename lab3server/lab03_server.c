//
// CS 431 - Lab 03 Server Skeleton
// PC/Linux (Provided)
//

#include <stdlib.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include "pc_crc16.h"
#include <string.h>
#include "lab03.h"

int main(int argc, char* argv[])
{
	double troll_pct=0;		// Perturbation % for the troll (if needed)
	int ifd,ofd,i,N,troll=0;	// Input and Output file descriptors (serial/troll)
	char str[MSG_BYTES_MSG],opt;	// String input
	struct termios oldtio, tio;	// Serial configuration parameters
	int VERBOSE = 0;		// Verbose output - can be overriden with -v

	// Command line options
	while ((opt = getopt(argc, argv, "t:v")) != -1) {
		switch (opt) {
			case 't':	troll = 1; 
					troll_pct = atof(optarg);
					break;
			case 'v':	VERBOSE = 1; break;
			default: 	break;
		}
	}

	printf("CS431 - Lab 03 Server\n(Enter a message to send.  Type \"quit\" to exit)\n");


	//
	// WRITE ME: Open the serial port (/dev/ttyS0) read-write
	//
        ifd = open("/dev/ttyS0", O_RDWR); //open port
	if (ifd == -1)
  	{
   	/*
   	 * Could not open the port.
    	*/
   	 perror("open_port: Unable to open /dev/ttyf1 - ");
  	}
	

	// Start the troll if necessary
	if (troll)
	{
		// Open troll process (lab03_troll) for output only
		FILE * pfile;		// Process FILE for troll (used locally only)
		char cmd[128];		// Shell command

		snprintf(cmd, 128, "./lab03_troll -p%f %s", troll_pct, (VERBOSE) ? "-v" : "");

		pfile = popen(cmd, "w");
		if (!pfile) { perror("lab03_troll"); exit(-1); }
		ofd = fileno(pfile);
	}
	else ofd = ifd;		// Use the serial port for both input and output
	


	//
 	// WRITE ME: Set up the serial port parameters and data format
	//	
	tcgetattr(ifd, &oldtio);		//save old termios
		
	tio.c_cflag = (CS8 | B9600 | CLOCAL | CREAD);
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	tcsetattr(ofd, TCSANOW, &tio);

	while(1)
	{

		//
		// WRITE ME: Read a line of input (Hint: use fgetc(stdin) to read each character)
		//

		memset(str,'\0',255);		
		fgets(str, 255, stdin);
		str[strlen(str) -1] = '\0';
		
		if (strcmp(str, "quit") == 0) break;

		//
		// WRITE ME: Compute crc (only lowest 16 bits are returned)
		//
		unsigned char ack = '\0';
		int attempts = 0;
		while (!ack)
		{
			unsigned short crc = pc_crc16(str, strlen(str));
		
		unsigned char start_byte = '\0';
		int length_int = strlen(str);
		unsigned char crc_l = crc & 0x00FF;
		unsigned char crc_m = (crc >> 8) & 0x00FF;
		unsigned char length = (char)(strlen(str));
	        unsigned char buffer[260];
		memset(buffer,'\0',260);	
		buffer[0] = start_byte;
		buffer[1] = crc_m;
		buffer[2] = crc_l;
		buffer[3] = length;

		printf("CRC = %x \n", crc);
		for(i=0; i<length_int; i++)
		{
		    buffer[i+4] = str[i];
		}
		//buffer[4] = 'a';
			printf("Sending (attempt %d)...\n", ++attempts);
			
			/*printf("crc %x\n", crc);
			printf("l %x\n", crc_l);
			printf("m %x\n", crc_m);
			printf("length %d \n", length);
			printf("string %s \n", str);*/
			// 
			// WRITE ME: Send message
			/*
			write(ifd, &start_byte,1);	l		//
			write(ifd, &crc_l, 1);
			write(ifd, &crc_m,1);
			//write(ifd, &crc, 2);
			write(ifd, &length,1);
			write(ifd, str, strlen(str));
			*/
			write(ofd, &buffer, length_int+4);
		
			printf("Message sent, waiting for ack... ");

			
			//
			// WRITE ME: Wait for MSG_ACK or MSG_NACK
			//
			
			while( read(ifd, &ack, 1) != 1 );

			//printf("%s\n", ack ? "ACK" : "NACK, resending");]
			printf("\n\n%d = ack value \n",ack);
			//printf("%d = readval\n",readval);

		}

			printf("I am here \n");
		
	}



	//
	// WRITE ME: Reset the serial port parameters
	//
	tcsetattr(ifd, TCSANOW, &oldtio);		//reset termios


	// Close the serial port
	close(ifd);

	
	return 0;
}
