	#include <stdio.h>
    	#include <fcntl.h>   	 /* File Control Definitions           */
    	#include <termios.h>	 /* POSIX Terminal Control Definitions */
    	#include <unistd.h> 	 /* UNIX Standard Definitions 	       */ 
    	#include <errno.h>   	 /* ERROR Number Definitions           */
	#include <sys/ioctl.h>   /* ioctl()                            */
    	void main(void)
    	{
        	int fd;	/*File Descriptor*/
		printf("\n  +--------------------------------------------+");
		printf("\n  | Program to Control DTR pins of Serial Port |");
		printf("\n  +--------------------------------------------+");

		/*------------------ Opening the Serial port ------------------*/

		/* Change /dev/ttyUSB0 to the one corresponding to your system */

        	fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY );	        /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
			   						/* O_RDWR Read/Write access to serial port           */
									/* O_NOCTTY - No terminal will control the process   */
									/* Blocking Mode  */                       
									
        	if(fd == -1)						/* Error Checking */
            	   printf("\n    Error! in Opening ttyUSB0  ");
        	else
            	   printf("\n    ttyUSB0 Opened Successfully \n");
		
		
		/*--------- Controlling the DTR pins  of Serial Port --------*/

		int DTR_flag;

		DTR_flag = TIOCM_DTR;	/* Modem Constant for DTR pin */
		

		/*--------------------- Controlling the DTR pin using ioctl() system call ---------------------*/
		
		/* setting DTR = 1,~DTR = 0 */

		ioctl(fd,TIOCMBIS,&DTR_flag);		/* fd -file descriptor pointing to the opened Serial port */
						        /* TIOCMBIS - Set the bit corrosponding to  DTR_flag      */
		printf("\n    Setting DTR = 1,~DTR = 0 "); 
		printf("\n\n    Press any Key...");
		getchar();

		/* setting DTR = 0,~DTR = 1 */

		ioctl(fd,TIOCMBIC,&DTR_flag); 		 /* fd -file descriptor pointing to the opened Serial port */
						         /* TIOCMBIC - Clear the bit corrosponding to  DTR_flag    */           
		printf("\n    Setting DTR = 0,~DTR = 1 ");
		printf("\n\n    Press any Key...");
		getchar();
		
		close(fd); /* Close the Opened Serial Port */
		printf("\n  +--------------------------------------------+\n\n");
	}
