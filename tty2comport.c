/*
 * tty2comport.c
 *
 * Direct tty to UART(COM) port
 *
 * Author: Soukthavy Sopha <soukthavy.sopha@yahoo.com>
 *
 * This file is licensed under  the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 *$Date: 2016-03-17 20:07:40 -0700 (Thu, 17 Mar 2016) $
 *$Id: tty2comport.c 36 2016-03-18 03:07:40Z ssop $
 *$Revision: 36 $
 */

#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/syslog.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <linux/unistd.h>

#define TRANSFER_BUFFER_SIZE 1024*2

#define MAKE_TTY_RAW(fd, original)\
term = original;\
term.c_iflag &= ~(BRKINT | IGNCR | INLCR | ICRNL | IUCLC |\
                 IXANY  | INPCK | ISTRIP);\
term.c_iflag |= (IGNBRK  | IGNPAR);\
term.c_oflag &= ~(OPOST  | ONLCR);\
term.c_lflag &= ~(XCASE  | ECHONL | NOFLSH);\
term.c_lflag &= ~(ICANON | ISIG   | ECHO | IEXTEN);\
term.c_cflag |= CREAD;\
term.c_cc[VTIME] = 5;\
term.c_cc[VMIN] = 1;\
tcsetattr(fd, TCSADRAIN, &term);

static int hndl;
static int tty_modified = 0;
static struct termios saved_termios;
static struct termios original_stdin,original_stdout,original_tty_stdx;

static void set_noecho(int fd)		/* turn off echo (for slave pty) */
{
	struct termios	stermios;

	if (tcgetattr(fd, &stermios) < 0)
		fprintf(stderr,"tcgetattr error");

	stermios.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL);
	stermios.c_oflag &= ~(ONLCR);
			/* also turn off NL to CR/NL mapping on output */

	if (tcsetattr(fd, TCSANOW, &stermios) < 0)
		fprintf(stderr,"tcsetattr error");
}
/*
clean up and close handle
*/
void cleanup()
{
   if (tty_modified) {
      tcsetattr(0,TCSADRAIN,&original_stdin);
      tcsetattr(1,TCSADRAIN,&original_stdout);
      tty_modified = 0;
   }
   close(hndl);
}
/*
signal catcher
*/
void sig_catcher(int sig)
{
   tcsetattr(hndl,TCSANOW,&saved_termios);
   printf("restored termios\n");
   cleanup();
   exit(1);
}
/*
detect keyboard hit
*/
int kbhit(void)
{
	int cnt=0, error;
	struct termios otty,otty_save;

	tcgetattr(STDIN_FILENO,&otty_save);
	otty=otty_save;
	otty.c_iflag=0;
	otty.c_oflag=0;
	otty.c_lflag &= ~ICANON; /* set raw */
	otty.c_cc[VMIN] = 1;
	otty.c_cc[VTIME] = 1;
	if ( !(error=tcsetattr(STDIN_FILENO,TCSANOW,&otty)) ) {
		struct timeval tv;
		error=ioctl(STDIN_FILENO,FIONREAD,&cnt);
		error+=tcsetattr(STDIN_FILENO,TCSANOW,&otty_save);
		tv.tv_sec=0;
		tv.tv_usec=100;
		select(STDIN_FILENO+1,NULL,NULL,NULL,&tv);
	}
	return ( error == 0 ? cnt: -1);
}
int getch(void)
{
	int error;
	struct termios otty,otty_save;
	char ch;
	
	fflush(stdout);
	tcgetattr(STDIN_FILENO,&otty_save);
	otty=otty_save;
	otty.c_iflag=0;
	otty.c_oflag=0;
	otty.c_lflag &= ~(ICANON | ECHO); /* set raw, non echo*/
	otty.c_cc[VMIN] = 1;
	otty.c_cc[VTIME] = 1;
	if ( !(error=tcsetattr(STDIN_FILENO,TCSANOW,&otty)) ) {
		error = read(STDIN_FILENO,&ch,1);
		error+= tcsetattr(STDIN_FILENO,TCSANOW,&otty_save);
	}
	return ( error == 1 ? (int)ch : -1);
}
/*
Open up UART port and begin redirect input/output from login shell.
*/
void *ttyS(char *port)
{
   struct termios term,stdIn,stdOut;
   fd_set rset;
   int fd, maxfd,selectval,nbytes;
   int sd=STDIN_FILENO;
   char buf[TRANSFER_BUFFER_SIZE];
   char devname[40];

  int ch,lastchar;
   /* connect to tty device write port */
   sprintf(devname,"/dev/tty%s",port);
   if( (fd=open(devname,O_RDWR | O_NOCTTY |O_NDELAY)) < 0 ){
	  fprintf(stderr,"can't open device %s\n",devname);
     return NULL;
   }
#ifdef DEBUG
   fprintf(stderr,"%s opened, fd=%d\n",devname,fd);
#endif   
   hndl =fd;
   fcntl(fd,F_SETFL,O_NONBLOCK); /* set non-blocking mode */
   if (tcgetattr(fd,&term) ){
      fprintf(stderr,"Can't get device attr.\n");
      close(fd);
      return NULL;
   }
   saved_termios = term; //save original termios
   if(signal(SIGINT,sig_catcher) == SIG_ERR) {
      fprintf(stderr,"Can't install signal SIGINT handler\n");
      close(fd);
      return NULL;
   }
   if(signal(SIGTERM,sig_catcher) == SIG_ERR) {
      fprintf(stderr,"Can't install signal SIGTERM handler\n");
      close(fd);
      return NULL;
   }
   if(signal(SIGHUP,sig_catcher) == SIG_ERR) {
      fprintf(stderr,"Can't install signal SIGHUP handler\n");
      close(fd);
      return NULL;
   }
    /* make the following file descriptors raw */
   
    if (isatty (0)) {
       tcgetattr(0,   &original_stdin);
       tcgetattr(1,   &original_stdout);
       tty_modified = 1;
       stdIn = original_stdin;
       stdOut = original_stdout;
       MAKE_TTY_RAW(0,  original_stdin);
       MAKE_TTY_RAW(1,  original_stdout);
    }
    
   cfmakeraw(&term);	/* raw mode */
   atexit(cleanup);

   cfsetispeed(&term,B115200); //B9600);
   cfsetospeed(&term,B115200); //B9600);
   tcsetattr(fd,TCSANOW,&term);
#ifdef DEBUG
   if (!tcgetattr(fd,&term) ){
	fprintf(stderr,"tty: ispeed=%d, ospeed = %d\n",\
	cfgetispeed(&term),cfgetospeed(&term));
    }
#endif    
   set_noecho(fd);
   set_noecho(STDOUT_FILENO);
   
#ifdef DEBUG
   if (!tcgetattr(STDIN_FILENO,&stdIn) ){
	fprintf(stderr,"STDIN: ispeed=%d, ospeed = %d\n",\
	cfgetispeed(&stdIn),cfgetospeed(&stdIn));
    }
#endif
   tcflush(fd,TCIOFLUSH);
   maxfd = ((sd > fd) ? sd:fd)+1;
  /*
   * Loop to read/write between remote client and host until 'esc  q' is received. 
   */
  for (;;)
    {
     FD_ZERO(&rset); 
      FD_SET (STDIN_FILENO, &rset);
      FD_SET (fd, &rset);
      selectval = select (maxfd, &rset, NULL, NULL, NULL);
      if (selectval == -1)
		{
		  syslog (LOG_ERR, "select(): %m");
		  break;
		}
      
      /*
	   * if client is readable, read from client, write to destination 
       */
      if (FD_ISSET (STDIN_FILENO, &rset))
		{
			  ch = getch();
			  if ((lastchar==0x1b) && (ch=='q')) {
#ifdef DEBUG				  
				  fprintf(stderr,"Esc-q received\n");
#endif				  
				  break;
			  }
			  write(fd,&ch,1);
			  lastchar = ch;
#ifdef DEBUG			  
			  if(lastchar == 0x1b)fprintf(stderr,"0x1b char received\n");
#endif			  
		}
      
      /*
	   * If destination is readable, read from destination, write to client. 
       */
      if (FD_ISSET (fd, &rset))
		{
		  nbytes = read(fd, &buf, TRANSFER_BUFFER_SIZE);
		  if (nbytes <= 0)
		    {
    	      fprintf(stderr,"server close connection\n");
		      break;
		    }
	  
		  if (write (STDOUT_FILENO, &buf, nbytes) != nbytes)
		    syslog(LOG_ERR, "Error on write %m");
	  
		  nbytes -= nbytes;
		}
    }
  
    cleanup();
#ifdef DEBUG    
   fprintf(stderr,"thread exit\n");
#endif   
   tcsetattr(hndl,TCSANOW,&saved_termios);
  return NULL;

}
/* Main */
int main (int argc, char *argv[])
{
   if (argc > 1)
   	ttyS(argv[1]);
}
