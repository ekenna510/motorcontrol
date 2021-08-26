


#include "motorcontrol/serial.h"
#include <linux/serial.h>



/* baudrate settings are defined in <asm/termbits.h>, which is
   included by <termios.h> */

#define _POSIX_SOURCE 1 /* POSIX compliant source */

#define FALSE 0
#define TRUE 1



Serial::Serial()
{
IsStreamOpen = false;
bufferIndex = 0;
for (int i = 0;i< MYSERIALNUMBUFFERS ;i ++)
{
    inUse[i]= false;
    endLine[i]=0;
}
IsLineAvailable = false;
StartofLine = 0;

}
#define    BOTHER 0010000
//----------------------------------------------------------------------------
//Initialize serial port, return file descriptor
bool Serial::serialInit(char * port, int baud)
{
  int BAUD = 0;
  fd = -1;
  struct termios newtio;
  struct serial_struct serinfo;
  IsStreamOpen = false;
  int speedset= 0;
printf("inserialinit");
 //Open the serial port as a file descriptor for low level configuration
 // read/write, not controlling terminal for process,
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY );
  if ( fd<0 )
  {
    return IsStreamOpen;
  }
// from https://en.wikibooks.org/wiki/Serial_Programming/termios exclusive use

if (ioctl(fd, TIOCEXCL, NULL) < 0) {
    printf("TIOCEXCL 1 fail");    
  return  IsStreamOpen;
}
  // set up new settings
  /* BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
     CRTSCTS : output hardware flow control (only used if the cable has
               all necessary lines. See sect. 7 of Serial-HOWTO)
     CS8     : 8n1 (8bit,no parity,1 stopbit)
     CLOCAL  : local connection, no modem contol
     CREAD   : enable receiving characters */
  //memset(&newtio, 0,sizeof(newtio)); // efk uncommented 20210615


  // efk added Get current options and | to = to form |= per howto
  //tcgetattr(fd, &newtio);

  // efk change to raw
  //newtio.c_lflag = ICANON;     //process input as lines

  BAUD = baud;
  // new stuff to do custom bauds
  if (BAUD == 78400 || BAUD == 250000 || BAUD == 57600 )
    {
		/* Custom divisor */
		serinfo.reserved_char[0] = 0;
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
            {
            perror("TIOCGSERIAL 1 fail");
			return IsStreamOpen;
            }
		serinfo.flags &= ~ASYNC_SPD_MASK;
		serinfo.flags |= ASYNC_SPD_CUST;
		serinfo.custom_divisor = (serinfo.baud_base + (BAUD / 2)) / BAUD;
		if (serinfo.custom_divisor < 1) 
			serinfo.custom_divisor = 1;
//		if (ioctl(fd, TIOCSSERIAL, &serinfo) < 0)
//            {
//             perror("TIOCSSERIAL 1 fail");
//			return IsStreamOpen;            
//            }
		if (ioctl(fd, TIOCGSERIAL, &serinfo) < 0)
            {
             perror("TIOCGSERIAL 2 fail");
			return IsStreamOpen;            
            }
		if (serinfo.custom_divisor * BAUD != serinfo.baud_base) {
			printf("actual baudrate is %d / %d = %f",
			      serinfo.baud_base, serinfo.custom_divisor,
			      (float)serinfo.baud_base / serinfo.custom_divisor);
		}

    speedset=1;
    }
  // get current settings (speed set at thie point I assume)  
  tcgetattr(fd, &newtio);
  newtio.c_cflag =  CS8 & CLOCAL & CREAD;  //no parity, 1 stop bit
  newtio.c_iflag = IGNCR & IGNBRK;    //ignore CR, ignore break condition ,other options off
  newtio.c_oflag = 0;        //all options off
  if (speedset==0)
    {
    if (cfsetispeed(&newtio, BAUD) < 0 || cfsetospeed(&newtio, BAUD) < 0)
    {
        close(fd);
        return IsStreamOpen;
    }
    }
    else
    {
       newtio.c_cflag &= ~CBAUD;
       newtio.c_cflag |= BOTHER;
       newtio.c_ispeed=BAUD;
       newtio.c_ospeed=BAUD;

    }
    

  // activate new settings
  tcsetattr(fd, TCSANOW, &newtio);
  tcflush(fd, TCIOFLUSH);
  IsStreamOpen = true;

  //efk stop using stream
  //Open file as a standard I/O stream
//  fpSerial = fdopen(fd, "r+");
//  if (!fpSerial) {
//    fpSerial = NULL;
//  }
//  else
//  {
//
//      IsStreamOpen = true;
//  }
  return IsStreamOpen;
} //serialInit




bool Serial::SerialIsOpen()
{
return IsStreamOpen;
}



void Serial::SerialClose()
{
    close(fd);
    //fclose(fpSerial);
      IsStreamOpen = false;

}



bool Serial::SerialLineAvailable()
{
    return IsLineAvailable;
}


bool Serial::SerialRead()
{
    bool retval;
    char tempbuf[rcvBufSize];
    // make sure stream is open
    if (!IsStreamOpen)
    {
        return IsStreamOpen;
    }
    retval = false;
    int bytes,bytesread; //toread,

    ioctl(fd, FIONREAD, &bytes);
    if (bytes > 0)
    {
        // read at most 59 bytes
        // if (bytes > rcvBufSize-1)
        // {
        //    toread =59;
        // }
        bytesread=read(fd, tempbuf, sizeof(tempbuf));
        if (bytesread > 0)
        {
            // we should not get this but in check bufferIndex not pointing to empty buffer
            if (inUse[bufferIndex])
            {
               bufferIndex++;
               if (bufferIndex == MYSERIALNUMBUFFERS)
               {
                   bufferIndex = 0;
               }
            }

            for (int i = 0;i < bytesread ;i++ )
            {
                if (tempbuf[i]== '\0' && i==0)
                {
                    // ignore first byte which is always \0
                    ;

                }
                else
                {
                //check if we are at max (only allow index= 57 so we can fit \n) or end of line
                if (endLine[bufferIndex] > rcvBufSize-3 || tempbuf[i]=='\n')
                {
                    // write \n to buffer (should be at most 58)
                    if (tempbuf[i]=='\n')
                    {
                        incomingBufferArray[bufferIndex][endLine[bufferIndex]] = tempbuf[i];
                        endLine[bufferIndex]++;
                    }
                    // write terminating 0 (should be at most 59
                    incomingBufferArray[bufferIndex][endLine[bufferIndex]]=0;
                    // mark line is available
                    inUse[bufferIndex]=true;
                    IsLineAvailable=true;
                    // go to next buffer to record any remaining data
                    bufferIndex++;
                    if (bufferIndex == MYSERIALNUMBUFFERS)
                    {
                        bufferIndex = 0;
                    }
                    // start at beginning of buffer
                    endLine[bufferIndex]=0;
                }
                else
                {
                    //otherwise copy a letter
                    incomingBufferArray[bufferIndex][endLine[bufferIndex]] =tempbuf[i] ;
                    endLine[bufferIndex]++;
                }
                }
            }
            // if we got here theoretically we wrote something to the buffer
            retval = true;
        }
    }

    return retval;


}



char *Serial::SerialGetData()
{
int EndIndex;
// this should be true when this gets called.
if (this->IsLineAvailable)
    {
        // reset flag so this does not get called until se fill buffer or see \n
        IsLineAvailable = false;
        EndIndex = bufferIndex -1;
        if (EndIndex < 0)
        {
            EndIndex = MYSERIALNUMBUFFERS;
        }
        consolidatedBuffer[0] = 0;
        if (StartofLine <= EndIndex)
            {
            for(; StartofLine <= EndIndex;StartofLine++)
                {

                strncat(consolidatedBuffer,&incomingBufferArray[StartofLine][0],endLine[StartofLine]);
                inUse[StartofLine]=false;

                }
            }
        else
        {
            for (; StartofLine <= MYSERIALNUMBUFFERS;StartofLine++ )
            {
                 strncat(consolidatedBuffer,&incomingBufferArray[StartofLine][0],endLine[StartofLine]);
                 inUse[StartofLine]=false;
            }
            for (StartofLine =0; StartofLine <= EndIndex;StartofLine++ )
            {
                strncat(consolidatedBuffer,&incomingBufferArray[StartofLine][0],endLine[StartofLine]);
                inUse[StartofLine]=false;
            }

        }


    }
    else
    {
        consolidatedBuffer[0] = 0;
    }
    return consolidatedBuffer;
}





void Serial::SerialWrite(char * data, int nbrbytes)
{
    int n;
    // if the stream is open
    if (IsStreamOpen)
    {
        // send data.
        n = write(fd, data, nbrbytes);
        //fprintf(fpSerial, "%s",data);
    }
}
