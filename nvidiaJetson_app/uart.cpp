
#include "uart.hpp"

#include <stdio.h>
#include <unistd.h>    // Used for UART
#include <sys/fcntl.h> // Used for UART
// #include <termios.h>   // Used for UART
#include <string>
#include <stdlib.h>
#include <iostream>
#include <stdint.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <bitset>
#include <climits>
#include <cstring>
#include <iostream>

using namespace std;
using namespace UART;

Uart::Uart()
{
  
  /* -1- Initialize configuration */
  std::cout << "=> Start UART object"<< std::endl;

  /* -2- Open UART */
  Uart::openUart();
}

void Uart::openUart(void)
{

  int ii, jj, kk;

  // SETUP SERIAL WORLD
  struct termios port_options; // Create the structure

  tcgetattr(fid, &port_options); // Get the current attributes of the Serial port

  //------------------------------------------------
  //  OPEN THE UART
  //------------------------------------------------
  // The flags (defined in fcntl.h):
  //	Access modes (use 1 of these):
  //		O_RDONLY - Open for reading only.
  //		O_RDWR   - Open for reading and writing.
  //		O_WRONLY - Open for writing only.
  //	    O_NDELAY / O_NONBLOCK (same function)
  //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
  //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
  //				   immediately with a failure status if the output can't be written immediately.
  //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
  //	    O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

  //SET PERMISSION FOR uart_target
  // system("sudo chmod 666 /dev/ttyTHS0");

  /* //SET PERMISSION FOR uart_target */
  /*  //if not working without -> /etc/udev/rules.d/99-tegra-devices.rules -> KERNEL=="ttyTHS0" MODE="0666"
      $ systemctl stop nvgetty
      $ systemctl disable nvgetty
      $ udevadm trigger
      # You may want to reboot instead */

  fid = open(uart_target, O_RDWR | O_NOCTTY | O_NONBLOCK);

  tcflush(fid, TCIFLUSH);
  tcflush(fid, TCIOFLUSH);

  usleep(1000000); // 1 sec delay

  if (fid == -1)
  {
    // printf("**Error - Unable to open UART**.  \n=>Ensure it is not in use by another application\n=>Ensure proper privilages are granted to accsess /dev/.. by run as a sudo\n");
    perror("**Error - Unable to open UART**.  \n=>Ensure it is not in use by another application\n=>Ensure proper privilages are granted to accsess /dev/.. by run as a sudo\n");
	  exit(1);
  }

  //------------------------------------------------
  // CONFIGURE THE UART
  //------------------------------------------------
  // flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
  //	Baud rate:
  //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200,
  //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000,
  //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
  //	CSIZE: - CS5, CS6, CS7, CS8
  //	CLOCAL - Ignore modem status lines
  //	CREAD  - Enable receiver
  //	IGNPAR = Ignore characters with parity errors
  //	ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
  //	PARENB - Parity enable
  //	PARODD - Odd parity (else even)

  port_options.c_cflag &= ~PARENB;                         // Disables the Parity Enable bit(PARENB),So No Parity
  port_options.c_cflag &= ~CSTOPB;                         // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
  port_options.c_cflag &= ~CSIZE;                          // Clears the mask for setting the data size
  port_options.c_cflag |= CS8;                             // Set the data bits = 8
  port_options.c_cflag &= ~CRTSCTS;                        // No Hardware flow Control
  port_options.c_cflag |= CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
  port_options.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable XON/XOFF flow control both input & output
  port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Non Cannonical mode
  port_options.c_oflag &= ~OPOST;                          // No Output Processing

  port_options.c_lflag = 0; //  enable raw input instead of canonical,

  port_options.c_cc[VMIN] = VMINX; // Read at least 1 character
  port_options.c_cc[VTIME] = 0;    // Wait indefinetly

  cfsetispeed(&port_options, BAUDRATE); // Set Read  Speed
  cfsetospeed(&port_options, BAUDRATE); // Set Write Speed

  // Set the attributes to the termios structure
  int att = tcsetattr(fid, TCSANOW, &port_options);

  if (att != 0)
  {
    printf("ERROR in Setting port attributes\n");
  }
  else
  {
    printf("SERIAL Port Good to Go.\n");
    std::cout << "UART object: Default UART construtor is started" << std::endl;
  }

  // Flush Buffers
  tcflush(fid, TCIFLUSH);
  tcflush(fid, TCIOFLUSH);

  usleep(500000); // 0.5 sec delay

}

void Uart::closeUart(void){
  //-------------------------------------------
  //  CLOSE THE SERIAL PORT
  //-------------------------------------------
  close(Uart::fid);
  std::cout << "=> Stop uart" << std::endl;
}

int Uart::sendUart(uint8_t *msg, int nbytes)
{
  int retval = -1;

    Uart::TX_nbytes = nbytes;
    //--------------------------------------------------------------
    // TRANSMITTING BYTES
    //--------------------------------------------------------------
    uint8_t tx_buffer[nbytes];
    uint8_t *p_tx_buffer;

    p_tx_buffer = &tx_buffer[0];

    // so that i have the number of bytes to write
    // by doing p_tx - tx
    for (int i = 0; i < nbytes; i++)
    {
      *p_tx_buffer++ = msg[i];
    }

    for (int i = 0; i < (nbytes); i++)
    {
      Uart::TX_buf[i] = tx_buffer[i];
    }
    // printf("[0x%x]", CRC16);
    // std::cout << std::endl;
    // printf("\n");

    // printf("fid 1=%d\n", fid);

    if (fid != -1)
    {
      int count = write(fid, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0])); // Filestream, bytes to write, number of bytes to write
      retval = 1;
      //  usleep(1000);   // .001 sec delay

      // printf("Count = %d\n", count);

      if (count < 0){
        printf("UART TX error\n");
        retval = 0;
      }
    }

    return retval;
}

int Uart::readUart()
{
    int reval = -1;
    uint8_t read_byte;

    int num_bytes = read(fid, &read_byte, 1);

    if (read_byte == 0x02)
    {
      uint8_t read_buf[256];
      uint8_t check_buf[256];

      num_bytes = read(fid, &read_buf, 8);

      if (num_bytes > 0)
      {

        // printf("Rx: 0x%X ", read_byte);
        check_buf[0] = read_byte;

        for (int i = 0; i < num_bytes; i++)
        {
          check_buf[i + 1] = read_buf[i];
          // printf("0x%X ", read_buf[i]);
        }

        // Check CRC16
        uint16_t CRC16 = Uart::calcCRC(check_buf, 7);

        // printf(" CRC16: 0x%X \n", CRC16);

        reval = 1;
      }
      else
      {
        reval = 0;
      }
    }
    else
    {
      reval = 0;
    }

    return reval;
}

void Uart::UartTestLoop()
{
  
  // Open the serial port. Change device path as needed (currently set to an standard FTDI USB-UART cable type device)
  int serial_port = open("/dev/ttyTHS0", O_RDWR);

  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      // return 1;
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, BAUDRATE);
  cfsetospeed(&tty, BAUDRATE);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      // return 1;
  }

  // Allocate memory for read buffer, set size according to your needs
  uint8_t read_buf [256];

  // Normally you wouldn't do this memset() call, but since we will just receive
  // ASCII data for this example, we'll set everything to 0 so we can
  // call printf() easily.
  // memset(&read_buf, '\0', sizeof(read_buf));

  // Read bytes. The behaviour of read() (e.g. does it block?,
  // how long does it block for?) depends on the configuration
  // settings above, specifically VMIN and VTIME
  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
      // return 1;
  }

  // Here we assume we received ASCII data, but you might be sending raw bytes (in that case, don't try and
  // print it to the screen like this!)
  // printf("Read %i bytes. Received message: %s \n", num_bytes, read_buf);
  printf("Read %i bytes. Received message: ", num_bytes);

  for(int i=0; i<num_bytes; i++)
    printf("0x%X ", read_buf[i]);

  printf("\n");

  close(serial_port);
  // return 0; // success
}

uint16_t Uart::calcCRC(uint8_t *msg, int nbytes)
{
  //--------------------------------------------------------------
  // CALCULATION CRC16 - ModBus
  //--------------------------------------------------------------
  uint16_t poly = 0xA001;
  uint16_t crc;
  int i, j;

  crc = 0xFFFF; //Load to crc 0xFFFF
  for (j = 0; j < nbytes; j++)
  {
    crc ^= msg[j]; //XOR with first received byte (on left) and put to crc
    for (i = 0; i < 8; i++)
    {
      if (crc & 0x0001) // Check if LSB is equel 1
      {
        crc = ((crc >> 1) & 0xFFFF) ^ poly; //Shift crc to the right by 1 XOR with poly
      }
      else
      {
        crc = ((crc >> 1) & 0xFFFF);  //If not, just shift crc to the right by 1
      }
    }
  }
  return crc;
}

Uart::~Uart()
{
  //-------------------------------------------
  //  CLOSE THE SERIAL PORT
  //-------------------------------------------
  close(Uart::fid);
  std::cout << "=> Stop UART object" << std::endl;
}

/**************************************************************END OF FILE****/
