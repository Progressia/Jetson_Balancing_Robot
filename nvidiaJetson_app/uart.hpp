
#ifndef _UART_H
#define _UART_H

#include <stdint.h>

using namespace std;

namespace UART
{

  class Uart {
  private:
    /* data */
    int fid;

  public:

    // Define Constants
    const char *uart_target = "/dev/ttyACM0";  //USB
    // const char *uart_target = "/dev/ttyTHS1";     //J40
    #define NSERIAL_CHAR 256
    #define VMINX 0//1
    #define BAUDRATE B115200
    const char *SetBAUDRATE = "B115200";

    uint8_t RX_message[NSERIAL_CHAR];
    uint8_t TX_buf[NSERIAL_CHAR];
    int TX_nbytes = 8;
    int RX_nbytes = 8;

    Uart();
    void openUart(void);
    int sendUart(uint8_t *msg, int nbytes);
    int readUart();
    uint16_t calcCRC(uint8_t *msg, int nbytes);
    void closeUart(void);

    void UartTestLoop();

    ~Uart();
  };

}
#endif
