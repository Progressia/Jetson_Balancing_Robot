#include <unistd.h>
#include <stdio.h>
#include <chrono>
#include <uart.hpp>

#include <stdlib.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

using namespace std;
using namespace std::chrono;

using namespace UART;

char buff[30];
int n=0;

void testUART(Uart u)
{
    std::string CMDbuff = "";

    CMDbuff = "O1";
    n = sprintf(buff, "%s\n", CMDbuff.c_str());
    u.sendUart((uint8_t *)buff, n);
    usleep(2000000);

    CMDbuff = "O0";
    n = sprintf(buff, "%s\n", CMDbuff.c_str());
    u.sendUart((uint8_t *)buff, n);
    usleep(2000000);
}

std::string exec(const char *cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe)
    {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
    {
        result += buffer.data();
    }
    return result;
}

int rx_i = 0;

int main(int argc, char **argv)
{

    usleep(2000000);

    Uart u;
    uint8_t m[256];
    int comm_state = 0;

    int i = 0;
    
    system("../connect_WiFi.sh");
    usleep(50000000);

    system("python3 /home/nano/VSCode_Projects/RobotApp/app.py &");
    usleep(50000000);
    // -- Run camera stream by RTP (Using gstreamer)
    system("gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), format=NV12, width=1280, height=720' ! nvvidconv flip-method=0 ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=100.127.21.101 port=5001 &");

    while (true)
    {

        std::string IPbuff = exec("hostname -I");
        // std::cout << IPbuff;

        std::string strFirst = IPbuff.substr(0, IPbuff.find(' '));
        IPbuff = strFirst;
        // std::cout << IPbuff << std::endl;

        n = sprintf(buff, "IP:%s\n", IPbuff.c_str());

        u.sendUart((uint8_t *)buff, n);
        usleep(10000000);
    }

    return 0;
}
