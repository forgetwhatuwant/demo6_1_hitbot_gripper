#include <iostream>
#include "hitbot_interface.h"
#include <unistd.h>
#include <thread>

using namespace std;
ControlBeanEx *rbt;

int main(int argc, char *argv[])
{
    cout << "[Start] " << endl;
    int ret = net_port_initial();
    cout << "Connecting.. : " << ret << endl;

    rbt = get_robot(190);
    if (rbt->is_connected())
    {
        cout << "Robot connected" << endl;
    }
    else
    {
        cout << "Robot not connected" << endl;
    }

    ret = rbt->initial(1, 410);
    cout << "robot initial return = " << ret << endl;
   
    rbt->new_movej_xyz_lr(23.77,63.74,-34,-10,70,0,1);
    rbt->wait_stop();

    
    // 485 COM
    // ret = rbt->com485_initial(115200);
    // cout << "485 Initialize, return =" << ret << endl;

    // unsigned char data_recv[13]{0};
    // unsigned char data_send[13] = {0x01, 0x10, 0x00, 0x02, 0x00, 0x02, 0x04, 0x41, 0xF0, 0x00, 0x00, 0x66, 0x79};

    // cout << "data_send ";
    // for (int i = 0; i < 13; i++)
    // {
    //     cout << int(data_send[i]) << " ";
    // }

    // while (true)
    // {
    //     ret = rbt->com485_send(&data_send[0], 13);
    //     usleep(1000 * 2000);
    //     cout << "485, ret =" << ret << endl;
    //     ret = rbt->com485_recv(&data_recv[0]);
    //     cout << "485 recv, ret =" << ret << endl;
    //     for (int i = 0; i < 13; i++)
    //     {
    //         cout << int(data_recv[i]) << " ";
    //     }
    // }

    return true;
}