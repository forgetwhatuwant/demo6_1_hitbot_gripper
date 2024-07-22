#ifndef HITBOT_INTERFACE_H
#define HITBOT_INTERFACE_H

#include "ControlBeanEx.h"

extern "C" int              net_port_initial();
extern "C" ControlBeanEx *  get_robot(int card_number);
extern "C" int              card_number_connect(int num);
extern "C" void             close_tcpserver();

#endif // HITBOT_INTERFACE_H
