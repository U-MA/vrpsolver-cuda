#ifndef CUDAN_H
#define CUDAN_H

int assertEqual(int expect, int actual);
int assertEqualHostAndDevice(int host_expect, int &device_actual);

#endif
