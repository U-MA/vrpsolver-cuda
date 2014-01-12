#include "Device_VehicleManager.h"


int main(int argc, char **argv)
{
    Device_VehicleManager device_vm(100);

    int customer = device_vm[0].customer();
    printf("TEST device_vm_create is...");
    if (customer == 0)
        printf("ok\n");
    else
        printf("error\n");

    return 0;
}
