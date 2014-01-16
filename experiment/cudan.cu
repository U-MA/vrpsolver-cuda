#include <stdio.h>

int assertEqual(int expect, int actual)
{
	if (expect != actual)
	{
		printf("assertEqual: expect %d, but %d\n",
		       expect, actual);
		return 1;
	}
	return 0;
}

int assertEqualHostAndDevice(int host_expect, int &device_actual)
{
	int actual = 0;
	cudaMemcpy(&actual, &device_actual, sizeof(int),
		   cudaMemcpyDeviceToHost);

	if (host_expect == actual) return 0;

	printf("assertEqualHostAndDevice: expect %d, but %d\n",
	       host_expect, actual);
	return 1;
}
