#include <stdio.h>

extern "C"
{
#include "vrp_io.h"
#include "vrp_types.h"
}

#include "cudan.h"

vrp_problem *createVrpOnDevice(void)
{
	vrp_problem *device_vrp = NULL;
	cudaMalloc((void **)&device_vrp, sizeof(vrp_problem));
	return device_vrp;
}

static void transferHostToDevice(int **device_member, int *host_member, size_t size_bytes)
{
	int *device_ptr = NULL;
	cudaMalloc((void **)&device_ptr, size_bytes);
	cudaMemcpy(device_ptr, host_member, size_bytes,
		   cudaMemcpyHostToDevice);
	cudaMemcpy(device_member, &device_ptr, sizeof(int *),
		   cudaMemcpyHostToDevice);
}

void transferVrpHostToDevice(vrp_problem *device_vrp, vrp_problem *host_vrp)
{
	cudaMemcpy(device_vrp, host_vrp, sizeof(vrp_problem),
		   cudaMemcpyHostToDevice);

	transferHostToDevice(&device_vrp->dist.cost, host_vrp->dist.cost,
			     host_vrp->edgenum * sizeof(int));
	transferHostToDevice(&device_vrp->demand,    host_vrp->demand,
			     host_vrp->vertnum * sizeof(int));
}

int main(int argc, char **argv)
{
	/* SETUP VRP ON HOST */
	vrp_problem *host_vrp  = (vrp_problem *)calloc(1, sizeof(vrp_problem));
	host_vrp->vertnum      = 10;
	host_vrp->edgenum      = 10;
	host_vrp->dist.cost    = (int *)calloc(10, sizeof(int));
	host_vrp->dist.cost[0] = -1;
	host_vrp->demand       = (int *)calloc(10, sizeof(int));
	host_vrp->demand[0]    = 1;
	/* END SETUP */

	vrp_problem *device_vrp = createVrpOnDevice();
	transferVrpHostToDevice(device_vrp, host_vrp);

	int *device_dist_cost = NULL;
	cudaMemcpy(&device_dist_cost, &device_vrp->dist.cost, sizeof(int *),
		   cudaMemcpyDeviceToHost);

	int *device_demand = NULL;
	cudaMemcpy(&device_demand, &device_vrp->demand, sizeof(int *),
		   cudaMemcpyDeviceToHost);

	int count = 0;
	count += assertEqualHostAndDevice(host_vrp->vertnum,      device_vrp->vertnum);
	count += assertEqualHostAndDevice(host_vrp->dist.cost[0], device_dist_cost[0]);
	count += assertEqualHostAndDevice(host_vrp->demand[0],    device_demand[0]);

	if (count > 0)
		printf("%d FAILURE\n", count);
	else
		printf("ALL TEST PASSED\n");

	return 0;
}
