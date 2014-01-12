#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern "C"
{
#include "vrp_io.h"
#include "vrp_types.h"
}

#include "Node.h"
#include "Solver.h"
#include "VehicleManager.h"

vrp_problem *vrp;
long gSeed;
int  gCount;
int  gSimulationCount;


void Solver::setProblem(char *filename)
{
    vrp = (vrp_problem *)malloc(sizeof(vrp_problem));
    vrp_io(vrp, filename);
    printf("file name       : %s\n", filename);

    /* numroutesの設定 */
    char *k   = strrchr(filename, 'k');
    char *dot = strrchr(filename, '.');
    char numVehicle[3];
    int n = (dot-k)/sizeof(char);
    strncpy(numVehicle, k+1, n);
    numVehicle[n+1] = '\0';
    vrp->numroutes = atoi(numVehicle);
}

void Solver::setSeed(long seed)
{
    gSeed = seed;
}

void Solver::setMctsIterationCount(int count)
{
    gCount = count;
}

void Solver::setSimulationCount(int count)
{
    gSimulationCount = count;
}

static void checkGlobalVariable(void)
{
    if (gSeed == 0)
        gSeed = 2013;

    if (gCount == 0)
        gCount = 1000;

    if (gSimulationCount == 0)
        gSimulationCount = 1;

    printf("seed            : %ld\n"  , gSeed);
    printf("search count    : %d\n"   , gCount);
    printf("simulation count: %d\n\n" , gSimulationCount);
}

void Solver::run(void)
{
    checkGlobalVariable();

    VehicleManager vm;
    while (!vm.isVisitAll(vrp))
    {
        Node mct;

        for (int i=0; i < gCount; i++)
            mct.search(vrp, vm, gSimulationCount);

        int move = mct.next();
        printf("move %d\n", move);

        if (!vm.move(vrp, move))
            break;
    }

    int cost = 1e6;
    if (vm.isVisitAll(vrp))
        cost = vm.computeTotalCost(vrp);

    vm.print();
    printf("[COST] %6d\n", cost);
}
