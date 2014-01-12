#include <stdlib.h>

extern "C"
{
#include "vrp_types.h"
}

#include "VrpSimulation.h"

int VrpSimulation::sequentialRandomSimulation(const vrp_problem *vrp, VehicleManager& vm)
{
    while (!vm.isVisitAll(vrp))
    {
        int candidates[200], candidatesSize = 0;

        /* 次に選ばれる顧客の候補を調べる */
        for (int i=1; i < vrp->vertnum; i++)
        {
            if (!vm.isVisit(i) && vm.canVisit(vrp, i))
                candidates[candidatesSize++] = i;
        }

        if (candidatesSize == 0)
        {
            /* 候補がいなければ次の車体へ
             * 但し, moveが失敗するとbreak */
            if (!vm.move(vrp, VehicleManager::CHANGE))
                break;
        }
        else
        {
            /* 候補の中から無作為に一つ選び、その動作を行う */
            int nextCustomer = candidates[rand() % candidatesSize];
            vm.move(vrp, nextCustomer);
        }
    }

    int cost = MISS;
    if (vm.isVisitAll(vrp))
        cost = vm.computeTotalCost(vrp);

    return cost;
}

int VrpSimulation::sequentialRandomSimulation(const vrp_problem *vrp, VehicleManager& vm, int loopCount)
{
    int minCost = MISS;
    for (int i=0; i < loopCount; i++)
    {
        VehicleManager vm_copy = vm.copy();
        int cost = sequentialRandomSimulation(vrp, vm_copy);
        if (cost < minCost)
            minCost = cost;
    }
    return minCost;
}
