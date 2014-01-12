#ifndef VRPSOLVER_CPP_VEHICLE_MANAGER_H
#define VRPSOLVER_CPP_VEHICLE_MANAGER_H

extern "C"
{
#include "vrp_types.h"
}

#include "Vehicle.h"

class VehicleManager
{
public:
    enum
    {
        CHANGE = 0
    };

    VehicleManager(void);

    /* getter */
    int  size(void) const;

    bool isVisit(int customer) const;
    bool isVisitAll(const vrp_problem *vrp) const;
    bool isFinish(const vrp_problem *vrp);
    bool canVisit(const vrp_problem *vrp, int customer);

    bool move(const vrp_problem *vrp, int move);
    int  computeTotalCost(const vrp_problem *vrp) const;

    VehicleManager copy(void) const;

    void print(void) const;
   
private:
    static const int VEHICLE_MAX  = 20;
    static const int CUSTOMER_MAX = 200;
    Vehicle vehicle[VEHICLE_MAX];
    bool isVisit_[CUSTOMER_MAX];
    int  size_; /* 車体の数 */
};

#endif /* VRPSOLVER_CPP_VEHICLE_MANAGER_H */
