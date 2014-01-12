#ifndef VRPSOLVER_CPP_SOLVER_H
#define VRPSOLVER_CPP_SOLVER_H

namespace Solver
{
    void setProblem(char *filename);
    void setSeed(long seed);
    void setMctsIterationCount(int count);
    void setSimulationCount(int count);
    void run(void);
}

#endif /* VRPSOLVER_CPP_SOLVER_H */
