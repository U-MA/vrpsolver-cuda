#include <math.h>
#include <stdio.h>
#include <stdlib.h>

extern "C"
{
#include "vrp_types.h"
}

#include "Node.h"
#include "VehicleManager.h"
#include "VrpSimulation.h"


Node::Node(void)
{
    customer_  = 0;
    count_     = 0;
    childSize_ = 0;
    value_     = 0;
    child      = NULL;
    tabu_      = NULL;
}

Node::~Node(void)
{
    delete[] child;
    delete[] tabu_;
}

int Node::customer(void) const
{
    return customer_;
}

int Node::count(void) const
{
    return count_;
}

int Node::childSize(void) const
{
    return childSize_;
}

int Node::value(void) const
{
    return value_;
}

bool Node::tabu(int customer) const
{
    return tabu_[customer];
}

void Node::setTabu(int customer)
{
    tabu_[customer] = true;
}

void Node::expand(const vrp_problem *vrp, VehicleManager& vm)
{
    int childSize = 0;
    child = new Node[vrp->vertnum];
    tabu_ = new bool[vrp->vertnum];
    for (int i=0; i < vrp->vertnum; i++)
        tabu_[i] = true;

    /* 次の車体が存在 */
    if (vm.size() < vrp->numroutes)
    {
        child[childSize++].customer_ = VehicleManager::CHANGE; /* 各顧客が訪問可能か調べる */
        tabu_[VehicleManager::CHANGE] = false;
    }

    for (int i=1; i < vrp->vertnum; i++)
    {
        if (!vm.isVisit(i) && vm.canVisit(vrp, i))
        {
            child[childSize++].customer_ = i;
            tabu_[i] = false;
        }
    }
    childSize_ = childSize;
}

double Node::computeUcb(int parentCount)
{
    double ucb = 1e6 + (rand() % (int)1e6);
    if (count_ != 0)
        ucb = - value_ / count_ + 1.0 * sqrt(log((double)parentCount+1)) / count_;

    return ucb;
}

Node *Node::select(void)
{
    Node *selected = NULL;
    double maxUcb = -MISS;
    for (int i=0; i < childSize_; i++)
    {
        /* tabu_に含まれていれば飛ばす */
        if (tabu_[child[i].customer()]) continue;

        double ucb = child[i].computeUcb(count_);
        //fprintf(stderr, "child[%d].computeUcb(%d) is %lg and child[%d].count() is %d\n", i, count_, ucb, i, child[i].count());
        if (ucb > maxUcb)
        {
            maxUcb = ucb;
            selected = &child[i];
        }
    }

    return selected;
}

bool Node::isLeaf(void) const
{
    return (childSize_ == 0);
}

bool Node::isTabu(const vrp_problem *vrp) const
{
    for (int i=0; i < vrp->vertnum; i++)
    {
        if (!tabu_[i])
            return false;
    }
    return true;
}

void Node::update(int value)
{
    count_++;
    value_ += value;
}

void Node::search(const vrp_problem *vrp, const VehicleManager& vm)
{
    /* 引数として渡されるvmは変更しない
     * そのため変更させるための変数を作成 */
    VehicleManager vm_copy = vm.copy();

    Node *visited[300];
    int  visitedSize = 0;

    Node *node = this;

    /* nodeは訪問済み */
    visited[visitedSize++] = this;

    /* SELECTION */
    while (!node->isLeaf())
    {
        node = node->select();
        visited[visitedSize++] = node;
        //printf("node->customer() is %d\n", node->customer());
        vm_copy.move(vrp, node->customer());
    }

    /* nodeが全探索木の葉でなければexpandする*/
    if (!vm_copy.isFinish(vrp))
    {
        /* EXPANSION */
        node->expand(vrp, vm_copy);
        Node *newNode = node->select();
        visited[visitedSize++] = newNode;
        //printf("newNode->customer() is %d\n", newNode->customer());
        vm_copy.move(vrp, newNode->customer());
    }

    /* SIMULATION */
    int cost = VrpSimulation::sequentialRandomSimulation(vrp, vm_copy);

    /* BACKPROPAGATION */
    for (int i=0; i < visitedSize; i++)
        visited[i]->update(cost);
}

void Node::search(const vrp_problem *vrp, const VehicleManager& vm, int count)
{
    /* 引数として渡されるvmは変更しない
     * そのため変更させるための変数を作成 */
    VehicleManager vm_copy = vm.copy();

    Node *visited[300];
    int  visitedSize = 0;

    Node *node   = this;
    Node *parent = NULL;

    /* nodeは訪問済み */
    visited[visitedSize++] = this;

    fprintf(stderr, "\nMONTE CARLO TREE ROOT address %p IS ", node);

    /* SELECTION */
    while (!node->isLeaf())
    {
        fprintf(stderr, "NODE\n");
        parent = node;
        node = node->select();
        fprintf(stderr, "\tNODE address %p (HAVE CUSTOMER %d) IS ", node, node->customer());
        if (!node->isLeaf() && node->isTabu(vrp))
        {
            fprintf(stderr, "TABU\n");
            parent->setTabu(node->customer());
            return ; /* 探索を破棄 */
        }
        visited[visitedSize++] = node;
        //printf("node->customer() is %d\n", node->customer());
        vm_copy.move(vrp, node->customer());
    }

    fprintf(stderr, "LEAF\n");

    /* 後の操作で現在のVehiceManaagerの状態を使う必要が
     * あるかもしれないので、ここで記憶しておく
     * 囲碁将棋の「待った」から来ている */
    VehicleManager matta = vm_copy.copy();

    /* nodeが全探索木の葉でなければexpandする*/
    if (!vm_copy.isFinish(vrp))
    {
        /* EXPANSION */
        fprintf(stderr, "\tEXPAND\n");
        node->expand(vrp, vm_copy);
        parent = node;
        node = node->select();
        fprintf(stderr, "\t\tSELECTED NODE address %p HAVE CUSTOMER %d\n", node, node->customer());
        //printf("node->customer() is %d\n", node->customer());
        vm_copy.move(vrp, node->customer());
    }

    /* SIMULATION */
    int cost = MISS;
    while ((cost = VrpSimulation::sequentialRandomSimulation(vrp, vm_copy, count)) == MISS)
    {
        fprintf(stderr, "\t\t[SIMULATION RESULT] %d\n", cost);
        fprintf(stderr, "\t\t\tSO, NODE address %p ADD CUSTOMER %d TO TABU\n", parent, node->customer());
        parent->setTabu(node->customer());
        vm_copy = matta.copy(); /* VehicleManagerを直前の状態に移す */
        if (parent->isTabu(vrp))
        {
            fprintf(stderr, "\t\t\tPARENT NODE address %p IS TABU\n", parent);
            return ; /* 探索を破棄 */
        }
        node = parent->select();
        vm_copy.move(vrp, node->customer());
    }
    fprintf(stderr, "\t\t[SIMULATION RESULT] %d\n", cost);
    visited[visitedSize++] = node;

    /* BACKPROPAGATION */
    for (int i=0; i < visitedSize; i++)
        visited[i]->update(cost);
}


int Node::next(void) const
{
    int maxCount = -1;
    int move     = -1;
    for (int i=0; i < childSize_; i++)
    {
        int count = child[i].count();
        if (count > maxCount)
        {
            maxCount = count;
            move     = child[i].customer();
        }
    }

    return move;
}
