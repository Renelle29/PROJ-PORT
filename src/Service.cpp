#include "Service.h"

using namespace std;

// Constructors
Service::Service(int id, string n, int c, int d)
{
    ID = id;
    name = n;
    cost = c;
    duration = d;
    arcs = vector<int>();
}
Service::Service()
{
    ID = 0;
    name = "default";
    cost = 0;
    duration = 0;
    arcs = vector<int>();
}
void Service::addArc(int a)
{
    arcs.push_back(a);
}

// Getters
int Service::getID()
{
    return ID;
}
string Service::getName()
{
    return name;
}
int Service::getCost()
{
    return cost;
}
int Service::getDuration()
{
    return duration;
}
