#ifndef SERVICE_H
#define SERVICE_H

#include <string>
#include <vector>
using namespace std;

/*
Service class
Contains all necessary information to a given service type.
Their id, name, fixed cost and duration
*/
class Service
{
private:
    int ID;
    string name;
    int cost;
    int duration;
    vector<int> arcs; // corresponding service arcs

public:
    Service(int id, string n, int c, int d);
    Service();

    void addArc(int a);

    int getID();
    string getName();
    int getCost();
    int getDuration();
    vector<int> getArcs() { return arcs; }
};

#endif
