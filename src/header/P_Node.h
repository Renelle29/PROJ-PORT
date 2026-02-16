#ifndef P_Node_H
#define P_Node_H

#include <iostream>
#include <string>
#include <vector>
using namespace std;

/*
Physical Node class
This class represents a physical node in the network.
It contains information about the node's ID, name, and type.
*/
class P_Node
{
private:
    int ID;
    string name;
    int type;

public:
    P_Node(int id, string n, int t);
    P_Node();

    int getID();
    string getName();
    int getType();
};

/*
Node type class
Contains all necessary information to a node type and allowed services
id, name, io, parking, dwelling, services and a list of nodes of its type
*/
class Node_Type
{
private:
    int ID;
    string name;
    bool io;
    bool parking;
    bool dwelling;
    vector<bool> services;
    vector<int> pnodes;

public:
    Node_Type(int id, string n, bool i, bool p, bool d, vector<bool> s);
    Node_Type();

    void addService(bool s) { services.push_back(s); }
    void addNode(int p);

    int getID();
    string getName();
    bool getIO();
    bool getParking();
    bool getDwelling();
    vector<bool> getServices() { return services; }
    bool getService(int i);
    vector<int> getNodes() { return pnodes; }
};

#endif
