#include "P_Node.h"

// constructor
P_Node::P_Node(int id, string n, int t)
{
    ID = id;
    name = n;
    type = t;
}

P_Node::P_Node()
{
    ID = 0;
    name = "default";
    type = 0;
}

// getters
int P_Node::getID()
{
    return ID;
}
string P_Node::getName()
{
    return name;
}
int P_Node::getType()
{
    return type;
}

Node_Type::Node_Type(int id, string n, bool i, bool p, bool d, vector<bool> s)
{
    ID = id;
    name = n;
    io = i;
    parking = p;
    dwelling = d;
    services = s;
    pnodes = {};
}
Node_Type::Node_Type()
{
    ID = 0;
    name = "default";
    io = false;
    parking = false;
    dwelling = false;
    services = vector<bool>(0);
    pnodes = {};
}

// print node type information
void Node_Type::addNode(int p)
{
    pnodes.push_back(p);
}

// getters
int Node_Type::getID()
{
    return ID;
}
string Node_Type::getName()
{
    return name;
}
bool Node_Type::getIO()
{
    return io;
}
bool Node_Type::getParking()
{
    return parking;
}
bool Node_Type::getDwelling()
{
    return dwelling;
}
bool Node_Type::getService(int i)
{
    if (i <= 0 || i > static_cast<int>(services.size()))
    {
        cout << "Invalid index" << endl;
        return 0;
    }
    return services[i - 1];
}
