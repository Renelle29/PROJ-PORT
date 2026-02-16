#ifndef NODE_H
#define NODE_H

#include <vector>
using namespace std;
/*
Time space network Node class
This class represents a node in the time space network.
It contains information about the node's ID, the physical ID (which physical node it refers to),
time coordinate and layer coordinate
*/
class Node
{
private:
    int ID;            // ID of the virtual node
    int p_ID;          // ID of the physical node
    int time;          // Time coordinate of the node
    int layer;         // Layer of the node in the time space network
    vector<int> iArcs; // vector of incompatible arcs for the node
    vector<int> fArcs; // that leave from this node
    vector<int> tArcs; // arcs that reach this node
public:
    Node(int id, int p, int t, int l);
    Node();

    void addIncArc(int a) { iArcs.push_back(a); }
    void addFromArc(int a) { fArcs.push_back(a); }
    void addToArc(int a) { tArcs.push_back(a); }

    int getPNode();
    int getTime();
    vector<int> getIncArcs() { return iArcs; }
    vector<int> getFromArcs() { return fArcs; }
    vector<int> getToArcs() { return tArcs; }
};

/*
Helper function that computes virtual node id based on its coordinates
(p-1)*|L| + (t/t_s)*|P|*|L| + l + 2
|L| = 3
*/
int n_id(int p, int t, int l, int P, int t_s);

#endif // NODE_H
