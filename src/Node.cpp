#include "Node.h"

using namespace std;
Node::Node(int id, int p, int t, int l)
{
    ID = id;
    p_ID = p;
    time = t;
    layer = l;
    iArcs = vector<int>();
    fArcs = vector<int>();
    tArcs = vector<int>();
}

Node::Node()
{
    ID = 0;
    p_ID = 0;
    time = 0;
    layer = 100;
    iArcs = vector<int>();
    fArcs = vector<int>();
    tArcs = vector<int>();
}

int Node::getPNode()
{
    return p_ID;
}
int Node::getTime()
{
    return time;
}

int n_id(int p, int t, int l, int P, int t_s)
{
    if (p == 0)
    {
        return l - 1;
    }
    return (p - 1) * 3 + (t / t_s) * P * 3 + l + 2;
}
