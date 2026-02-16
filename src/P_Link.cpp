#include "P_Link.h"

using namespace std;

// constructor
P_Link::P_Link(int id, int o, int d, int l)
{
    ID = id;
    origin = o;
    destination = d;
    length = l;
}

P_Link::P_Link()
{
    ID = 0;
    origin = 0;
    destination = 0;
    length = 0;
}

// getters
int P_Link::get_ID()
{
    return ID;
}
int P_Link::get_origin()
{
    return origin;
}
int P_Link::get_destination()
{
    return destination;
}
int P_Link::get_length()
{
    return length;
}
