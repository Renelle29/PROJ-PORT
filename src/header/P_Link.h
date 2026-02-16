#ifndef P_LINK_H
#define P_LINK_H

using namespace std;

/*
Physical Link class
This class represents a physical link in the network.
It contains information about the link's ID, origin, destination and length.
*/
class P_Link
{
private:
    int ID;
    int origin;
    int destination;
    int length;

public:
    P_Link(int id, int o, int d, int l);
    P_Link();

    int get_ID();
    int get_origin();
    int get_destination();
    int get_length();
};

#endif
