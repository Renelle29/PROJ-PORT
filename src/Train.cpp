#include "Train.h"
using namespace std;

Train::Train(int id, string n, int t, int a, int b, int ns)
{
    ID = id;
    name = n;
    type = t;
    a_t = a;
    d_t = b;
    Npath = {};
    Apath = {};
    for (int s = 0; s < ns; s++)
    {
        services.push_back(false);
        p_services.push_back(0);
    }
    A_b = {};
    dummy = 0;
}
Train::Train()
{
    ID = 0;
    name = "default";
    type = 0;
    a_t = 0;
    d_t = 0;
    Npath = {};
    Apath = {};
    services = {};
    p_services = {};
    A_b = {};
    dummy = 0;
}

int Train::get_ID() const
{
    return ID;
}
string Train::get_name() const
{
    return name;
}
int Train::get_type() const
{
    return type;
}
int Train::get_a_t() const
{
    return a_t;
}
int Train::get_d_t() const
{
    return d_t;
}
vector<bool> Train::get_services() const
{
    return services;
}

Train_Type::Train_Type(int id, string n, int w, int t)
{
    ID = id;
    name = n;
    w_shu = w;
    t_turn = t;
}
Train_Type::Train_Type()
{
    ID = 0;
    name = "default";
    w_shu = 0;
    t_turn = 0;
}

int Train_Type::get_ID() const
{
    return ID;
}
string Train_Type::get_name() const
{
    return name;
}
int Train_Type::get_w_shu() const
{
    return w_shu;
}
int Train_Type::get_t_turn() const
{
    return t_turn;
}
