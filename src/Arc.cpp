#include "Arc.h"

Arc::Arc(int id, ArcType ty, int o, int d, int w, int c, int t)
{
    ID = id;
    type = ty;
    from = o;
    to = d;
    cost = w * t + c;
    s = 0;
}

Arc::Arc(int id, ArcType ty, int o, int d)
{
    ID = id;
    type = ty;
    from = o;
    to = d;
    cost = 0;
    s = 0;
}

Arc::Arc()
{
    ID = 0;
    type = DEFAULT;
    from = 0;
    to = 0;
    cost = 0;
    s = 0;
}

void Arc::initialise_reduced_cost(int nt)
{
    rho.clear();
    rho = vector<double>(nt, 0.0);
    for (int k = 1; k <= nt; k++)
    {
        rho[k - 1] = static_cast<double>(get_cost(k));
    }
}

void Arc::update_reduced_cost(int k, double d)
{
    rho[k - 1] += d;
}

void Arc::update_reduced_cost(double d)
{
    for (double &rho_k : rho)
    {
        rho_k += d;
    }
}

