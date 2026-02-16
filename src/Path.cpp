#include "Path.h"

Path::Path()
{
    ID = 0;
    arc_ids = {};
    cost = 0;
    train = 0;
}

Path::Path(int id, vector<int> a, int c, int k, int ns)
{
    ID = id;
    arc_ids = a;
    cost = c;
    train = k;
    services = vector<bool>(ns, false);
}

Path::Path(int id, vector<Arc> arcs, vector<int> aPath, int k, int ns)
{
    ID = id;
    cost = 0;
    train = k;
    arc_ids = aPath;
    services = vector<bool>(ns, false);
    for (int a_id : aPath)
    {
        cost += arcs[a_id].get_cost(k);
        if (arcs[a_id].get_type() == SERVICE)
        {
            services[arcs[a_id].get_service() - 1] = true;
        }
    }
}

void Path::build_GRBVar(GRBModel &model, GRBLinExpr &obj)
{
    lambda = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS); // adds variable as a continuous variable
    obj += static_cast<double>(cost) * lambda;                     // updates objective function
}

void Path::update_service(int s)
{
    if (s > static_cast<int>(services.size()) || s <= 0)
    {
        cerr << "Setting inexistant service" << endl;
        return;
    }
    services[s - 1] = true;
}

void Path::build_GRBVar(GRBModel &model, GRBLinExpr &obj, GRBColumn &column)
{
    lambda = model.addVar(0.0, 1.5, 0.0, GRB_CONTINUOUS, column); // Testing different upper bounds to get Gurobi to cooperate
    obj += static_cast<double>(cost) * lambda;
}
