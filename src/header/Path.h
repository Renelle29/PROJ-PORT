#ifndef PATH_H
#define PATH_H

#include <vector>
#include "Arc.h"

#include "gurobi_c++.h"
using namespace std;

class Path
{
private:
    int ID;                // Path id
    vector<int> arc_ids;   // Set of arcs in the path
    vector<bool> services; // Set of services performed by the path
    int cost;              // Total cost of the path
    GRBVar lambda;         // decision variable
    int train;             // id of the associated train

public:
    Path();
    Path(int id, vector<int> a, int c, int k, int ns);
    Path(int id, vector<Arc> arcs, vector<int> aPath, int k, int ns);

    void update_service(int s);

    GRBVar get_lambda() const { return lambda; }
    bool get_service(int s) const { return services[s - 1]; }
    vector<int> get_arcs() const { return arc_ids; }
    int get_id() const { return ID; }
    int get_train() const { return train; }

    void build_GRBVar(GRBModel &model, GRBLinExpr &obj);
    void build_GRBVar(GRBModel &model, GRBLinExpr &obj, GRBColumn &column);
};

#endif
