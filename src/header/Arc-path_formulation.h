#ifndef ARC_PATH_H
#define ARC_PATH_H

#include "gurobi_c++.h"
#include "Path.h"
#include "Train.h"
#include "Arc.h"
#include "Node.h"
#include <math.h>
#include <limits>
#include <chrono>
#include <tuple>
#include <algorithm>

using namespace std;
using namespace std::chrono;

const double INF = numeric_limits<double>::infinity();

struct RcspResult
{
    double cost;
    vector<int> arcs;
};

int initialize_master_AP(GRBModel &master, GRBLinExpr &obj, vector<Path> &paths, vector<vector<bool>> &s_assigned, vector<GRBConstr> &path_constraints, vector<Train> trains, vector<Arc> arcs, int nn, int T, int t_s, int ns);

int build_dual_vectors(vector<double> &pi, vector<vector<double>> &alpha, vector<vector<double>> &mu, vector<GRBConstr> &path_constraints, int nt, vector<vector<bool>> s_assigned, int nn, int T, int t_s);

int compute_reduced_cost_network(vector<Arc> &arcs, int nt, vector<vector<double>> alpha, vector<vector<double>> mu, int t_s, bool include_alpha);

int pricing_algorithm(vector<Path> &paths, GRBModel &master, GRBLinExpr &obj, vector<GRBConstr> &path_constraints, vector<Train> trains, vector<Node> nodes, vector<Arc> arcs, vector<vector<bool>> s_assigned, vector<vector<double>> alpha, vector<double> pi, int nn, int T, int t_s, int ns, vector<double> *best_rcosts = nullptr, int k_paths=1);

vector<RcspResult> shortest_path_algorithm_rcsp(vector<Node> nodes, vector<Arc> arcs, int nn, Train train, int T, int t_s, vector<vector<double>> alpha, int k_paths = 1);

int get_constraint_id(int k, int nt, int ns);
int get_constraint_id(int k, int s, int nt, int ns);
int get_constraint_id(int p, int t, int nt, int ns, int nn, int T, int t_s);

void solve_binary_model(GRBModel &master, vector<Path> &paths);

#endif
