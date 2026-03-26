#ifndef PATH_BUILDER_H
#define PATH_BUILDER_H

#include "Train.h"
#include "Arc.h"
#include "Node.h"
#include "P_Node.h"
#include "Service.h"
#include "gurobi_c++.h"
#include "Path.h"

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

int build_train_apaths(vector<Train> &trains, vector<pair<double,Path>> extended_paths);
int build_train_apaths_old(vector<Train> &trains, vector<Path> paths);

// Builds each train's node path based on the solution given by the solver and checks if it respects constraints
int build_train_npaths(vector<Train> &trains, vector<Arc> arcs);

// Checks for each service if the service constraint was respected
int check_services(vector<Train> &trains, vector<Service> services);

int print_path(Train train, vector<Node> nodes, vector<P_Node> pNodes, vector<Service> services);

#endif
