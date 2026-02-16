#ifndef NETWORK_BUILDER_H
#define NETWORK_BUILDER_H

#include "Node.h"
#include "P_Node.h"
#include "P_Link.h"
#include "Train.h"
#include "Arc.h"
#include "Service.h"
#include "params.h"
#include <algorithm>

const int w_shu = 2;

using namespace std;

/*
Computes minimum time step based on input data to avoid building too many arcs or constraints.
*/
int compute_time_step(vector<Train> trains, vector<Service> services, vector<Train_Type> train_types, vector<P_Link> plinks);

/*
Time-space-layer node building function
Based on physical nodes and time frame, builds all nodes in the nodes vector
Nodes 0 and 1 are by definition dummy origin and destination nodes
Then builds all nodes in chronological order, at each time stamp building each P_Node p for each layer.
For a given node (p, t, l), its node index is:
(p-1)*|L| + (t/t_s)*|P|*|L| + l + 2
*/
int build_nodes(vector<Node> &nodes, int n_pnodes, int T, int t_s);

/*
Builds all time-space layer arcs and adds them to arcs, train lists, incompatible arc nodes, and service lists when necessary
Calls all other build_arcs functions.
*/
vector<int> build_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Service> &services, vector<Train_Type> train_types, vector<P_Link> plinks, vector<Node_Type> node_types, int nn, int nt, int ns, int T, int t_s);

/*
Builds arrival and departure arcs for train k. The number of physical nodes is used to compute each virtual node's index.
Adds these arcs to the arcs vector, to train k's specific arcs and to each node's corresponding incompatibility arc sets
*/
void build_a_d_arcs_k(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, int k, vector<Node_Type> node_types, int nn, int T, int t_s);

// Builds arrival and departure arcs for all trains, calling build_a_d_arcs_k
int build_a_d_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Node_Type> node_types, int nt, int nn, int T, int t_s);

/*
Builds shunting arc a_id of duration d from physical nodes p_id to q_id in layer l in {1,2} at time t1
Updates arcs, trains and nodes with the arc in the authorized and incompatible arc sets.
*/
void build_shunting_arc(int a_id, int p_id, int q_id, int l, int t1, int d, vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, const vector<Train_Type> &train_types, int T, int nn, int t_s);

// Builds all shunting arcs, calling build_shunting_arc
int build_shunting_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Train_Type> train_types, vector<P_Link> plinks, int T, int nn, int t_s);

/*
Builds service arc a_id of duration d from physical node p_id in layer l at time t1
Updates arcs, trains, services and nodes with the arc in the authorized and incompatible arc sets.
*/
void build_service_arc(int a_id, int p_id, int l, int t1, int d, int s, int w, vector<Arc> &arcs, vector<Service> &services, vector<Train> &trains, vector<Node> &nodes, int nn, int t_s);

// builds all service arcs, calling build_service_arc
int build_service_arcs(vector<Arc> &arcs, vector<Service> &services, vector<Train> &trains, vector<Node> &nodes, vector<Node_Type> node_types, int nn, int T, int t_s);

/*
Builds storage arc a_id of duration d from physical node p_id in layer l at time t1
Updates arcs, trains and nodes with the arc in the authorized and incompatible arc sets.
*/
void build_storage_arc(int a_id, int p_id, int l, int t1, int d, int type, vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, int nn, int t_s);

// builds all storage arcs, calling build_storage_arc
int build_storage_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Train_Type> train_types, vector<Node_Type> node_types, int nn, int T, int t_s);

/*
Builds dwelling arc a_id of duration 1 from physical node p_id in layer l at time t1
Updates arcs, trains and nodes with the arc in the authorized and incompatible arc sets.
*/
void build_dwelling_arc(int a_id, int p_id, int t1, vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, int nn, int t_s);

// builds all dwelling arcs, calling build_dwelling_arc
int build_dwelling_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Node_Type> node_types, int nn, int T, int t_s);

/*
Builds state transfer arc a_id in physical node p_id to layer l at time t1
Updates arcs and trains with the arc in the authorized arc sets.
*/
void build_state_transfer_arc(int a_id, int p_id, int l, int t1, vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, int nn, int t_s);

// builds all state transfer arcs, calling build_state_transfer_arc
int build_state_transfer_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Node_Type> node_types, int nn, int ns, int T, int t_s);

/*
Builds dummy arc a_id for a given train k with it's weight
Updates arcs, train k and services with the arc in the authorized arc sets.
*/
void build_dummy_arc_k(int a_id, int k, int w_shu, vector<Arc> &arcs, vector<Train> &trains, vector<Service> &services);

// builds all dummy arcs, calling build_dummy_arc_k
int build_dummy_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Service> &services, vector<Train_Type> train_types);

#endif // NETWORK_BUILDER_H
