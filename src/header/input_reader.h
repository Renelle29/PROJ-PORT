#ifndef INPUT_READER_H
#define INPUT_READER_H

#include "P_Node.h"
#include "P_Link.h"
#include "Service.h"
#include "Train.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>

using namespace std;

vector<vector<string>> readCSV(const string &filename);

/*
Reads input filename that should be a csv in the right format containing service data
Stores it in a pointer array services given in input.
Returns the number of services or 0 if an issue occurs
*/
int read_service_input(const string &filename, vector<Service> &services);

/*
Reads input filename that should be a csv in the right format containing node type data
Stores it in a pointer array node_types given in input.
Returns the number of node types or 0 if an issue occurs
*/
int read_node_type_input(const string &filename, vector<Node_Type> &node_types, int nservices);

/*
Reads input filename that should be a csv in the right format containing physical node data
Stores it in a pointer array pnodes given in input.
Returns the number of nodes or 0 if an issue occurs
*/
int read_node_input(const string &filename, vector<P_Node> &pnodes, vector<Node_Type> &node_types, int ntypes);

/*
Reads input filename that should be a csv in the right format containing physical link data.
Stores it in a pointer array links.
Returns the number of links or 0 if an issue occurs
*/
int read_link_input(const string &filename, vector<P_Link> &links, vector<P_Node> &pnodes, int nnodes);

int read_train_type(const string &filename, vector<Train_Type> &ttypes);

int read_train_input(const string &filename, vector<Train> &trains, int ntypes, int nservices, int &T);

/*
Retrieves operation assignment input and returns a vector containing the number of assignement of each type.
Signals in the output that there is an error in the input file but will not stop the process of the model and continue with
no assigned operations
*/
vector<int> read_operation_input(const string &filename, vector<Train> &trains, int ntrains, int nservices);

#endif // TOOLS_H
