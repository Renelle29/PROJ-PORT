#ifndef ARC_H
#define ARC_H

#include <vector>
#include <utility>

using namespace std;

enum ArcType
{
    ARRIVAL,
    DEPARTURE,
    SHUNTING,
    SERVICE,
    STORAGE,
    DWELLING,
    STATE_TRANSFER,
    DUMMY,
    DEFAULT
};

/*
Time space Arc class
Represents an arc in the virtual time-space network.
Contains an ID, type, from/to node indices, and costs per train.
*/
class Arc
{
private:
    int ID;
    ArcType type;
    int from;
    int to;
    int cost;
    vector<int> cost_by_train;
    int s;              // id of associated service if necessary, 0 otherwise
    vector<double> rho; // reduced cost
    vector<pair<int, int>> iNodes; // nodes for which arc is in incompatible node set

public:
    Arc(int id, ArcType ty, int o, int d, int w, int c, int t); // for all arcs
    Arc(int id, ArcType ty, int o, int d);                      // for cost 0 arcs
    Arc();

    void initialise_reduced_cost(int nt);
    void update_reduced_cost(int k, double d);
    void update_reduced_cost(double d);
    void add_node(int p, int t) { iNodes.push_back({p, t}); }
    void set_service(int s_id) { s = s_id; }
    void set_costs(const vector<int> &costs) { cost_by_train = costs; }

    int get_from() { return from; }
    int get_to() { return to; }
    int get_cost(int k) { return cost_by_train.empty() ? cost : cost_by_train[k - 1]; }
    int get_service() { return s; }
    double get_r_cost(int k) { return rho[k - 1]; }
    ArcType get_type() { return type; }
    vector<pair<int, int>> get_iNodes() { return iNodes; }
    int get_ID() { return ID; }
};
#endif
