#include "network_builder.h"

int compute_time_step(vector<Train> trains, vector<Service> services, vector<Train_Type> train_types, vector<P_Link> plinks)
{
    int t = 0; // time step
    for (Train train : trains)
    {
        t = __gcd(t, train.get_a_t());
        t = __gcd(t, train.get_d_t());
    }
    for (Service service : services)
    {
        t = __gcd(t, service.getDuration());
    }
    for (Train_Type train_type : train_types)
    {
        t = __gcd(t, train_type.get_t_turn());
    }
    for (P_Link link : plinks)
    {
        t = __gcd(t, link.get_length());
    }
    // cout << "Time step: " << t << endl;

    return t;
}

int build_nodes(vector<Node> &nodes, int n_pnodes, int T, int t_s)
{
    nodes.push_back(Node(0, 0, 0, 1)); // origin node
    nodes.push_back(Node(1, 0, 0, 2)); // destination node
    int id = 1;
    for (int t = 0; t <= T; t += t_s)
    {
        for (int p = 1; p <= n_pnodes; p++)
        {
            for (int l = 0; l < 3; l++)
            {
                id += 1;
                nodes.push_back(Node(id, p, t, l));
            }
        }
    }
    return id;
}

vector<int> build_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Service> &services, vector<Train_Type> train_types, vector<P_Link> plinks, vector<Node_Type> node_types, int nn, int nt, int ns, int T, int t_s)
{
    vector<int> na = {0};
    na.push_back(build_a_d_arcs(arcs, trains, nodes, node_types, nt, nn, T, t_s));
    na.push_back(build_shunting_arcs(arcs, trains, nodes, train_types, plinks, T, nn, t_s));
    na.push_back(build_service_arcs(arcs, services, trains, nodes, node_types, nn, T, t_s));
    na.push_back(build_storage_arcs(arcs, trains, nodes, train_types, node_types, nn, T, t_s));
    na.push_back(build_dwelling_arcs(arcs, trains, nodes, node_types, nn, T, t_s));
    na.push_back(build_state_transfer_arcs(arcs, trains, nodes, node_types, nn, ns, T, t_s));
    na.push_back(build_dummy_arcs(arcs, trains, services, train_types));
    return na;
}

void build_a_d_arcs_k(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, int k, vector<Node_Type> node_types, int nn, int T, int t_s)
{
    int a_t = trains[k - 1].get_a_t();
    int d_t = trains[k - 1].get_d_t();
    int a_id = arcs.size();
    int v;
    for (Node_Type type : node_types)
    {
        if (type.getIO())
        {
            for (int p : type.getNodes())
            {
                v = n_id(p, a_t, 1, nn, t_s);
                arcs.push_back(Arc(a_id, ARRIVAL, 0, v));
                for (Train &train : trains)
                {
                    train.add_arcb(train.get_ID() == k);
                }
                nodes[0].addFromArc(a_id);
                nodes[v].addToArc(a_id);
                a_id++;
                v = n_id(p, d_t, 2, nn, t_s);
                arcs.push_back(Arc(a_id, DEPARTURE, v, 1));
                for (Train &train : trains)
                {
                    train.add_arcb(train.get_ID() == k);
                }
                nodes[v].addFromArc(a_id);
                nodes[0].addToArc(a_id);
                for (int t = 0; t < min(t_h, T - d_t + 1); t += t_s)
                { // add incompatible arc to each corresponding node during the safety headway interval
                    v = n_id(p, d_t + t, 0, nn, t_s);
                    nodes[v].addIncArc(a_id);
                    arcs[a_id].add_node(p, d_t + t);
                }
                a_id++;
            }
        }
    }
}

int build_a_d_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Node_Type> node_types, int nt, int nn, int T, int t_s)
{
    for (int k = 1; k <= nt; k++)
    {
        build_a_d_arcs_k(arcs, trains, nodes, k, node_types, nn, T, t_s);
    }
    return arcs.size();
}

void build_shunting_arc(int a_id, int p_id, int q_id, int l, int t1, int d, vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, const vector<Train_Type> &train_types, int T, int nn, int t_s)
{
    int n1 = n_id(p_id, t1, l, nn, t_s);
    int n2 = n_id(q_id, t1 + d, l, nn, t_s);
    int v;
    arcs.push_back(Arc(a_id, SHUNTING, n1, n2, w_shu, c_shu, d));
    vector<int> costs;
    costs.reserve(trains.size());
    for (const Train &train : trains)
    {
        int w = train_types[train.get_type() - 1].get_w_shu();
        costs.push_back(w * d + c_shu);
    }
    arcs[a_id].set_costs(costs);
    nodes[n2].addToArc(a_id);
    nodes[n1].addFromArc(a_id);
    for (int dt = 0; dt < min(t_h, T - t1 + 1); dt += t_s)
    { // add incompatible arc to each corresponding departure node during the safety headway interval
        v = n_id(p_id, t1 + dt, 0, nn, t_s);
        nodes[v].addIncArc(a_id);
        arcs[a_id].add_node(p_id, dt + t1);
    }
    for (Train &train : trains) // Add arcs to train arcs
    {
        train.add_arcb(true);
    }
}

int build_shunting_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Train_Type> train_types, vector<P_Link> plinks, int T, int nn, int t_s)
{
    int p_id;
    int q_id;
    int l;
    int a_id = arcs.size();
    int d; // shunting duration

    for (P_Link link : plinks)
    {
        d = link.get_length();
        p_id = link.get_origin();
        q_id = link.get_destination();
        for (int t1 = 0; t1 < T; t1 += t_s)
        {
            if (t1 + d > T)
            {
                break;
            }
            // Layer 1 arc
            l = 1;
            build_shunting_arc(a_id, p_id, q_id, l, t1, d, arcs, trains, nodes, train_types, T, nn, t_s);
            a_id++;
            // Layer 2 arc
            l = 2;
            build_shunting_arc(a_id, q_id, p_id, l, t1, d, arcs, trains, nodes, train_types, T, nn, t_s);
            a_id++;
        }
    }

    return a_id;
}

void build_service_arc(int a_id, int p_id, int l, int t1, int d, int s, int w, vector<Arc> &arcs, vector<Service> &services, vector<Train> &trains, vector<Node> &nodes, int nn, int t_s)
{
    int n1 = n_id(p_id, t1, l, nn, t_s);
    int n2 = n_id(p_id, t1 + d, 0, nn, t_s);
    int v;
    int c = services[s - 1].getCost();

    arcs.push_back(Arc(a_id, SERVICE, n1, n2, w, c, d)); // Add arc to arcs
    arcs[a_id].set_service(s);
    services[s - 1].addArc(a_id); // add to corresponding service arc set
    nodes[n2].addToArc(a_id);
    nodes[n1].addFromArc(a_id);
    for (int dt = 0; dt < d; dt += t_s)
    { // add incompatible arc to each node it occupies during service
        v = n_id(p_id, t1 + dt, 0, nn, t_s);
        nodes[v].addIncArc(a_id);
        arcs[a_id].add_node(p_id, t1 + dt);
    }
    for (Train &train : trains) // Add arcs to train arcs
    {
        train.add_arcb(train.get_service(s));
    }
}

int build_service_arcs(vector<Arc> &arcs, vector<Service> &services, vector<Train> &trains, vector<Node> &nodes, vector<Node_Type> node_types, int nn, int T, int t_s)
{
    int a_id = arcs.size();
    int d;
    int w = 0;
    for (Service service : services) // For each service...
    {
        d = service.getDuration();
        for (Node_Type type : node_types) // if the node authorizes this service...
        {
            if (type.getService(service.getID()))
            {
                for (int p_id : type.getNodes())
                {
                    for (int l = 1; l < 3; l++) // for each potential starting layer...
                    {
                        for (int t1 = 0; t1 <= T - d; t1 += t_s) // for each time instant.
                        {
                            build_service_arc(a_id, p_id, l, t1, d, service.getID(), w, arcs, services, trains, nodes, nn, t_s);
                            a_id++;
                        }
                    }
                }
            }
        }
    }
    return a_id;
}

void build_storage_arc(int a_id, int p_id, int l, int t1, int d, int ttype, vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, int nn, int t_s)
{
    int n1 = n_id(p_id, t1, l, nn, t_s);
    int n2 = n_id(p_id, t1 + d, 0, nn, t_s);
    int v;

    arcs.push_back(Arc(a_id, STORAGE, n1, n2, w_dwell, c_park, d)); // Add arc to arcs

    nodes[n2].addToArc(a_id);
    nodes[n1].addFromArc(a_id);
    for (int dt = 0; dt < d; dt += t_s)
    { // add incompatible arc to each node it occupies during service
        v = n_id(p_id, t1 + dt, 0, nn, t_s);
        nodes[v].addIncArc(a_id);
        arcs[a_id].add_node(p_id, dt + t1);
    }
    for (Train &train : trains) // Add arcs to train arcs
    {
        train.add_arcb(train.get_type() == ttype);
    }
}

int build_storage_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Train_Type> train_types, vector<Node_Type> node_types, int nn, int T, int t_s)
{
    int a_id = arcs.size();
    int d;

    for (Node_Type ntype : node_types)
    {
        if (ntype.getParking())
        {
            for (Train_Type ttype : train_types)
            {
                d = ttype.get_t_turn();
                for (int p_id : ntype.getNodes())
                {
                    for (int l = 1; l < 3; l++)
                    {
                        for (int t1 = 0; t1 <= T - d; t1 += t_s)
                        {
                            build_storage_arc(a_id, p_id, l, t1, d, ttype.get_ID(), arcs, trains, nodes, nn, t_s);
                            a_id++;
                        }
                    }
                }
            }
        }
    }
    return a_id;
}

void build_dwelling_arc(int a_id, int p_id, int t1, vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, int nn, int t_s)
{
    int n1 = n_id(p_id, t1, 0, nn, t_s);
    int n2 = n_id(p_id, t1 + t_s, 0, nn, t_s);

    arcs.push_back(Arc(a_id, DWELLING, n1, n2, w_dwell, 0, t_s));
    nodes[n2].addToArc(a_id);
    nodes[n1].addFromArc(a_id);

    nodes[n1].addIncArc(a_id);
    arcs[a_id].add_node(p_id, t1);
    for (Train &train : trains)
    {
        train.add_arcb(true);
    }
}

int build_dwelling_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Node_Type> node_types, int nn, int T, int t_s)
{
    int a_id = arcs.size();
    for (Node_Type ntype : node_types)
    {
        if (ntype.getDwelling())
        {
            for (int p_id : ntype.getNodes())
            {
                for (int t1 = 0; t1 < T; t1 += t_s)
                {
                    build_dwelling_arc(a_id, p_id, t1, arcs, trains, nodes, nn, t_s);
                    a_id++;
                }
            }
        }
    }
    return a_id;
}

void build_state_transfer_arc(int a_id, int p_id, int l, int t1, vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, int nn, int t_s)
{
    int n1 = n_id(p_id, t1, 0, nn, t_s);
    int n2 = n_id(p_id, t1, l, nn, t_s);

    arcs.push_back(Arc(a_id, STATE_TRANSFER, n1, n2)); // add arc
    nodes[n2].addToArc(a_id);
    nodes[n1].addFromArc(a_id);
    for (Train &train : trains)
    {
        train.add_arcb(true);
    }
}

int build_state_transfer_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Node> &nodes, vector<Node_Type> node_types, int nn, int ns, int T, int t_s)
{
    int a_id = arcs.size();
    bool can_0; // stores wether a type can enter dwelling layer
    for (Node_Type ntype : node_types)
    {
        can_0 = ntype.getParking();
        for (int s = 1; s <= ns; s++)
        {
            can_0 = can_0 || ntype.getService(s);
        }
        if (can_0)
        {
            for (int p_id : ntype.getNodes())
                for (int l = 1; l < 3; l++)
                {
                    for (int t1 = 0; t1 < T; t1 += t_s)
                    {
                        build_state_transfer_arc(a_id, p_id, l, t1, arcs, trains, nodes, nn, t_s);
                        a_id++;
                    }
                }
        }
    }
    return a_id;
}

void build_dummy_arc_k(int a_id, int k, int w_shu, vector<Arc> &arcs, vector<Train> &trains, vector<Service> &services)
{
    int d = trains[k - 1].get_d_t() - trains[k - 1].get_a_t();
    int w = w_shu + w_dwell + c_park + c_shu;
    int c = 0;

    for (Service &s : services)
    {
        c += s.getCost();
    }

    c = 2 * c;

    arcs.push_back(Arc(a_id, DUMMY, 0, 1, w, c, d));
    trains[k - 1].set_dummy(a_id);
    for (Train &train : trains)
    {
        train.add_arcb(train.get_ID() == k);
    }
}

int build_dummy_arcs(vector<Arc> &arcs, vector<Train> &trains, vector<Service> &services, vector<Train_Type> train_types)
{
    int a_id = arcs.size();
    int w_shu;
    for (Train &train : trains)
    {
        w_shu = train_types[train.get_type() - 1].get_w_shu();
        build_dummy_arc_k(a_id, train.get_ID(), w_shu, arcs, trains, services);
        a_id++;
    }
    return a_id;
}

