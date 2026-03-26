#include "Arc-path_formulation.h"

bool path_already_exists(vector<Path>& paths, vector<int>& new_arcs, int train_id)
{
    for (Path& p : paths)
    {
        if (p.get_train() != train_id)
            continue;
        if (p.get_arcs() == new_arcs)
            return true;
    }
    return false;
}

int initialize_master_AP(GRBModel &master, GRBLinExpr &obj, vector<Path> &paths, vector<vector<bool>> &s_assigned, vector<GRBConstr> &path_constraints, vector<Train> trains, vector<Arc> arcs, int nn, int T, int t_s, int ns, vector<Path> warm_paths)
{
    int k;        // working train id
    Path path;    // working path
    int p_id = 1; // path id
    int s_id;     // service id
    int c_id;
    GRBLinExpr L;

    for (Train train : trains) // For each train
    {
        L = 0;
        // Build initial path
        k = train.get_ID();
        s_id = 1;
        path = Path(p_id, {train.get_dummy()}, arcs[train.get_dummy()].get_cost(k), k, ns);
        path.build_GRBVar(master, obj);
        // path.print();

        // Build initial constraints
        L += path.get_lambda();
        path_constraints.push_back(master.addConstr(L, GRB_GREATER_EQUAL, 1.0, "Flow train " + to_string(k))); // Build flow constraint
        for (bool s : train.get_services())
        {
            s_assigned[k - 1].push_back(s);
            if (s)
            {
                path_constraints.push_back(master.addConstr(path.get_lambda(), GRB_GREATER_EQUAL, 1.0, "Train " + to_string(k) + " service " + to_string(s_id))); // Build service constraint
                path.update_service(s_id);
            }
            else
            {
                path_constraints.push_back(master.addConstr(path.get_lambda(), GRB_GREATER_EQUAL, 0.0, "Train " + to_string(k) + " service " + to_string(s_id)));
            }
            s_id++;
        }
        p_id++;
        paths.push_back(path);
    }

    for (int p = 1; p <= nn; p++)
    {
        for (int t = 0; t <= T; t += t_s)
        {
            path_constraints.push_back(master.addConstr(0.0, GRB_LESS_EQUAL, 1.0, "Incompatibility " + to_string(p) + " at " + to_string(t))); // build incompatibility constraint for node n
        }
    }

    int nt = trains.size();

    for (Path& path : warm_paths)
    {
        //cerr << "Reusing path : " << path.get_id() << endl;
        vector<int> arc_ids = path.get_arcs();
        if (path_already_exists(paths, arc_ids, path.get_train()))
            continue;

        int k = path.get_train();
        GRBColumn column = GRBColumn();

        // Train flow constraint
        c_id = get_constraint_id(k, nt, ns);
        column.addTerm(1.0, path_constraints[c_id]);

        // Service constraints
        s_id = 1;
        for (bool s : s_assigned[k - 1])
        {
            if (s)
            {
                if (path.get_service(s_id))
                {
                    c_id = get_constraint_id(k, s_id, nt, ns);
                    column.addTerm(1.0, path_constraints[c_id]);
                }
            }
            s_id++;
        }

        // Incompatibility constraints
        for (int a_id : path.get_arcs())
            for (pair<int, int> n : arcs[a_id].get_iNodes())
            {
                c_id = get_constraint_id(n.first, n.second, nt, ns, nn, T, t_s);
                column.addTerm(1.0, path_constraints[c_id]);
            }

        path.build_GRBVar(master, obj, column);
        paths.push_back(path);
        p_id++;
    }

    return p_id;
}

int build_dual_vectors(vector<double> &pi, vector<vector<double>> &alpha, vector<vector<double>> &mu, vector<GRBConstr> &path_constraints, int nt, vector<vector<bool>> s_assigned, int nn, int T, int t_s)
{
    int s_id;
    int c_id;
    int ns = s_assigned[0].size();

    for (int k = 1; k <= nt; k++) // For each train...
    {
        c_id = get_constraint_id(k, nt, ns);
        pi.push_back(path_constraints[c_id].get(GRB_DoubleAttr_Pi)); // and add its dual variable value to pi.

        s_id = 1;
        for (bool s : s_assigned[k - 1]) // For each service...
        {
            if (s) // if this service was assigned to the train...
            {
                c_id = get_constraint_id(k, s_id, nt, ns);
                alpha[s_id - 1].push_back(path_constraints[c_id].get(GRB_DoubleAttr_Pi)); // and add its dual variable value to alpha.
            }
            else // If it wasn't assigned to train k...
            {
                alpha[s_id - 1].push_back(0.0); // add 0 to alpha.
            }
            s_id++;
        }
    }

    for (int p = 1; p <= nn; p++)
    {
        for (int t = 0; t <= T; t += t_s)
        {
            c_id = get_constraint_id(p, t, nt, ns, nn, T, t_s);
            mu[p - 1].push_back(path_constraints[c_id].get(GRB_DoubleAttr_Pi));
        }
    }
    return 1;
}

int compute_reduced_cost_network(vector<Arc> &arcs, int nt, vector<vector<double>> alpha, vector<vector<double>> mu, int t_s, bool include_alpha)
{
    int k;
    for (Arc &arc : arcs)
    {
        arc.initialise_reduced_cost(nt);
        if (include_alpha && arc.get_type() == SERVICE)
        {
            k = 1;
            for (double a : alpha[arc.get_service() - 1])
            {
                arc.update_reduced_cost(k, -a);
                k++;
            }
        }
        for (pair<int, int> n : arc.get_iNodes())
        {
            arc.update_reduced_cost(-mu[n.first - 1][n.second / t_s]);
        }
    }
    return 1;
}

int pricing_algorithm(vector<Path> &paths, GRBModel &master, GRBLinExpr &obj, vector<GRBConstr> &path_constraints, vector<Train> trains, vector<Node> nodes, vector<Arc> arcs, vector<vector<bool>> s_assigned, vector<vector<double>> alpha, vector<double> pi, int nn, int T, int t_s, int ns, vector<vector<int>> forbidden_arcs, vector<double> *best_rcosts, int k_paths)
{
    int np = paths.size();
    int nt = trains.size();

    Path path;
    int s_id;
    int c_id;
    auto t1 = high_resolution_clock ::now();
    auto t2 = t1;

    GRBColumn column;
    for (int k = 1; k <= nt; k++)
    {

        vector<int> forbidden_arcs_k = forbidden_arcs[k-1];

        // TODO mettre à jour arcs avant algo de plus court chemins
        // WARNING : Horrible complexité - Deep copy de arcs puis clear
        // NO IT DOES'NT WORK, BECAUSE THE SHORTEST PATH ACCESS ARCS BY THEIR INDEX, AND THIS WOULD SHIFT ALL ARCS!
        /*vector<Arc> new_arcs;
        for (Arc arc : arcs)
        {
            if (count(forbidden_arcs_k.begin(), forbidden_arcs_k.end(), arc.get_ID()) == 0)
                new_arcs.push_back(arc);
        }*/
        

        vector<RcspResult> sps = shortest_path_algorithm_rcsp(nodes, arcs, nn, trains[k - 1], T, t_s, alpha, forbidden_arcs_k, k_paths);
        double best_rcost = 0; // WARNING - changed from INF to 0 for better logging info

        for (RcspResult &sp : sps)
        {
            //cerr << "Train : " << k << " cost : " << sp.cost << endl;
            if (sp.cost >= INF) continue;

            double rcost = sp.cost - pi[k - 1];
            best_rcost = min(best_rcost, rcost);

            if (rcost < -0.001 && !sp.arcs.empty())
            {
                // WARNING CHECKING DUPLICATES - TO BE INVESTIGATED
                if (path_already_exists(paths, sp.arcs, k))
                    continue;

                //cerr << "Reduced cost for train " + to_string(k) << " : " << rcost << endl;
                t2 = high_resolution_clock::now();
                //cout << "Shortest path duration: " << duration_cast<seconds>(t2 - t1).count() << " seconds." << endl;
                t1 = t2;

                column = GRBColumn();
                // Build path and path variable
                path = Path(np + 1, arcs, sp.arcs, k, ns);
                // path.print();
                t2 = high_resolution_clock::now();
                //cout << "Build path: " << duration_cast<seconds>(t2 - t1).count() << " seconds." << endl;
                t1 = t2;

                // Add to train flow constraint
                c_id = get_constraint_id(k, nt, ns);
                column.addTerm(1.0, path_constraints[c_id]);

                // Add to service constraints
                s_id = 1;
                for (bool s : s_assigned[k - 1])
                {
                    if (s)
                    {
                        if (path.get_service(s_id))
                        {
                            c_id = get_constraint_id(k, s_id, nt, ns);
                            column.addTerm(1.0, path_constraints[c_id]);
                        }
                    }
                    s_id++;
                }

                // Add to incompatibility constraints
                for (int a_id : path.get_arcs())
                {
                    for (pair<int, int> n : arcs[a_id].get_iNodes())
                    {
                        c_id = get_constraint_id(n.first, n.second, nt, ns, nn, T, t_s);
                        column.addTerm(1.0, path_constraints[c_id]);
                    }
                }

                t2 = high_resolution_clock::now();
                //cout << "Add column to constraints: " << duration_cast<seconds>(t2 - t1).count() << " seconds." << endl;

                np++;
                path.build_GRBVar(master, obj, column);
                paths.push_back(path);
            }
        }
        if (best_rcosts != nullptr)
        {
            (*best_rcosts)[k - 1] = (best_rcost < INF) ? best_rcost : INF;
        }
    }

    return np;
}

vector<RcspResult> shortest_path_algorithm_rcsp(vector<Node> nodes, vector<Arc> arcs, int nn, Train train, int T, int t_s, vector<vector<double>> alpha, vector<int> forbidden_arcs, int k_paths)
{
    int N = static_cast<int>(nodes.size());
    int k = train.get_ID();
    int ns = static_cast<int>(alpha.size());

    vector<int> service_bit(ns, -1);
    vector<double> alpha_bit;
    alpha_bit.reserve(ns);
    for (int s = 1; s <= ns; s++)
    {
        if (train.get_service(s))
        {
            service_bit[s - 1] = static_cast<int>(alpha_bit.size());
            alpha_bit.push_back(alpha[s - 1][k - 1]);
        }
    }

    int mask_count = 1 << static_cast<int>(alpha_bit.size());
    int total_states = N * mask_count;
    
    vector<vector<double>> dist(total_states, vector<double>(k_paths, INF));
    vector<vector<int>> prev_state(total_states, vector<int>(k_paths, -1));
    vector<vector<int>> prev_arc(total_states, vector<int>(k_paths, -1));
    vector<vector<int>> prev_slot(total_states, vector<int>(k_paths, -1));

    auto state_index = [mask_count](int node_id, int mask) { return node_id * mask_count + mask; };

    dist[state_index(0, 0)][0] = 0.0;

    auto relax_from_node = [&](int node_id)
    {
        Node &node = nodes[node_id];
        for (int mask = 0; mask < mask_count; mask++)
        {
            for (int slot = 0; slot < k_paths; slot++)
            {
                double d0 = dist[state_index(node_id, mask)][slot];
                if (d0 >= INF)
                {
                    continue;
                }
                for (int a_id : node.getFromArcs())
                {
                    if (!train.get_arc(a_id))
                    {
                        continue;
                    }
                    if (count(forbidden_arcs.begin(), forbidden_arcs.end(), a_id) != 0)
                    {
                        continue;
                    }    
                    int n2 = arcs[a_id].get_to();
                    int new_mask = mask;
                    double d = d0 + arcs[a_id].get_r_cost(k);
                    if (arcs[a_id].get_type() == SERVICE)
                    {
                        int s_id = arcs[a_id].get_service();
                        int bit = service_bit[s_id - 1];
                        if (bit >= 0 && (mask & (1 << bit)) == 0)
                        {
                            d -= alpha_bit[bit];
                            new_mask = mask | (1 << bit);
                        }
                    }
                    int next_state = state_index(n2, new_mask);
                    
                    // Chercher le slot le plus mauvais de l'état suivant
                    int worst_slot = -1;
                    double worst_val = d;
                    for (int s = 0; s < k_paths; s++)
                    {
                        if (dist[next_state][s] > worst_val)
                        {
                            worst_val = dist[next_state][s];
                            worst_slot = s;
                        }
                    }

                    // Si d améliore un des k slots, on met à jour
                    if (worst_slot >= 0)
                    {
                        dist[next_state][worst_slot] = d;
                        prev_state[next_state][worst_slot] = state_index(node_id, mask);
                        prev_arc[next_state][worst_slot] = a_id;
                        prev_slot[next_state][worst_slot] = slot; // on mémorise depuis quel slot on vient
                    }
                }
            }
        }
    };

    // Initialize from origin
    relax_from_node(0);

    for (int t = 0; t <= T; t += t_s)
    {
        for (int p = 1; p <= nn; p++)
        {
            for (int l : {0, 1, 2})
            {
                int n1 = n_id(p, t, l, nn, t_s);
                relax_from_node(n1);
            }
        }
    }

    // Collecter tous les (coût, état, slot) au nœud destination
    vector<tuple<double, int, int>> candidates;
    for (int mask = 0; mask < mask_count; mask++)
    {
        int s = state_index(1, mask);
        for (int slot = 0; slot < k_paths; slot++)
        {
            if (dist[s][slot] < INF)
                candidates.push_back(make_tuple(dist[s][slot], s, slot));
        }
    }

    // Trier par coût croissant et garder les k meilleurs
    sort(candidates.begin(), candidates.end());
    if ((int)candidates.size() > k_paths)
        candidates.resize(k_paths);

    vector<RcspResult> results;
    // Si aucun candidat trouvé, retourner un résultat vide
    if (candidates.empty())
    {
        results.push_back({INF, {}});
        return results;
    }

    for (auto &c : candidates)
    {
        double cost  = get<0>(c);
        int state    = get<1>(c);
        int slot     = get<2>(c);

        RcspResult result;
        result.cost = cost;

        int cur = state;
        int cur_slot = slot;
        while (cur != state_index(0, 0))
        {
            int a_id = prev_arc[cur][cur_slot];
            if (a_id < 0) break;
            result.arcs.push_back(a_id);
            int next = prev_state[cur][cur_slot];
            cur_slot = prev_slot[cur][cur_slot];
            cur = next;
            if (cur < 0) break;
        }
        results.push_back(result);
    }
    return results;
}

int get_constraint_id(int k, int nt, int ns)
{
    if (k > nt || k < 1)
    {
        cerr << "Invalid train ID" << endl;
        return 0;
    }
    return (k - 1) * (ns + 1);
}

int get_constraint_id(int k, int s, int nt, int ns)
{
    if (k > nt || k < 1)
    {
        cerr << "Invalid train ID" << endl;
        return 0;
    }
    if (s > ns || s < 1)
    {
        cerr << "Invalid service ID" << endl;
        return 0;
    }
    return (k - 1) * (ns + 1) + s;
}

int get_constraint_id(int p, int t, int nt, int ns, int nn, int T, int t_s)
{
    if (p > nn || p < 1)
    {
        cerr << "Invalid physical node ID" << endl;
        return 0;
    }
    if (t > T || t < 0 || t % t_s != 0)
    {
        cerr << "Invalid time coordinate" << endl;
        return 0;
    }
    int maxTimeStep = T / t_s + 1;
    int timeStep = t / t_s;
    return nt + nt * ns + (p - 1) * maxTimeStep + timeStep;
}

void solve_binary_model(GRBModel &master, vector<Path> &paths)
{
    for (Path &path : paths)
    {
        path.get_lambda().set(GRB_CharAttr_VType, GRB_BINARY);
    }
    master.update();
    master.optimize();

    cout << "---- Binary solution ----" << endl;
    for (Path &path : paths)
    {
        if (path.get_lambda().get(GRB_DoubleAttr_X) > 0.0)
        {
            cout << "Lambda value: " << path.get_lambda().get(GRB_DoubleAttr_X) << endl;
            cout << "Path ID: " << path.get_id() << endl;
            cout << "---------" << endl;
            // path.print();
        }
    }
}
