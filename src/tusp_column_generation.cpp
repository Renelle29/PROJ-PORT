#include <iostream>
#include <string>
#include <string.h>
#include <chrono>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cmath>
#include <sys/stat.h>
#include <algorithm>

// Our includes
#include "input_reader.h"
#include "P_Node.h"
#include "P_Link.h"
#include "Logger.h"
#include "Service.h"
#include "Train.h"
#include "Node.h"
#include "Arc.h"
#include "network_builder.h"
#include "path_builder.h"
#include "gurobi_c++.h"
#include "Arc-path_formulation.h"
#include "DWResults.h"

using namespace std;
using namespace std::chrono;

int best_objective = 1e9;
vector<Path> best_paths = {};
vector<pair<double,Path>> best_extended_paths = {};
int nodes_explored = 0;

static DWResults dw(int nt, vector<Train> trains, vector<Arc> arcs, int nn, int T, int t_s, int ns, int max_iters, vector<Node> nodes, int k_paths, int time_budget, vector<vector<int>> forbidden_arcs);

static pair<vector<int>, vector<int>> partition_arcs(vector<Arc> arcs, vector<Node> nodes, Path path1, Path path2)
{
    pair<vector<int>, vector<int>> partition;    
    vector<int> arc_ids1 = path1.get_arcs();
    reverse(arc_ids1.begin(), arc_ids1.end());
    vector<int> arc_ids2 = path2.get_arcs();
    for (int i : arc_ids1)
    {
        if (count(arc_ids2.begin(), arc_ids2.end(), i) == 0)
        {
            cerr << "Path1 using arc : " << i << " but not Path2" << endl;
            int separation_id = arcs[i].get_from();
            Node node = nodes[separation_id];
            vector<int> fArcs = node.getFromArcs();

            // TODO improve separation of arcs
            partition.first = {i};
            fArcs.erase(std::remove(fArcs.begin(), fArcs.end(), i), fArcs.end());
            partition.second = fArcs;
            break;
        }
        cerr << "Path1 & Path2 using arc : " << i << endl;
    }

    return partition;
}

static int bandp(int nt, vector<Train> trains, vector<Arc> arcs, int nn, int T, int t_s, int ns, int max_iters, vector<Node> nodes, int k_paths, int time_budget, vector<vector<int>> forbidden_arcs, Logger& logger, int max_nodes=20, int depth=0)
{
    // Check node budget before doing anything
    if (nodes_explored >= max_nodes)
    {
        logger.log(INFO, "Node budget reached ! Nodes explored : " + to_string(nodes_explored));
        return best_objective;
    }

    logger.log(INFO, "Starting a new DW iteration, depth : " + to_string(depth));
    DWResults dwresults = dw(nt, trains, arcs, nn, T, t_s, ns, max_iters, nodes, k_paths, time_budget, forbidden_arcs);
    nodes_explored++;
    logger.log(INFO, "Explored : " + to_string(nodes_explored) + " nodes.");

    logger.log(INFO, "Found a continuous objective of : " + to_string(dwresults.continuous_obj));
    // Pruning
    if (dwresults.continuous_obj >= best_objective)
    {
        logger.log(INFO, "Pruning ! Best objective : " + to_string(best_objective));
        return dwresults.continuous_obj;
    }

    // Branching - NAIVE FOR NOW - Think about other branching ideas

    int fractional_train = -1;
    Path path1;
    Path path2;

    // Trouver train fractionnel et deux chemins fractionels
    for (auto [lambda, path] : dwresults.extended_paths)
    {

        if (lambda > 1e-9 && lambda < (1 - 1e-9))
            if (fractional_train == -1)
            {
                fractional_train = path.get_train();
                logger.log(INFO, "Found a train with fractional lambdas : " + to_string(fractional_train));
                path1 = path;
                logger.log(INFO, "Path 1 : " + to_string(path.get_id()) + " lambda : " + to_string(lambda));
            }
            else if (path.get_train() == fractional_train)
            {
                path2 = path;
                logger.log(INFO, "Path 2 : " + to_string(path.get_id()) + " lambda : " + to_string(lambda));
                break;
            }
    }

    cerr << "Fractional train : " << fractional_train << endl;
    // Si solution entière, sortir de la boucle
    if (fractional_train == -1)
    {
        logger.log(INFO, "Found an integer solution of value : " + to_string(dwresults.continuous_obj));
        if (dwresults.continuous_obj < best_objective)
        {
            best_objective = dwresults.continuous_obj;
            vector<Path> new_paths;

            for (auto [lambda, path] : dwresults.extended_paths)
                if (lambda > 1 - 1e-9)  // only keep paths with lambda = 1
                    new_paths.push_back(path);
            
            best_paths = new_paths;
            best_extended_paths = dwresults.extended_paths;
        }
        return dwresults.continuous_obj;
    }

    // Sinon trouver premier noeud de changement
    // Partitionner les arcs sortant de ce noeud, et utiliser chaque partition comme forbidden_nodes
    pair<vector<int>, vector<int>> partition = partition_arcs(arcs, nodes, path1, path2);

    // Can't forbid dummy paths (as they are used in initialisation)
    bool no_dummy1 = true;
    bool no_dummy2 = true;

    vector<vector<int>> forbidden_arcs1 = forbidden_arcs;
    for (int arc_id : partition.first)
    {
        if (arcs[arc_id].get_type() == DUMMY)
            no_dummy1 = false;
        logger.log(INFO, "Arc : " + to_string(arc_id) + " for train : " + to_string(fractional_train) + " forbidden for left node");
        forbidden_arcs1[fractional_train-1].push_back(arc_id);
    }

    vector<vector<int>> forbidden_arcs2 = forbidden_arcs;
    for (int arc_id : partition.second)
    {
        if (arcs[arc_id].get_type() == DUMMY)
            no_dummy2 = false;
        logger.log(INFO, "Arc : " + to_string(arc_id) + " for train : " + to_string(fractional_train) + " forbidden for right node");
        forbidden_arcs2[fractional_train-1].push_back(arc_id);
    }

    // Appels récursifs
    depth++;
    int best_obj1 = 1e9;
    int best_obj2 = 1e9;
        
    if (no_dummy1)
        best_obj1 = bandp(nt, trains, arcs, nn, T, t_s, ns, max_iters, nodes, k_paths, time_budget, forbidden_arcs1, logger, max_nodes, depth);
    if (no_dummy2)
        best_obj2 = bandp(nt, trains, arcs, nn, T, t_s, ns, max_iters, nodes, k_paths, time_budget, forbidden_arcs2, logger, max_nodes, depth);

    if (best_obj1 < best_obj2)
        return best_obj1;

    return best_obj2;
}

static DWResults dw(int nt, vector<Train> trains, vector<Arc> arcs, int nn, int T, int t_s, int ns, int max_iters, vector<Node> nodes, int k_paths, int time_budget, vector<vector<int>> forbidden_arcs)
{
    DWResults results;

    //debuglogger.log(DEBUG, "Call Gurobi solver");
    try
    {
        // Gurobi environment (single shared env for master and pricing).
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "mip1.log"); // Gurobi log in working directory.
        env.set("LogToConsole", "0");
        env.start();

        // Master problem (restricted) for column generation.
        //logger.log(INFO, "Solving arc-path model");
        //debuglogger.log(DEBUG, "--- Build Arc-path problem ---");
        GRBModel master = GRBModel(env);
        master.set("JSONSolDetail", "1");
        GRBLinExpr Master_obj = 0;

        vector<Path> paths;
        vector<vector<bool>> s_assigned(nt);
        vector<GRBConstr> path_constraints;

        //debuglogger.log(DEBUG, "Initialize master problem");
        int np0 = initialize_master_AP(master, Master_obj, paths, s_assigned, path_constraints, trains, arcs, nn, T, t_s, ns);

        /*cerr << "Initialized!" << endl;
        for (Path path : paths)
        {
            cerr << "Path for train : " << path.get_train() << endl;
            for (int i : path.get_arcs())
            {
                cerr << "Containing arc : " << i << endl;
            }
        }*/

        int np = paths.size();
        int iter = 0;
        auto start_cg = high_resolution_clock::now();
        string stop_reason;

        cerr << "Starting column generation (arc-path formulation)..." << endl;
        // 4) Column generation loop:
        //    - Solve the restricted master problem (RMP) with the current path set.
        //    - Extract dual multipliers for flow, service, and incompatibility constraints.
        //    - Build reduced costs on the arc network.
        //    - Pricing: per-train RCSP shortest path (pricing_algorithm)
        //    - Add every negative reduced-cost path found to the RMP.
        //    - Stop when:
        //        * time budget reached
        //        * no new column added
        //        * max_iters reached
        while (iter < max_iters)
        {
            auto iter_start = high_resolution_clock::now();
            master.setObjective(Master_obj, GRB_MINIMIZE);
            master.optimize();

            if (master.get(GRB_IntAttr_Status) != 2)
            {
                cerr << "Reduced master non optimal" << endl;
                break;
            }

            // Debug: report dummy path usage per train in the current RMP solution.
            vector<double> dummy_lambda(nt, 0.0);
            for (const Path &path : paths)
            {
                const vector<int> arc_ids = path.get_arcs();
                bool has_dummy = false;
                for (int a_id : arc_ids)
                {
                    if (arcs[a_id].get_type() == DUMMY)
                    {
                        has_dummy = true;
                        break;
                    }
                }
                if (has_dummy)
                {
                    int k = path.get_train();
                    if (k >= 1 && k <= nt)
                    {
                        dummy_lambda[k - 1] += path.get_lambda().get(GRB_DoubleAttr_X);
                    }
                }
            }

            // Duals from RMP.
            vector<double> pi;
            vector<vector<double>> alpha(ns);
            vector<vector<double>> mu(nn);
            build_dual_vectors(pi, alpha, mu, path_constraints, nt, s_assigned, nn, T, t_s);
            compute_reduced_cost_network(arcs, nt, alpha, mu, t_s, false);

            int before = np;
            vector<double> best_rcosts(trains.size(), numeric_limits<double>::quiet_NaN());
            // Per-train shortest path pricing.
            np = pricing_algorithm(paths, master, Master_obj, path_constraints, trains, nodes, arcs, s_assigned, alpha, pi, nn, T, t_s, ns, forbidden_arcs, &best_rcosts, k_paths);

            iter++;
            auto iter_seconds = duration_cast<duration<double>>(high_resolution_clock::now() - iter_start).count();
            auto elapsed = duration_cast<seconds>(high_resolution_clock::now() - start_cg).count();
            int new_paths = np - before;
            double rc_sum = 0.0;
            int rc_count = 0;
            for (double rc : best_rcosts)
            {
                if (!isnan(rc))
                {
                    rc_sum += rc;
                    rc_count++;
                }
            }

            cerr << "Iteration " << iter << " finished: new_paths=" << new_paths
                 << ", sum_best_reduced_costs=" << fixed << setprecision(6) << rc_sum
                 << " (sum of best per-train reduced costs over " << rc_count << " trains)" << endl;

            // Stop if time budget is reached or no new column was added.
            stop_reason.clear();
            if (elapsed >= time_budget)
            {
                stop_reason = "time_budget";
            }
            else if (new_paths == 0)
            {
                stop_reason = "no_new_columns";
            }
            else if (iter >= max_iters)
            {
                stop_reason = "max_iters";
            }

            if (!stop_reason.empty())
            {
                break;
            }
        }
        
        cerr << "Column generation finished after " << iter << " iterations and " << duration_cast<seconds>(high_resolution_clock::now() - start_cg).count() << " seconds." << endl;
        cerr << "Final solve of the restricted master problem solved with " << paths.size() << " columns." << endl;

        master.setObjective(Master_obj, GRB_MINIMIZE);
        master.update();
        master.optimize();
        double continuous_obj = Master_obj.getValue();
        cerr << "Obtained continuous objective value: " + to_string(continuous_obj) << endl;

        results.continuous_obj = continuous_obj;
        
        vector<pair<double,Path>> extended_paths;
        for (Path path : paths)
        {
            extended_paths.push_back(make_pair(path.get_lambda().get(GRB_DoubleAttr_X), path));
        }
        results.extended_paths = extended_paths;
        
        results.np0 = np0;
    }
    catch (...){
        cerr << "On a été dans le catch, c'est pas cool, j'pense y a un problème :'( bonne chance thierry & bruno !" << endl;
    }

    return results;
}

// Simple CLI helper: detect numeric strings (optional sign, optional decimal).
static bool is_number(const string &s)
{
    if (s.empty())
    {
        return false;
    }
    size_t i = 0;
    if (s[0] == '+' || s[0] == '-')
    {
        i = 1;
    }
    if (i >= s.size())
    {
        return false;
    }
    bool has_digit = false;
    bool has_dot = false;
    for (; i < s.size(); i++)
    {
        if (!isdigit(static_cast<unsigned char>(s[i])))
        {
            if (s[i] == '.' && !has_dot)
            {
                has_dot = true;
                continue;
            }
            return false;
        }
        has_digit = true;
    }
    return has_digit;
}

// Sanitize a label so it can be safely used in filenames.
static string sanitize_label(const string &s)
{
    string out;
    for (char c : s)
    {
        if (isalnum(static_cast<unsigned char>(c)))
        {
            out.push_back(c);
        }
        else
        {
            out.push_back('_');
        }
    }
    if (out.empty())
    {
        out = "unknown";
    }
    return out;
}

static void ensure_dir(const string &path)
{
    mkdir(path.c_str(), 0755);
}

int main(int argc, char *argv[])
{
    // CLI:
    //   tusp_solver "<station name>" <scenario_number> [time_budget_cg_seconds] [max_iters] [time_budget_binary_ilp_seconds] [k_paths]
    if (argc < 3)
    {
        cerr << "Usage: tusp_solver \"<station name>\" <scenario_number> [time_budget_cg_seconds] [max_iters] [time_budget_binary_ilp_seconds]" << endl;
        return 1;
    }

    bool verbose = true;
    string station = argv[1];
    string scenario = argv[2];

    // Optional parameters with defaults.
    int time_budget = 7200; // column generation time budget
    int max_iters = 1000;
    double time_budget_binary = 0.0; // 0 means no time limit for final binary master (integer solve)
    int k_paths = 1;

    vector<double> numeric_args;
    for (int i = 3; i < argc; i++)
    {
        if (is_number(argv[i]))
        {
            numeric_args.push_back(stod(argv[i]));
        }
    }
    if (numeric_args.size() > 0)
    {
        time_budget = static_cast<int>(numeric_args[0]);
    }
    if (numeric_args.size() > 1)
    {
        max_iters = static_cast<int>(numeric_args[1]);
    }
    if (numeric_args.size() > 2)
    {
        time_budget_binary = numeric_args[2];
    }
    if (numeric_args.size() > 3)
    {
        k_paths = static_cast<int>(numeric_args[3]);
    }

    // Runtime logs are written to files in the working directory.
    // Output files for summary + per-iteration pricing diagnostics.
    string safe_station = sanitize_label(station);
    string safe_scenario = sanitize_label(scenario);
    string summary_path = "results_" + safe_station + "_scenario_" + safe_scenario + ".txt";
    string iter_pricing_path = "iterations_pricing.csv";
    string iter_pricing_trains_path = "iterations_pricing_reduced_costs.csv";

    string viz_root = "solutions_log";
    string viz_dir = viz_root + "/" + safe_station + "_scenario_" + safe_scenario;
    ensure_dir(viz_root);
    ensure_dir(viz_dir);

    string logfile_path = viz_dir + "/logfile.log";
    string output_path = viz_dir + "/output.txt";

    // Output locations (relative to the working directory, typically repo root):
    // - results_<station>_scenario_<scenario>.txt (summary of the run)
    // - iterations_pricing.csv (per-iteration summary)
    // - iterations_pricing_reduced_costs.csv (per-iteration reduced costs by train)
    // - solutions_log/<station>_scenario_<scenario>/logfile.log (logger output for visualization)
    // - solutions_log/<station>_scenario_<scenario>/output.txt (stdout used by visualization)
    Logger logger(logfile_path);
    Logger debuglogger(logfile_path);
    logger.log(INFO, "------- Begin new arc-path run -------");
    debuglogger.log(DEBUG, "------ Begin new arc-path run ------");

    // Redirect stdout to solutions_log/<station>_scenario_<scenario>/output.txt (for visualization).
    freopen(output_path.c_str(), "w", stdout);

    vector<Service> services;
    vector<Node_Type> node_types;
    vector<P_Node> pNodes;
    vector<P_Link> plinks;
    vector<Train_Type> train_types;
    vector<Train> trains;
    vector<Node> nodes;
    vector<Arc> arcs;

    // Build file paths for the selected station + scenario.
    string base = "instances/" + station;
    string scen = base + "/scenario " + scenario;

    string service_path = scen + "/Service input.csv";
    string node_types_path = base + "/Node types.csv";
    string node_path = base + "/Node input.csv";
    string link_path = base + "/Link input.csv";
    string train_types_path = scen + "/Train types.csv";
    string train_path = scen + "/Train input.csv";
    string ops_path = scen + "/Operations assignment.csv";

    // Time horizon and time-step are inferred from inputs.
    int T = 1;   // Time frame
    int t_s = 0; // Minimum time step

    // 1) Read inputs.
    debuglogger.log(DEBUG, "Read service input.");
    int ns = read_service_input(service_path, services);
    if (ns == 0)
    {
        logger.log(ERROR, "Input file issue.");
        return 1;
    }

    debuglogger.log(DEBUG, "Read node type input.");
    int nnt = read_node_type_input(node_types_path, node_types, ns);
    if (nnt == 0)
    {
        logger.log(ERROR, "Input file issue.");
        return 1;
    }

    debuglogger.log(DEBUG, "Read node input.");
    int nn = read_node_input(node_path, pNodes, node_types, nnt);
    if (nn == 0)
    {
        logger.log(ERROR, "Input file issue.");
        return 1;
    }

    debuglogger.log(DEBUG, "Read link input.");
    int nl = read_link_input(link_path, plinks, pNodes, nn);
    if (nl == 0)
    {
        logger.log(ERROR, "Input file issue.");
        return 1;
    }

    debuglogger.log(DEBUG, "Read train type input.");
    int ntt = read_train_type(train_types_path, train_types);
    if (ntt == 0)
    {
        logger.log(ERROR, "Input file issue.");
        return 1;
    }

    debuglogger.log(DEBUG, "Read train input.");
    int nt = read_train_input(train_path, trains, ntt, ns, T);
    if (nt == 0)
    {
        logger.log(ERROR, "Input file issue.");
        return 1;
    }

    debuglogger.log(DEBUG, "Read operations input.");
    vector<int> assignment = read_operation_input(ops_path, trains, nt, ns);
    int no = 0;
    for (unsigned int a : assignment)
    {
        no += a;
    }

    // 2) Compute time step and build the time-space network.
    t_s = compute_time_step(trains, services, train_types, plinks);
    logger.log(INFO, "Time step is worth: " + to_string(t_s));
    logger.log(INFO, "Time horizon is worth: " + to_string(T));

    debuglogger.log(DEBUG, "Build Time-space network nodes");
    int N = build_nodes(nodes, nn, T, t_s);
    if (N != nn * (T / t_s + 1) * 3 + 1)
    {
        logger.log(ERROR, "Building issue, wrong number of nodes, expected " + to_string(nn * T * 3) + ", got: " + to_string(N));
        return 1;
    }

    debuglogger.log(DEBUG, "Build all arcs");
    build_arcs(arcs, trains, nodes, services, train_types, plinks, node_types, nn, nt, ns, T, t_s);
    for (Train train : trains)
    {
        if (arcs.size() != train.get_bools().size())
        {
            logger.log(ERROR, "Boolean arc train construct is not the right size");
            return 1;
        }
    }
    debuglogger.log(DEBUG, "All arcs built");

    if (verbose){
        /*for (Node node : nodes){
            logger.log(INFO, "Built node : ID - " + to_string(node.getPNode()) + " TIME - " + to_string(node.getTime()));
        }*/
        /*for (Arc arc : arcs){
            logger.log(INFO, "Built arc : FROM - " + to_string(arc.get_from()) + " TO - " + to_string(arc.get_to()) + " SERVICE - " + to_string(arc.get_service()) + " TYPE - " + to_string(arc.get_type()));
        }*/
        for (Train train : trains){
            logger.log(INFO, "Train : ID - " + to_string(train.get_ID()) + " NAME - " + train.get_name() + " TYPE - " + to_string(train.get_type()) + " ARRIVAL - " + to_string(train.get_a_t()) + " DEPARTURE - " + to_string(train.get_d_t()));
        }
    }

    cerr << "Constructed three-layer time-space network (nodes + arcs)." << endl;

    // Prepare pricing report files (headers depend on train count).
    ofstream iter_pricing(iter_pricing_path);
    ofstream iter_pricing_trains(iter_pricing_trains_path);
    if (iter_pricing.is_open())
    {
        iter_pricing << "iter,new_paths,iter_seconds,stop_reason" << endl;
    }
    if (iter_pricing_trains.is_open())
    {
        iter_pricing_trains << "iter";
        for (const Train &train : trains)
        {
            iter_pricing_trains << ",train_" << train.get_ID();
        }
        iter_pricing_trains << endl;
    }
    
    vector<vector<int>> forbidden_arcs;
    for (const Train &train : trains)
    {
        forbidden_arcs.push_back({});
    }

    logger.log(INFO, "Starting Branch & Price");
    bandp(nt, trains, arcs, nn, T, t_s, ns, max_iters, nodes, k_paths, time_budget, forbidden_arcs, logger);

    cerr << "Obtained continuous objective value: " + to_string(best_objective) << endl;
    for (Path& path : best_paths)
    {
        cerr << "Path : " << path.get_id() << " used by train : " << path.get_train() << endl;
    }

    build_train_apaths(trains, best_extended_paths);
    build_train_npaths(trains, arcs);
    check_services(trains, services);

    //DWResults dwresults = dw(nt, trains, arcs, nn, T, t_s, ns, max_iters, nodes, k_paths, time_budget, forbidden_arcs);
    return 0;
   
//// START  
// 3) Solve with column generation (arc-path formulation).
    debuglogger.log(DEBUG, "Call Gurobi solver");
    try
    {
        // Gurobi environment (single shared env for master and pricing).
        GRBEnv env = GRBEnv(true);
        env.set("LogFile", "mip1.log"); // Gurobi log in working directory.
        env.set("LogToConsole", "0");
        env.start();

        // Master problem (restricted) for column generation.
        logger.log(INFO, "Solving arc-path model");
        debuglogger.log(DEBUG, "--- Build Arc-path problem ---");
        GRBModel master = GRBModel(env);
        master.set("JSONSolDetail", "1");
        GRBLinExpr Master_obj = 0;

        vector<Path> paths;
        vector<vector<bool>> s_assigned(nt);
        vector<GRBConstr> path_constraints;

        debuglogger.log(DEBUG, "Initialize master problem");
        int np0 = initialize_master_AP(master, Master_obj, paths, s_assigned, path_constraints, trains, arcs, nn, T, t_s, ns);

        int np = paths.size();
        int iter = 0;
        auto start_cg = high_resolution_clock::now();
        string stop_reason;

        cerr << "Starting column generation (arc-path formulation)..." << endl;
        // 4) Column generation loop:
        //    - Solve the restricted master problem (RMP) with the current path set.
        //    - Extract dual multipliers for flow, service, and incompatibility constraints.
        //    - Build reduced costs on the arc network.
        //    - Pricing: per-train RCSP shortest path (pricing_algorithm)
        //    - Add every negative reduced-cost path found to the RMP.
        //    - Stop when:
        //        * time budget reached
        //        * no new column added
        //        * max_iters reached
        while (iter < max_iters)
        {
            auto iter_start = high_resolution_clock::now();
            master.setObjective(Master_obj, GRB_MINIMIZE);
            master.optimize();

            if (master.get(GRB_IntAttr_Status) != 2)
            {
                cerr << "Reduced master non optimal" << endl;
                break;
            }

            // Debug: report dummy path usage per train in the current RMP solution.
            vector<double> dummy_lambda(nt, 0.0);
            for (const Path &path : paths)
            {
                const vector<int> arc_ids = path.get_arcs();
                bool has_dummy = false;
                for (int a_id : arc_ids)
                {
                    if (arcs[a_id].get_type() == DUMMY)
                    {
                        has_dummy = true;
                        break;
                    }
                }
                if (has_dummy)
                {
                    int k = path.get_train();
                    if (k >= 1 && k <= nt)
                    {
                        dummy_lambda[k - 1] += path.get_lambda().get(GRB_DoubleAttr_X);
                    }
                }
            }

            // Duals from RMP.
            vector<double> pi;
            vector<vector<double>> alpha(ns);
            vector<vector<double>> mu(nn);
            build_dual_vectors(pi, alpha, mu, path_constraints, nt, s_assigned, nn, T, t_s);
            compute_reduced_cost_network(arcs, nt, alpha, mu, t_s, false);

            int before = np;
            vector<double> best_rcosts(trains.size(), numeric_limits<double>::quiet_NaN());
            // Per-train shortest path pricing.
            //np = pricing_algorithm(paths, master, Master_obj, path_constraints, trains, nodes, arcs, s_assigned, alpha, pi, nn, T, t_s, ns, &best_rcosts, k_paths);

            iter++;
            auto iter_seconds = duration_cast<duration<double>>(high_resolution_clock::now() - iter_start).count();
            auto elapsed = duration_cast<seconds>(high_resolution_clock::now() - start_cg).count();
            int new_paths = np - before;
            double rc_sum = 0.0;
            int rc_count = 0;
            for (double rc : best_rcosts)
            {
                if (!isnan(rc))
                {
                    rc_sum += rc;
                    rc_count++;
                }
            }

            cerr << "Iteration " << iter << " finished: new_paths=" << new_paths
                 << ", sum_best_reduced_costs=" << fixed << setprecision(6) << rc_sum
                 << " (sum of best per-train reduced costs over " << rc_count << " trains)" << endl;

            if (iter_pricing_trains.is_open())
            {
                iter_pricing_trains << iter;
                iter_pricing_trains << fixed << setprecision(6);
                for (double rc : best_rcosts)
                {
                    iter_pricing_trains << "," << rc;
                }
                iter_pricing_trains << endl;
            }

            // Stop if time budget is reached or no new column was added.
            stop_reason.clear();
            if (elapsed >= time_budget)
            {
                stop_reason = "time_budget";
            }
            else if (new_paths == 0)
            {
                stop_reason = "no_new_columns";
            }
            else if (iter >= max_iters)
            {
                stop_reason = "max_iters";
            }

            if (iter_pricing.is_open())
            {
                iter_pricing << iter << "," << new_paths << "," << fixed << setprecision(6) << iter_seconds << "," << stop_reason << endl;
            }

            if (!stop_reason.empty())
            {
                break;
            }
        }


        cerr << "Column generation finished after " << iter << " iterations and " << duration_cast<seconds>(high_resolution_clock::now() - start_cg).count() << " seconds." << endl;

        cerr << "Final solve of the restricted master problem solved with " << paths.size() << " columns." << endl;

        // Final solve of the restricted master (LP) on the last column set.
        master.setObjective(Master_obj, GRB_MINIMIZE);
        master.update();
        master.optimize();
        double continuous_obj = Master_obj.getValue();
//// END
        //logger.log(INFO, "Obtained continuous objective value: " + to_string(dwresults.continuous_obj));
        //logger.log(INFO, to_string(dwresults.paths.size() - dwresults.np0 + 1) + " columns generated.");

        // Vérifier l'intégralité des lambdas
        bool is_integer = true;
        int nb_paths = 0;
        int zero_paths = 0;
        for (Path &path : paths) {
            nb_paths++;
            double lambda_val = path.get_lambda().get(GRB_DoubleAttr_X);
            if (lambda_val > 0.001) { // Afficher seulement les non-nulles
                logger.log(INFO, "Path " + to_string(path.get_id()) + 
                        " train " + to_string(path.get_train()) + 
                        " lambda = " + to_string(lambda_val));
                if (lambda_val > 0.001 && lambda_val < 0.999)
                    is_integer = false;
            }
            else{
                zero_paths++;
            }
        }
        logger.log(INFO, is_integer ? "Solution is integer!" : "Solution is fractional");
        logger.log(INFO, "There are " + to_string(zero_paths) + " unused paths over " + to_string(nb_paths) + " paths.");

        cerr << "Obtained continuous objective value: " + to_string(continuous_obj) << endl;

        cerr << "Gurobi solve of the binary master with the generated columns (it can take a while)..." << endl;

        // 5) Solve the binary master on the generated columns.
        //    This yields an integer solution optimal for the *restricted* column set.
        if (time_budget_binary > 0.0)
        {
            master.set(GRB_DoubleParam_TimeLimit, time_budget_binary);
            cerr << "Binary ILP time limit: " << time_budget_binary << " seconds." << endl;
        }
        debuglogger.log(DEBUG, "Solve binary model");
        solve_binary_model(master, paths);

        if (time_budget_binary > 0.0)
        {
            int status = master.get(GRB_IntAttr_Status);
            int solcount = master.get(GRB_IntAttr_SolCount);
            if (status == GRB_TIME_LIMIT && solcount == 0)
            {
                cerr << "Binary ILP: no feasible solution found within time limit ("
                     << time_budget_binary << " seconds)." << endl;
            }
            else if (status == GRB_TIME_LIMIT && solcount > 0)
            {
                cerr << "Binary ILP: time limit reached, using best feasible solution found." << endl;
            }
        }

        double binary_obj = Master_obj.getValue();
        logger.log(INFO, "Obtainted binary objective value: " + to_string(binary_obj));
        double mipGap = 1.0 - continuous_obj / binary_obj;
        logger.log(INFO, "Optimality gap: " + to_string(mipGap));
        logger.log(INFO, "After " + to_string(iter) + " iterations.");

        cerr << "Obtained binary objective value: " + to_string(binary_obj) << endl;
        cerr << "Optimality gap: " + to_string(mipGap) << endl;

        // 6) Reconstruct and print train paths.
        build_train_apaths_old(trains, paths);
        build_train_npaths(trains, arcs);
        check_services(trains, services);

        int accepted_trains = 0;
        int dummy_trains = 0;
        int total_assigned_services = 0;
        int assigned_services_completed = 0;
        for (const Train &train : trains)
        {
            if (train.get_Apath().size() > 1)
            {
                accepted_trains++;
            }
            else
            {
                dummy_trains++;
            }

            for (int s = 1; s <= ns; s++)
            {
                if (train.get_service(s))
                {
                    total_assigned_services++;
                    if (train.get_p_service(s) > 0)
                    {
                        assigned_services_completed++;
                    }
                }
            }
        }

        // Summary file (one per station+scenario), saved in working directory.
        ofstream summary(summary_path);
        if (summary.is_open())
        {
            summary << "=== Instance ===" << endl;
            summary << "station: " << station << endl;
            summary << "scenario: " << scenario << endl;
            summary << "planning_horizon_T: " << T << endl;
            summary << "time_step_t_s: " << t_s << endl;
            summary << "num_trains: " << nt << endl;
            summary << "num_services: " << ns << endl;
            summary << "total_assigned_services: " << total_assigned_services << endl;
            summary << "num_nodes: " << nn << endl;
            summary << "num_links: " << nl << endl;
            summary << endl;
            summary << "=== Solution ===" << endl;
            summary << "pricing_mode: per_train_shortest_path" << endl;
            summary << "time_budget_seconds: " << time_budget << endl;
            summary << "time_budget_binary_ilp_seconds: " << time_budget_binary << endl;
            summary << "max_iters: " << max_iters << endl;
            summary << "iterations: " << iter << endl;
            summary << "columns_generated: " << (paths.size() - np0 + 1) << endl;
            summary << "continuous_obj: " << continuous_obj << endl;
            summary << "binary_obj: " << binary_obj << endl;
            summary << "mip_gap: " << mipGap << endl;
            summary << "accepted_trains: " << accepted_trains << endl;
            summary << "dummy_trains: " << dummy_trains << endl;
            summary << "assigned_services_completed: " << assigned_services_completed << endl;
            summary << "stop_reason: " << (stop_reason.empty() ? "completed" : stop_reason) << endl;
        }

        cout << "--- arc-path ---" << endl;
        for (Train train : trains)
        {
            print_path(train, nodes, pNodes, services);
            if (train.get_Apath().size() <= 1)
            {
                logger.log(INFO, "Dummy path was taken by train " + to_string(train.get_ID()));
            }
        }
    }
    catch (const GRBException &e)
    {
        string Error = "Error code = " + to_string(e.getErrorCode());
        logger.log(ERROR, Error);
    }
    catch (...)
    {
        logger.log(ERROR, "Exception during optimization");
    }

    // 7) Summary statistics.
    logger.log(INFO, "Time period: " + to_string(T));
    logger.log(INFO, "Number of trains: " + to_string(nt));
    logger.log(INFO, "Station size: " + to_string(nn) + " nodes");
    logger.log(INFO, "Average node degree: " + to_string(2 / nn * nl));
    logger.log(INFO, "Number of operations assigned: " + to_string(no));
    (void)assignment;

    logger.log(INFO, "Program finished running");
    return 0;
}
