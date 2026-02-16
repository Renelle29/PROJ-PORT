#include "path_builder.h"

int build_train_apaths(vector<Train> &trains, vector<Path> paths)
{
    for (Train &train : trains)
    {
        train.clear_Apath();
    }

    for (Path path : paths)
    {
        if (path.get_lambda().get(GRB_DoubleAttr_X) >= 0.999)
        {
            trains[path.get_train() - 1].set_Apath(path.get_arcs());
        }
        else if (path.get_lambda().get(GRB_DoubleAttr_X) > 0.001)
        {
            return path.get_id();
        }
    }
    return 0;
}

int build_train_npaths(vector<Train> &trains, vector<Arc> arcs)
{
    vector<int> from;
    vector<int> to;
    int path_size;
    for (Train &train : trains)
    {
        train.clear_Npath();
        from = {};
        to = {};
        path_size = 0;

        for (int a_id : train.get_Apath())
        {
            from.push_back(arcs[a_id].get_from());
            to.push_back(arcs[a_id].get_to());
            path_size++;
        }
        sort(from.begin(), from.end());
        sort(to.begin(), to.end());

        if (from[0] != 0 || to[0] != 1) // Check if path starts in o and ends in d
        {
            cout << "Incorrect path for train " << train.get_ID() << ": starts or ends on the wrong node" << endl;
            cout << "From: " << from[0] << endl;
            cout << "To: " << to[0] << endl;
            return 1;
        }

        for (int i = 1; i < path_size; i++) // checks if each node is entered once and exited once
        {
            if (from[i] != to[i])
            {
                cout << "Incorrect path for train " << train.get_ID() << ": flow constraint not  respected for node " << from[i] << " or node " << to[i] << endl;
                return 1;
            }
        }

        from.erase(from.begin());
        train.set_Npath(from);
    }
    return 0;
}

int check_services(vector<Train> &trains, vector<Service> services)
{
    for (Train &train : trains) // for each train...
    {
        train.clear_p_service();
    }
    vector<int> sArcs;
    for (Service service : services) // For each service...
    {
        sArcs = service.getArcs();
        for (Train &train : trains) // for each train...
        {
            for (int a_id : train.get_Apath()) // for each arc taken by this train...
            {
                if (binary_search(sArcs.begin(), sArcs.end(), a_id)) // if it accomplishes this service...
                {
                    train.set_p_service(service.getID()); // increment its performed cout.
                }
            }
        }
    }
    return 1;
}

int print_path(Train train, vector<Node> nodes, vector<P_Node> pNodes, vector<Service> services)
{
    vector<int> p_nodes;
    vector<int> time_stamps;
    int ns = services.size();
    Node n;

    for (int i : train.get_Npath())
    {
        n = nodes[i];
        p_nodes.push_back(pNodes[n.getPNode() - 1].getID());
        time_stamps.push_back(n.getTime());
    }
    cout << "Train " << train.get_name() << ":" << endl;

    cout << "Visited nodes: " << endl;
    for (int id : p_nodes)
    {
        cout << id << ", ";
    }
    cout << endl;
    cout << "Visited time stamps: " << endl;
    for (int t : time_stamps)
    {
        cout << t << ", ";
    }
    cout << endl;
    cout << "Services: " << endl;
    for (int s = 1; s <= ns; s++)
    {
        if (train.get_service(s))
        {
            cout << services[s - 1].getName() << " assigned and performed " << train.get_p_service(s) << " times." << endl;
        }
    }
    cout << "--------------------" << endl;

    return 0;
}
