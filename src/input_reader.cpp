#include "input_reader.h"

vector<vector<string>> readCSV(const string &filename)
{
    vector<vector<string>> data;
    ifstream file(filename);

    if (!file.is_open())
    {
        cerr << "Failed to open file: " << filename << endl;
        return data;
    }

    string line;
    while (getline(file, line))
    {
        vector<string> row;
        stringstream ss(line);
        string cell;

        while (getline(ss, cell, ','))
        {
            row.push_back(cell);
        }

        data.push_back(row);
    }

    file.close();
    return data;
}

int read_service_input(const string &filename, vector<Service> &services)
{
    vector<vector<string>> data;
    data = readCSV(filename);
    int n = data.size();
    int id;

    if (n < 1 || data[0].size() != 4)
    {
        cerr << "Wrong input file format, empty file or wrong number of columns: " << filename << endl;
        cerr << "Number of columns: " << data[0].size() << endl;
        cerr << "Expected number of columns: 4" << endl;
        return 0;
    }

    for (int i = 1; i < n; i++)
    {
        id = stoi(data[i][0]);
        if (i != id)
        {
            cerr << "Invalid input file, service id invalid: " << filename << endl;
            cerr << "Service ID: " << i << ", Wrong given ID: " << id << endl;
            return 0;
        }
        services.push_back(Service(id, data[i][1], stoi(data[i][2]), stoi(data[i][3])));
    }

    return n - 1; // Return the number of services (excluding header)
}

int read_node_type_input(const string &filename, vector<Node_Type> &node_types, int nservices)
{
    vector<vector<string>> data;
    data = readCSV(filename);
    int n = data.size();
    int id; // stores node type id

    if (n < 1 || static_cast<int>(data[0].size()) != 5 + nservices)
    {
        cerr << "Wrong input file format, empty file or wrong number of columns based on given services: " << filename << endl;
        cerr << "Number of columns: " << data[0].size() << endl;
        cerr << "Expected number of columns: " << 5 + nservices << endl;
        return 0;
    }

    for (int i = 1; i < n; i++)
    {
        id = stoi(data[i][0]);
        if (i != id)
        {
            cerr << "Invalid input file, node type id invalid: " << filename << endl;
            cerr << "Node type ID: " << i << ", Wrong given ID: " << id << endl;
            return 0;
        }
        node_types.push_back(Node_Type(id, data[i][1], data[i][2] == "1", data[i][3] == "1", data[i][4] == "1", {}));
        for (int j = 0; j < nservices; j++)
        {
            node_types[i - 1].addService(stoi(data[i][5 + j]) == 1); // For some reason doesn't work for the last column when using =="1" so transforming to int first
        }
    }

    return n - 1; // Return the number of node types (excluding header)
}

int read_node_input(const string &filename, vector<P_Node> &pnodes, vector<Node_Type> &node_types, int ntypes)
{
    vector<vector<string>> data;
    data = readCSV(filename);
    int n = data.size();
    int type; // stores node type
    int id;

    if (n < 1 || data[0].size() != 3)
    {
        cerr << "Wrong input file format, empty file or wrong number of columns: " << filename << endl;
        cerr << "Number of columns: " << data[0].size() << endl;
        cerr << "Expected number of columns: 3" << endl;
        return 0;
    }

    for (int i = 1; i < n; i++)
    {
        id = stoi(data[i][0]);
        if (i != id)
        {
            cerr << "Invalid input file, node id invalid: " << filename << endl;
            cerr << "Node ID: " << i << ", Wrong given ID: " << id << endl;
            return 0;
        }
        type = stoi(data[i][2]);
        if (type <= 0 || type > ntypes)
        {
            cerr << "Invalid input file, node type invalid: " << filename << endl;
            cerr << "Node ID: " << id << ", Node type: " << type << endl;
            return 0;
        }
        pnodes.push_back(P_Node(id, data[i][1], type));
        node_types[type - 1].addNode(id);
    }
    return n - 1; // Return the number of nodes (excluding header)
}

int read_link_input(const string &filename, vector<P_Link> &links, vector<P_Node> &pnodes, int nnodes)
{
    vector<vector<string>> data;
    data = readCSV(filename);
    int n = data.size();
    int o;  // stores origin id
    int d;  // stores destination id
    int id; // stores link id

    if (n < 1 || data[0].size() != 4)
    {
        cerr << "Wrong input file format, empty file or wrong number of columns: " << filename << endl;
        cerr << "Number of columns: " << data[0].size() << endl;
        cerr << "Expected number of columns: 4" << endl;
        return 0;
    }

    for (int i = 1; i < n; i++)
    {
        id = stoi(data[i][0]);
        if (i != id)
        {
            cerr << "Invalid input file, link id invalid: " << filename << endl;
            cerr << "Link ID: " << i << ", Wrong given ID: " << id << endl;
            return 0;
        }
        o = stoi(data[i][1]);
        d = stoi(data[i][2]);
        links.push_back(P_Link(id, o, d, stoi(data[i][3])));

        if (o < 1 || o > nnodes || d < 1 || d > nnodes)
        {
            cerr << "Invalid input file, origin or destination nodes invalid: " << filename << endl;
            cerr << "Link ID: " << id << ", Origin: " << o << ", Destination: " << d << endl;
            cerr << "Number of nodes: " << nnodes << endl;
            return 0;
        }

    }
    return n - 1; // Return the number of links (excluding header)
}

int read_train_type(const string &filename, vector<Train_Type> &ttypes)
{
    vector<vector<string>> data;
    data = readCSV(filename);
    int n = data.size();
    int id; // stores train type id

    if (n < 1 || data[0].size() != 4)
    {
        cerr << "Wrong input file format, empty file or wrong number of columns: " << filename << endl;
        cerr << "Number of columns: " << data[0].size() << endl;
        cerr << "Expected number of columns: 4" << endl;
        return 0;
    }

    for (int i = 1; i < n; i++)
    {
        id = stoi(data[i][0]);
        if (i != id)
        {
            cerr << "Invalid input file, train type id invalid: " << filename << endl;
            cerr << "Train type ID: " << i << ", Wrong given ID: " << id << endl;
            return 0;
        }
        ttypes.push_back(Train_Type(id, data[i][1], stoi(data[i][2]), stoi(data[i][3])));
    }

    return n - 1; // Return the number of types (excluding header)
}

int read_train_input(const string &filename, vector<Train> &trains, int ntypes, int nservices, int &T)
{
    vector<vector<string>> data;
    data = readCSV(filename);
    int n = data.size();
    int type; // stores train type
    int id;   // stores train id
    int a;    // stores arrival time
    int d;    // stores departure time

    if (n < 1 || data[0].size() != 5)
    {
        cerr << "Wrong input file format, empty file or wrong number of columns: " << filename << endl;
        cerr << "Number of columns: " << data[0].size() << endl;
        cerr << "Expected number of columns: 5" << endl;
        return 0;
    }

    for (int i = 1; i < n; i++)
    {
        if (data[i].size() != 5)
        {
            cerr << "Wrong number of columns: " << filename << " line " << i << endl;
            cerr << "Number of columns: " << data[i].size() << endl;
            cerr << "Expected number of columns: 5" << endl;
        }
        id = stoi(data[i][0]);
        if (i != id)
        {
            cerr << "Invalid input file, train id invalid: " << filename << endl;
            cerr << "Train ID: " << i << ", Wrong given ID: " << id << endl;
            return 0;
        }
        type = stoi(data[i][2]);
        if (type <= 0 || type > ntypes)
        {
            cerr << "Invalid input file, train type invalid: " << filename << endl;
            cerr << "Train ID: " << id << ", Wrong given type: " << type << endl;
            cerr << "Number of types: " << ntypes << endl;
            return 0;
        }
        a = stoi(data[i][3]);
        d = stoi(data[i][4]);
        if (a < 0 || d < a)
        {
            cerr << "Invalid input file, arrival or departure time invalid: " << filename << endl;
            cerr << "Train ID: " << id << ", Arrival time: " << a << ", Departure time: " << d << endl;
            return 0;
        }
        trains.push_back(Train(id, data[i][1], type, stoi(data[i][3]), stoi(data[i][4]), nservices));
        if (d > T)
        {
            T = d;
        }
    }
    return n - 1; // Return the number of trains (excluding header)
}

vector<int> read_operation_input(const string &filename, vector<Train> &trains, int ntrains, int nservices)
{
    vector<vector<string>> data;
    data = readCSV(filename);
    int n = data.size();
    int train;                            // stores train id
    int type;                             // stores service type
    vector<int> assignment(nservices, 0); // stores number of times a given service is assigned

    if (n < 1 || data[0].size() != 3)
    {
        cerr << "Wrong input file format, empty file or wrong number of columns: " << filename << endl;
        cerr << "Number of columns: " << data[0].size() << endl;
        cerr << "Expected number of columns: 3" << endl;
        return assignment;
    }
    for (int i = 1; i < n; i++)
    {
        type = stoi(data[i][1]);
        train = stoi(data[i][2]);
        if (train <= 0 || train > ntrains || type <= 0 || type > nservices)
        {
            cerr << "Invalid input file, train or service type invalid: " << filename << endl;
            cerr << "Operation ID: " << stoi(data[i][0]) << endl;
            cerr << "Train ID: " << train << ", Service type: " << type << endl;
            cerr << "Number of trains: " << ntrains << ", Number of services: " << nservices << endl;
            return assignment;
        }
        if (trains[train - 1].get_service(type))
        {
            cerr << "Invalid input file, service assigned twice or more to the same train: " << filename << endl;
            cerr << "Operation ID: " << stoi(data[i][0]) << endl;
            cerr << "Train ID: " << train << ", Service type: " << type << endl;
            cerr << "Number of trains: " << ntrains << ", Number of services: " << nservices << endl;
            return assignment;
        }
        trains[train - 1].set_service(type);
        assignment[type - 1]++;
    }
    return assignment; // Return the number of operations (excluding header)
}

