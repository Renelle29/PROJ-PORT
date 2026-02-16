#ifndef TRAIN_H
#define TRAIN_H

#include <string>
#include <vector>
using namespace std;

/*
Train class
This class represents a train in the network.
It contains information about the train's ID, name, type, arrival time, departure time,
the path it will take, and the services it will perform.
*/
class Train
{
private:
    int ID;
    string name;
    int type;
    int a_t;
    int d_t;
    vector<int> Npath;      // vector of nodes that the train will visit
    vector<int> Apath;      // vector of arcs that the train will visit
    vector<bool> services;  // vector of services that the train is assigned
    vector<int> p_services; // vector of services that the train performs
    vector<bool> A_b; // vector of arcs
    int dummy;        // dummy arc index

public:
    Train(int id, string n, int t, int a, int d, int ns);
    Train();

    void set_Npath(vector<int> p) { Npath = p; }
    void clear_Npath() { Npath.clear(); }
    void set_Apath(vector<int> p) { Apath = p; }
    void clear_Apath() { Apath.clear(); }

    void set_service(int s) { services[s - 1] = true; }
    void set_p_service(int s) { p_services[s - 1] += 1; }
    void clear_p_service() { p_services = vector<int>(p_services.size(), 0); }

    void add_arcb(bool b) { A_b.push_back(b); }
    void set_dummy(int d) { dummy = d; }

    int get_ID() const;
    string get_name() const;
    int get_type() const;
    int get_a_t() const;
    int get_d_t() const;
    vector<int> get_Npath() const { return Npath; }
    vector<int> get_Apath() const { return Apath; }

    vector<bool> get_services() const;
    bool get_service(int s) const { return services[s - 1]; }
    int get_p_service(int s) const { return p_services[s - 1]; }

    vector<bool> get_bools() const { return A_b; }
    bool get_arc(int i) const { return A_b[i]; }
    int get_dummy() const { return dummy; }
};

/*
Train_Type class
This class represents a train type in the network.
It contains information about the train type's ID, name, shunting weight and turnaround time.
*/
class Train_Type
{
private:
    int ID;
    string name;
    int w_shu;
    int t_turn;

public:
    Train_Type(int id, string n, int w, int t);
    Train_Type();

    int get_ID() const;
    string get_name() const;
    int get_w_shu() const;
    int get_t_turn() const;
};

#endif
