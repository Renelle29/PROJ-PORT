#ifndef DWRESULTS_H
#define DWRESULTS_H

#include <vector>

class DWResults{

public:
    double continuous_obj;
    vector<pair<double,Path>> extended_paths;
    int np0;
};

#endif