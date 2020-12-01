
#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"
#include "navigation/global_planner.h"

namespace planning {

class AWBPlanner {
public:
     AWBPlanner(){};
     
     void load_map(){};
     void train() {};
     void generate_sampled_graph() {};
     void generate_plan() {}

private:
    Graph map_workspace_;
    A_star ws_planner_;
    A_star config_space_planner_;

    Eigen::VectorXd weights_;
    //To Add: Features class "FeatureCalc", can take the astar ws_planner as an argument to contructor 
    //        or function generate the features to one of it's function.

     
};

}

