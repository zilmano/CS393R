
#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"
#include "navigation/global_planner.h"
#include <map>

namespace planning {

class AWBPlanner {
public:
     AWBPlanner(){};
     
     void load_map(){};
     
     // Do the gradient based learning to find the best weights.
     void train() {};
     
     // Use sampling using the weights to generate the graph, and store it in the new graph class.
     void generate_sampled_graph() {};
     
     // Use A* to find the best plan in the 3D configuration space
     void generate_plan() {};

private:
    //Try to generate edge between two points in config space.
    void generateEdge() {};

    // our line sampling from assigment 2 goes here.
    void sample_from_disribution() {}; 



private:
    Graph map_workspace_;
    A_star ws_planner_;
    
    // OLEG: This has to be a modified A* implementation so that it works with the new graph class that represent the sampled graph.
    A_star config_space_planner_;

    // Vector holding our weights used for the Gibbs sampling
    Eigen::VectorXd weights_;
    
    //To Add member: Features class "FeatureCalc", can take the astar ws_planner as an argument to contructor 
    //        or function generate the features to one of it's function.

    //To add member: Class to store the sampled config space graph.
     
};

class FeatureCalc{
public:
    FeatureCalc(A_star& planner, Graph& graph):
        planner_(planner), graph_(graph), start_(GraphIndex(0,0,0)), goal_(GraphIndex(0,0,0)){
    };
    
    void generateEllipDistValues(const navigation::PoseSE2& start,
                                    const navigation::PoseSE2& goal);

    float getEllipPathDist(GraphIndex& index);
    

private:
    

private:
    std::map<GraphIndex, double> ellipDistValues_;
    A_star planner_;
    Graph graph_;
    GraphIndex start_;
    GraphIndex goal_;

};

}

