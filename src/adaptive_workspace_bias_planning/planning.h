
#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"
#include "navigation/global_planner.h"
#include <shared/global_utils.h>
#include <shared/util/random.h>
#include <map>

namespace planning {

class FeatureCalc;
class AWBPlanner {
public:
     AWBPlanner(const Eigen::Vector2f& map_x_bounds,
                const Eigen::Vector2f& map_y_bounds,
                float margin_to_wall,float ws_graph_spacing,
                unsigned long int random_seed=0):
         map_x_bounds_(map_x_bounds),
         map_y_bounds_(map_y_bounds),
         margin_to_wall_(margin_to_wall),
         ws_graph_spacing_(ws_graph_spacing){

         generator_ = util_random::Random(random_seed);
     };
     
     void LoadMap(const std::string& map_file);
     
     // Do the gradient based learning to find the best weights.
     void Train() {};
     
     // Use sampling using the weights to generate the graph, and store it in the new graph class.
     void GenerateSampledGraphUniform(const navigation::PoseSE2& start, const Eigen::Vector2f& goal);
     
     void GenerateSampledGraphAdaptive(const navigation::PoseSE2& start, const Eigen::Vector2f& goal) {};

     // Use A* to find the best plan in the (3D) configuration space
     void GeneratePlan() {};

     // For testing
     const std::shared_ptr<FeatureCalc> GetFeatureCalc() const { return feature_calc_;};
     const SimpleGraph& GetCSpaceGraph() { return cspace_graph_;};
     const SimpleGraph& GetStartTreeGraph() { return start_tree_;};
     const SimpleGraph& GetGoalTreeGraph() { return goal_tree_;};


private:
    //Try to generate edge between two points in config space.
    bool GenerateEdge(navigation::PoseSE2 v1, navigation::PoseSE2 v2);

    navigation::PoseSE2 SampleUniform(const Eigen::Vector2f& x_bounds,
                                 const Eigen::Vector2f& y_bounds,
                                 const Eigen::Vector2f& angle_bounds);

    // our line sampling from assignment 2 goes here.
    navigation::PoseSE2 SampleFromAdaptedDistribution(const Eigen::Vector2f& angle_bounds);

    void RecalcAdpatedDistribution();

    void InitWeights();

private:
    Graph workspace_graph_;

    SimpleGraph cspace_graph_;
    // Dbg trees
    SimpleGraph start_tree_;
    SimpleGraph goal_tree_;

    A_star ws_planner_;
    
    // OLEG: This has to be a modified A* implementation so that it works with the new graph class that represent the sampled graph.
    A_star cspace_planner_;

    // Vector holding our weights used for the Gibbs sampling
    Eigen::VectorXf weights_;
    Eigen::VectorXf probabilities_;
    vector<GraphIndex> probabilities_indx_to_graph_indx_;
    

    //To Add member: Features class "FeatureCalc", can take the astar ws_planner as an argument to contructor 
    //        or function generate the features to one of it's function.

    //To add member: Class to store the sampled config space graph.

    Eigen::Vector2f map_x_bounds_; // map is equivalent to workspace.
    Eigen::Vector2f map_y_bounds_; // map is equivalent to workspace.
    float margin_to_wall_;
    float ws_graph_spacing_;

    util_random::Random generator_;
    vector_map::VectorMap map_;
    std::shared_ptr<FeatureCalc> feature_calc_;

};

class FeatureCalc{
public:
    FeatureCalc(A_star& planner, Graph& graph, vector_map::VectorMap map):
        planner_(planner), graph_(graph), map_(map),
        start_(GraphIndex(0,0,0)), goal_(GraphIndex(0,0,0)){
    };
    
    void generateEllipDistValues(const navigation::PoseSE2& start,
                                    const navigation::PoseSE2& goal);
    float getEllipPathDist(const GraphIndex& index);

    void GenerateFrvValues();
    float GetFrvValue(const GraphIndex& index);
    
    // For testing
    void GenerateFrvBitmap();
    void GenerateEllipDistBitmap();


private:
    float CalcFrv(const Eigen::Vector2f& loc);

    // For Testing - Draw Image
    template<typename EigenMatrixT>
        void NormalizedImWrite(const EigenMatrixT & in, const std::string & title);

private:
    std::map<GraphIndex, double> ellipDistValues_;
    std::map<GraphIndex, double> frvValues_;

    A_star planner_;
    Graph graph_;
    vector_map::VectorMap map_;
    GraphIndex start_;
    GraphIndex goal_;


};

}

