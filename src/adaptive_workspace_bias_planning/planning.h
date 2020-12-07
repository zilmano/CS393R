
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
                unsigned int feature_num = 2,
                unsigned long int random_seed=0,
                unsigned int num_train_episodes=1000,
                float lr = 0.00001):
         map_x_bounds_(map_x_bounds),
         map_y_bounds_(map_y_bounds),
         margin_to_wall_(margin_to_wall),
         ws_graph_spacing_(ws_graph_spacing),
         feature_num_(feature_num),
         num_train_episodes_(num_train_episodes),
         lr_(lr){

         generator_ = util_random::Random(random_seed);
     };
     
     void LoadMap(const std::string& map_file);
     
     // Do the gradient based learning to find the best weights.
     void Train();
     
     // Use sampling using the weights to generate the C-Space graph, and store it in the new graph class.
     void GenerateSampledGraphUniform(const navigation::PoseSE2& start,
                                      const Eigen::Vector2f& goal);
     void GenerateSampledGraphAdaptive(const navigation::PoseSE2& start,
                                       const Eigen::Vector2f& goal);

     // Use A* to find the best plan in the (3D) configuration space
     void GeneratePlan() {};

     // For testing
     const std::shared_ptr<FeatureCalc> GetFeatureCalc() const { return feature_calc_;};
     const SimpleGraph& GetCSpaceGraph() { return cspace_graph_;};
     const SimpleGraph& GetStartTreeGraph() { return start_tree_;};
     const SimpleGraph& GetGoalTreeGraph() { return goal_tree_;};

     void GenerateProbabilityBitmap();
     //OLEG: Make private later, this is for test;
     void RecalcAdpatedDistribution();
     void InitWeights();



private:
    //Try to generate edge between two points in config space.
    bool GenerateEdge(navigation::PoseSE2 v1, navigation::PoseSE2 v2);

    // CSpace Sampling implementation, both uniform and adaptive
    void GenerateSampledGraphImpl(const navigation::PoseSE2& start,
                                  const Eigen::Vector2f& goal,
                                  bool adaptive);

    // Sample unifrom vertex in CSpace
    navigation::PoseSE2 SampleUniform(const Eigen::Vector2f& x_bounds,
                                 const Eigen::Vector2f& y_bounds,
                                 const Eigen::Vector2f& angle_bounds);

    // Sa,
    navigation::PoseSE2 SampleFromAdaptedDistribution(const Eigen::Vector2f& angle_bounds);

    void UpdateGradient(float reward);
    void SampleStartAndGoal(PoseSE2& pose, Eigen::Vector2f &loc);




private:
    //2D worksapce graph
    Graph workspace_graph_;

    //3D CSpace planning graph
    SimpleGraph cspace_graph_;

    // For testing
    SimpleGraph start_tree_;
    SimpleGraph goal_tree_;

    // Planner in workspace to calc elliptical distance.
    A_star ws_planner_;
    
    // OLEG: This has to be a modified A* implementation so that it works with the new graph class that represent the sampled graph.
    A_star cspace_planner_;

    // features, weights, and probabilities
    Eigen::VectorXf weights_;
    Eigen::VectorXf probabilities_;
    vector<GraphIndex> probabilities_indx_to_graph_indx_;
    Eigen::VectorXf E_prob_;
    
    // Parameters
    Eigen::Vector2f map_x_bounds_; // map is equivalent to workspace.
    Eigen::Vector2f map_y_bounds_; // map is equivalent to workspace.
    float margin_to_wall_;
    float ws_graph_spacing_;
    unsigned int feature_num_;
    unsigned int num_train_episodes_;
    float lr_;

    util_random::Random generator_;
    vector_map::VectorMap map_;
    std::shared_ptr<FeatureCalc> feature_calc_; // Feature Calculator class

};

class FeatureCalc{
public:
    FeatureCalc(A_star& planner, Graph& graph, vector_map::VectorMap map):
        planner_(planner), graph_(graph), map_(map),
        start_(GraphIndex(0,0,0)), goal_(GraphIndex(0,0,0)){
    };
    
    void GenerateEllipDistValues(const navigation::PoseSE2& start,
                                    const navigation::PoseSE2& goal);
    float GetEllipPathDist(const GraphIndex& index);

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
    std::map<GraphIndex, float> ellipDistValues_;
    std::map<GraphIndex, float> frvValues_;

    A_star planner_;
    Graph graph_;
    vector_map::VectorMap map_;
    GraphIndex start_;
    GraphIndex goal_;


};

}

