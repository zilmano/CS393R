#include "planning.h"

#include <signal.h>
#include "visualization/visualization.h"
#include "ros/ros.h"
#include "gflags/gflags.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "shared/util/timer.h"

using Eigen::Vector2f;
using planning::FeatureCalc;
using amrl_msgs::VisualizationMsg;
using visualization::ClearVisualizationMsg;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::DrawLine;
using visualization::DrawParticle;

ros::Publisher visualization_publisher_;
VisualizationMsg vis_msg_;
bool run_ = true;

void initialize_msgs() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.seq = 0;

  vis_msg_ = visualization::NewVisualizationMessage("map", "adaptive_sample_based_planning");
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void visualize_graph(const planning::SimpleGraph& graph, long int color=0x000FF, bool clear=true){

    cout << "Start Visulize Graph" << endl;
    auto V = graph.GetVertices();
    if (clear)
        visualization::ClearVisualizationMsg(vis_msg_);

    //visualization::DrawCross(Eigen::Vector2f(0,0), 0.5, 0x000FF, map_viz_msg_);
    cout << "graph size:" << V.size() << endl;
    for (std::size_t id = 0; id < V.size(); ++id) {

        PoseSE2 vertex_pose = V[id]->pose;
                    //geometry::line2f edge(
                    //        graph.GetLocFromVertexIndex(x_id,y_id),
                    //       graph.GetLocFromVertexIndex(neighbor.x,neighbor.y));
                    /*visualization::DrawLine(
                            graph.GetLocFromVertexIndex(x_id,y_id),
                            graph.GetLocFromVertexIndex(neighbor.x,neighbor.y),
                            0x0000000,
                            map_viz_msg_);*/
        visualization::DrawCross(vertex_pose.loc, 0.1, color, vis_msg_);

    }
    cout << "Publishing vis msg..." <<  endl;
    visualization_publisher_.publish(vis_msg_);
 }

long unsigned int test_uniform_graph_gen(planning::AWBPlanner& planner) {
   cout << "Running Uniform RRT sampling test."<< endl;
   planner.GenerateSampledGraphUniform(PoseSE2(-33,19.9,0),Vector2f(-8,14));
   auto res = planner.GetCSpaceGraph().GetNumVertices();
   cout << "Finshed RRT. reward " << res << endl;
   return res;

}

long unsigned int test_adaptive_graph_gen(planning::AWBPlanner& planner) {
   PoseSE2 start(-33,19.9,0);
   Vector2f goal(-8,14);

   cout << "Running Adaptive RRT sampling test."<< endl;
   const std::shared_ptr<FeatureCalc> feature_calc = planner.GetFeatureCalc();
   cout << "Generating Frv values..." << endl;
   feature_calc->GenerateFrvValues();
   feature_calc->GenerateEllipDistValues(start, PoseSE2(goal,0));
   feature_calc->GenerateEllipDistBitmap();
   cout << "Calculate Gibbs distribution..." << endl;
   planner.InitWeights();
   planner.RecalcAdpatedDistribution();
   planner.GenerateProbabilityBitmap();
   cout << "Generate Sampled Graph:" << endl;
   planner.GenerateSampledGraphAdaptive(start, goal);
   auto res = planner.GetCSpaceGraph().GetNumVertices();
   cout << "Finshed RRT. reward " << res << endl;
   return res;
}

void test_training(planning::AWBPlanner& planner) {
    cout << "Start Training Policy Gradient..."<< endl;
    planner.Train();
    cout << "Done training." << endl;
    cout << "Generate PRobability image..." << endl;
    planner.GenerateProbabilityBitmap();
    cout << " Generate Sampled Graph" << endl;
    PoseSE2 start(-33,19.9,0);
    Vector2f goal(-8,14);
    planner.GenerateSampledGraphAdaptive(start, goal);
    //planner.GenerateProbabilityBitmap();

}

void test_fvr(const planning::AWBPlanner& planner) {
    cout << "Running fvr caluclation test."<< endl;
    const std::shared_ptr<FeatureCalc> feature_calc = planner.GetFeatureCalc();
    cout << "Generating Frv values..." << endl;
    feature_calc->GenerateFrvValues();
    cout << "Generating Frv Bitmap..." << endl;
    feature_calc->GenerateFrvBitmap();
}

void test_ellip_path(const planning::AWBPlanner& planner) {
    cout << "Running ellip path caluclation test."<< endl;
    const std::shared_ptr<FeatureCalc> feature_calc = planner.GetFeatureCalc();

    cout << "Generating Frv values..." << endl;
    PoseSE2 start(-33,19.9,0);
    PoseSE2 goal(33,19.9,0);
    feature_calc->GenerateEllipDistValues(start, goal);
    cout << "Generating Frv Bitmap..." << endl;
    feature_calc->GenerateEllipDistBitmap();
}


int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, false);
    signal(SIGINT, SignalHandler);
    ros::init(argc, argv, "adaptive_sample_based_planning",ros::init_options::NoSigintHandler);
    ros::NodeHandle n;


    visualization_publisher_ =
          n.advertise<VisualizationMsg>("visualization", 1);
    vis_msg_ = visualization::NewVisualizationMessage(
            "map",
            "adaptive_sample_based_planning");
    //initialize_msgs();

    Vector2f map_x_bounds(-50,50);
    Vector2f map_y_bounds(0,36);
    float margin_to_wall = 0.1;
    float ws_grid_spacing = 0.05;
    unsigned long int random_seed = 23;
    unsigned int feature_num = 2;

    planning::AWBPlanner sample_based_planner(
            map_x_bounds,
            map_y_bounds,
            margin_to_wall,
            ws_grid_spacing,
            feature_num,
            random_seed);

    sample_based_planner.LoadMap("GDC1");


    // Enter your test code here
    //test_fvr(sample_based_planner);
    //test_adaptive_graph_gen(sample_based_planner);
    //test_training(sample_based_planner);
    //test_ellip_path(sample_based_planner);
    //test_uniform_graph_gen(sample_based_planner);
    test_training(sample_based_planner);


    RateLoop loop(20.0);
    while (run_ && ros::ok()) {
        ros::spinOnce();
        visualize_graph(sample_based_planner.GetCSpaceGraph());
        //visualize_graph(sample_based_planner.GetStartTreeGraph());
        //visualize_graph(sample_based_planner.GetGoalTreeGraph(),0xFF00,false);
        loop.Sleep();
    }

    return 0;
}
