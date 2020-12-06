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

        PoseSE2 vertex_pose = V[id].pose;
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

void test_uniform_graph_gen(planning::AWBPlanner& planner) {
   cout << "Running Uniform RRT sampling test."<< endl;
   planner.GenerateSampledGraphUniform(PoseSE2(-33,19.9,0),Vector2f(-8,14));
   cout << "Finshed RRT." << endl;
   //visualize_graph(planner.GetCSpaceGraph());

}

void test_fvr(const planning::AWBPlanner& planner) {
    cout << "Running fvr caluclation test."<< endl;
    const std::shared_ptr<FeatureCalc> feature_calc = planner.GetFeatureCalc();

    cout << "Generating Frv values..." << endl;
    feature_calc->GenerateFrvValues();
    cout << "Generating Frv Bitmap..." << endl;
    feature_calc->GenerateFrvBitmap();
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

    planning::AWBPlanner sample_based_planner(
            map_x_bounds,
            map_y_bounds,
            margin_to_wall,
            ws_grid_spacing,
            random_seed);

    sample_based_planner.LoadMap("GDC1");


    // Enter your test code here
    //test_fvr(sample_based_planner);
    test_uniform_graph_gen(sample_based_planner);
    ////
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
