#include <cmath>
#include <iterator>
#include <shared/global_utils.h>
#include "planning.h"
#include "shared/math/line2d.h"
#include "shared/math/geometry.h"

using Eigen::Vector2f;
using navigation::PoseSE2;


namespace planning {
    
    void FeatureCalc::generateEllipDistValues(const navigation::PoseSE2& start,
                                                const navigation::PoseSE2& goal){
        std::list<GraphIndex> pathToStart;
        std::list<GraphIndex> pathToGoal;
        std::list<GraphIndex> completePath;

        float costToStart;
        float costToGoal;
        float costComplete;

        float ellipDist;
        GraphIndex s = graph_.GetClosestVertex(start);
        GraphIndex g = graph_.GetClosestVertex(goal);

        completePath = planner_.generatePath(start, goal);
        costComplete = planner_.getLocationCost(g);

        int numX = graph_.getNumVerticesX();
        int numY = graph_.getNumVerticesY();
        int numOrient = graph_.getNumOrient();

        for (int x = 0; x < numX; ++x) {
           for (int y = 0; y < numY; ++y) {
               for (int o = 0; o < numOrient; ++o) {
                    GraphIndex curr_vertex(x,y,o);
                    Eigen::Vector2f loc = graph_.GetLocFromVertexIndex(numX,numY);
                    PoseSE2 current(loc.x(),loc.y(),0);
                    pathToStart = planner_.generatePath(current, start);
                    costToStart = planner_.getLocationCost(s);
                    pathToGoal = planner_.generatePath(current, goal);
                    costToGoal = planner_.getLocationCost(g);

                    ellipDist = costToStart + costToGoal - costComplete;
                    ellipDistValues_[curr_vertex] = ellipDist;
                }
            }
        } 
    }

    float FeatureCalc::getEllipPathDist(GraphIndex& index){
        return ellipDistValues_[index];
    }

    void FeatureCalc::generateFrvValues() {
        int numX = graph_.getNumVerticesX();
        int numY = graph_.getNumVerticesY();
        int numOrient = graph_.getNumOrient();

        for (int x = 0; x < numX; ++x) {
            for (int y = 0; y < numY; ++y) {
                for (int o = 0; o < numOrient; ++o) {
                    GraphIndex curr_vertex(x,y,o);
                    Eigen::Vector2f loc = graph_.GetLocFromVertexIndex(numX,numY);
                    frvValues_[curr_vertex] = calcFrv(loc);

                }
            }
        }

    }

    float FeatureCalc::calcFrv(Eigen::Vector2f loc) {
        float min_dist = std::numeric_limits<float>::max();
        for (auto &line: map_.lines) {
            float dist_to_line;
            Vector2f project_point;
            geometry::ProjectPointOntoLineSegment(
                    loc,
                    line.p0,
                    line.p1,
                    &project_point,
                    &dist_to_line);
            dist_to_line = std::sqrt(dist_to_line);
            if (dist_to_line < min_dist)
                min_dist = dist_to_line;
        }

        return min_dist;
    }

    float FeatureCalc::GetFrvValue(GraphIndex& index) {
        return frvValues_[index];
    }


    void  AWBPlanner::LoadMap(const std::string& map_file) {
        std::string full_map_file;

        if (map_file.find('.') == std::string::npos) {
           full_map_file = "maps/" + map_file + "/" + map_file + ".vectormap.txt";
        } else {
           full_map_file = map_file;
        }

        cout << "Loading map file '" << full_map_file << "'...." << endl;
        map_.Load(full_map_file);
        cout << "Done loading map." << endl;\

    }

    PoseSE2 AWBPlanner::SampleUniform(const Eigen::Vector2f& x_bounds,
                                      const Eigen::Vector2f& y_bounds,
                                      const Eigen::Vector2f& angle_bounds) {
        return PoseSE2(generator_.UniformRandom(x_bounds(0), x_bounds(1)),
                       generator_.UniformRandom(y_bounds(0), y_bounds(1)),
                       generator_.UniformRandom(angle_bounds(0), angle_bounds(1)));
    }

    bool AWBPlanner::GenerateEdge(PoseSE2 v1, navigation::PoseSE2 v2) {
        geometry::line2f edge(v1.loc, v2.loc);
        for (auto &line: map_.lines) {
          if (line.CloserThan(edge.p0,edge.p1,margin_to_wall_))
             return false;
        }
        return true;

    }

    void AWBPlanner::GenerateSampledGraphUniform(const PoseSE2& start, const Vector2f& goal) {
        SimpleGraph start_tree;
        SimpleGraph goal_tree;

        Vector2f angle_bounds(-M_PI, M_PI);
        start_tree.AddVertex(start);
        goal_tree.AddVertex(PoseSE2(goal,generator_.UniformRandom(angle_bounds(0),angle_bounds(1))));

        unsigned int connect_trees = 100;
        std::size_t step_num = 0;
        std::size_t search_max_steps = 1000000;

        while(step_num < search_max_steps) {
            PoseSE2 cspace_point = SampleUniform(map_x_bounds_,map_y_bounds_,angle_bounds);
            std::size_t closest_vertex_in_tree = start_tree.GetClosestVertex(cspace_point);
            PoseSE2 closest_vertex_pose = start_tree.GetVertexPose(closest_vertex_in_tree);
            if (GenerateEdge(cspace_point ,closest_vertex_pose)) {
                std::size_t new_vertex = start_tree.AddVertex(cspace_point);
                start_tree.AddEdge(closest_vertex_in_tree, new_vertex);
                cspace_point = SampleUniform(map_x_bounds_,map_y_bounds_,angle_bounds);
            }

            closest_vertex_in_tree = goal_tree.GetClosestVertex(cspace_point);
            closest_vertex_pose = goal_tree.GetVertexPose(closest_vertex_in_tree);
            if (GenerateEdge(cspace_point ,closest_vertex_pose)) {
                std::size_t new_vertex = goal_tree.AddVertex(cspace_point);
                goal_tree.AddEdge(closest_vertex_in_tree, new_vertex);
            }

            if ((step_num % connect_trees) == 0) {
                for (std::size_t i = 0; i < start_tree.GetNumVertices(); ++i) {
                    PoseSE2 start_tree_vertex_pose = start_tree.GetVertexPose(i);
                    for (std::size_t j = 0; j < goal_tree.GetNumVertices(); ++j) {
                        PoseSE2 goal_tree_vertex_pose =  goal_tree.GetVertexPose(j);
                        if (GenerateEdge(start_tree_vertex_pose, goal_tree_vertex_pose)) {
                            start_tree.MergeGraph(goal_tree, start_tree.GetVertex(i),
                                                  goal_tree.GetVertex(j));
                            cspace_graph = start_tree;
                            return;
                        }
                    }
                }
            }
            ++step_num;
        }
    }
}

