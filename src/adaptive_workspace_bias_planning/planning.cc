#include <cmath>
#include <iterator>
#include <shared/global_utils.h>
#include "planning.h"
#include "shared/math/line2d.h"
#include "shared/math/geometry.h"

// For visualization
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

using Eigen::Vector2f;
using navigation::PoseSE2;




namespace planning {
    template<typename EigenMatrixT>
    void NormalizedImWrite(const EigenMatrixT & in, const std::string & title) {
        cv::Mat cvimg;
        auto maxele = in.maxCoeff();
        Eigen::MatrixXf norm_img = in / maxele;
        cv::eigen2cv(norm_img, cvimg);
        cv::imwrite((title + ".png").c_str(), cvimg * 256.0);
    }



    void FeatureCalc::generateEllipDistValues(const navigation::PoseSE2& start,
                                                const navigation::PoseSE2& goal){
        std::list<GraphIndex> pathToStart;
        std::list<GraphIndex> pathToGoal;
        std::list<GraphIndex> completePath;

        float costToStart;
        float costToGoal;
        float costComplete;
        float ellipDist;
        GraphIndex g = graph_.GetClosestVertex(goal);

        completePath = planner_.generatePath(start, goal);
        costComplete = planner_.getLocationCost(g);

        int numX = graph_.getNumVerticesX();
        int numY = graph_.getNumVerticesY();
        int numOrient = graph_.getNumOrient();

        std::map<GraphIndex, double> startCosts = planner_.generateDijCost(start);
        std::map<GraphIndex, double> goalCosts = planner_.generateDijCost(goal);

        for (int x = 0; x < numX; ++x) {
           for (int y = 0; y < numY; ++y) {
               for (int o = 0; o < numOrient; ++o) {
                    GraphIndex curr_vertex(x,y,o);
                    costToStart = startCosts[curr_vertex];
                    costToGoal = goalCosts[curr_vertex];
                    ellipDist = costToStart + costToGoal - costComplete;
                    ellipDistValues_[curr_vertex] = ellipDist;
                }
            }
        } 
    }

    float FeatureCalc::getEllipPathDist(const GraphIndex& index){
        return ellipDistValues_[index];
    }

    void FeatureCalc::GenerateFrvValues() {
        int numX = graph_.getNumVerticesX();
        int numY = graph_.getNumVerticesY();
        int numOrient = graph_.getNumOrient();

        if (numOrient != 1) {
            std::string msg = "Cannot generate Bitmap for 3D image\n";
            cout << msg;
            throw msg;
        }

        float max_val = 0;
        for (int x = 0; x < numX; ++x) {
            for (int y = 0; y < numY; ++y) {
                    GraphIndex curr_vertex(x,y,0);
                    Eigen::Vector2f loc = graph_.GetLocFromVertexIndex(x, y);
                    frvValues_[curr_vertex] = CalcFrv(loc);
                    if (frvValues_[curr_vertex] > max_val)
                        max_val = frvValues_[curr_vertex];

            }
        }

        for (auto& it: frvValues_) {
            it.second /= max_val;
        }

    }

    float FeatureCalc::CalcFrv(const Eigen::Vector2f& loc) {
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

    float FeatureCalc::GetFrvValue(const GraphIndex& index) {
        return frvValues_[index];
    }

    void FeatureCalc::GenerateFrvBitmap() {
            Eigen::MatrixXf image;
            image.resize(graph_.getNumVerticesX(), graph_.getNumVerticesY());

            int numOrient = graph_.getNumOrient();
            if (numOrient != 1) {
                std::string msg = "Cannot generate Bitmap for 3D image\n";
                cout << msg;
                throw msg;
            }

            float max_val = 0;
            for (auto const& bit: frvValues_) {
                GraphIndex index = bit.first;
                float val = bit.second;
                image(index.x,index.y) = val;
                if (val > max_val)
                    max_val = val;
            }

            //scale image to 0-255
            image *= (255/max_val);

            cout << " Writing out bitmap " << endl;
            planning::NormalizedImWrite(image,"frv");

    }

    void FeatureCalc::GenerateEllipDistBitmap() {
        Eigen::MatrixXf image;
        image.resize(graph_.getNumVerticesX(), graph_.getNumVerticesY());

        int numOrient = graph_.getNumOrient();
        if (numOrient != 1) {
            std::string msg = "Cannot generate Bitmap for 3D image\n";
            cout << msg;
            throw msg;
        }

        float max_val = 0;
        for (auto const& bit: ellipDistValues_) {
            GraphIndex index = bit.first;
            float val = bit.second;
            image(index.x,index.y) = val;
            if (val > max_val)
                max_val = val;
        }

        //scale image to 0-255
        image *= (255/max_val);

        cout << " Writing out bitmap " << endl;
        planning::NormalizedImWrite(image,"ellipi_dist");

    }

    /*
    * ABWPlanner calss starts here
    */
    void  AWBPlanner::LoadMap(const std::string& map_file) {
        std::string full_map_file;

        if (map_file.find('.') == std::string::npos) {
           full_map_file = "maps/" + map_file + "/" + map_file + ".vectormap.txt";
        } else {
           full_map_file = map_file;
        }

        cout << "Loading map file '" << full_map_file << "'...." << endl;
        map_.Load(full_map_file);
        cout << "Done loading map." << endl;

        int num_orient = 1;
        cout << "Generating Workspace graph..." << endl;
        workspace_graph_ = Graph(ws_graph_spacing_,
                                 map_x_bounds_(0),
                                 map_x_bounds_(1),
                                 map_y_bounds_(0),
                                 map_y_bounds_(1),
                                 num_orient,
                                 margin_to_wall_,
                                 map_);
        cout << "Done creating workspace graph." << endl;
        ws_planner_ = A_star(workspace_graph_);
        feature_calc_.reset(new FeatureCalc(ws_planner_, workspace_graph_, map_));

        // Resize the holder of probabilities and probability index mapping to new map graph
       int numX = workspace_graph_.getNumVerticesX();
       int numY = workspace_graph_.getNumVerticesY();

       probabilities_.resize(numY*numX);
       probabilities_indx_to_graph_indx_.resize(numY*numX);
       //const std::map<GraphIndex, double>& frvValues = feature_calc_.GetFrvValues();
       //probabilities_.resize(frvValues.size());
       //probabilities_indx_to_graph_indx_.resize(frvValues.size());
       std::size_t i = 0;
       for (int x = 0; x < workspace_graph_.getNumVerticesX(); ++x) {
            for (int y = 0; y < workspace_graph_.getNumVerticesY(); ++y) {
                probabilities_indx_to_graph_indx_[i] = GraphIndex(x,y,0);
                ++i;
            }
       }
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
          if (line.CloserThan(edge.p0,edge.p1,margin_to_wall_) || edge.Length() > 2)
             return false;
        }
        return true;

    }

    void AWBPlanner::GenerateSampledGraphUniform(const PoseSE2& start,
                                                  const Vector2f& goal) {

        GenerateSampledGraphImpl(start,goal,false);
    }

    void AWBPlanner::GenerateSampledGraphAdaptive(const PoseSE2& start,
                                                  const Vector2f& goal) {
        GenerateSampledGraphImpl(start,goal,true);
    }



    void AWBPlanner::GenerateSampledGraphImpl(const PoseSE2& start,
                                              const Vector2f& goal,
                                              bool adaptive) {
        SimpleGraph start_tree;
        SimpleGraph goal_tree;

        Vector2f angle_bounds(-M_PI, M_PI);
        start_tree.AddVertex(start);
        goal_tree.AddVertex(PoseSE2(goal,generator_.UniformRandom(angle_bounds(0),angle_bounds(1))));

        unsigned int connect_trees = 100;
        std::size_t step_num = 0;
        std::size_t search_max_steps = 1000000;

        while(step_num < search_max_steps) {
            PoseSE2 cspace_point;
            if (!adaptive)
                cspace_point = SampleUniform(map_x_bounds_,map_y_bounds_,angle_bounds);
            else
                cspace_point = SampleFromAdaptedDistribution(angle_bounds);

            std::size_t closest_vertex_in_tree = start_tree.GetClosestVertex(cspace_point);
            PoseSE2 closest_vertex_pose = start_tree.GetVertexPose(closest_vertex_in_tree);
            if (GenerateEdge(cspace_point ,closest_vertex_pose)) {
                std::size_t new_vertex = start_tree.AddVertex(cspace_point);
                start_tree.AddEdge(closest_vertex_in_tree, new_vertex);
                if (!adaptive)
                    cspace_point = SampleUniform(map_x_bounds_,map_y_bounds_,angle_bounds);
                else
                    cspace_point = SampleFromAdaptedDistribution(angle_bounds);
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
                            cspace_graph_ = start_tree;
                            cout << "made a graph! steps " << step_num << endl;
                            return;
                        }
                    }
                }
            }

            ++step_num;
            if ((step_num % 10000)==0) {
                cout << "-- Done sampling " << step_num << " steps."
                     << " Stree size: " << start_tree.GetNumVertices()
                     << " Gtree size: " << goal_tree.GetNumVertices() << endl;
            }
        }

        cout << "couldn't make a graph! steps " << step_num << endl;
        start_tree_ = start_tree;
        goal_tree_ = goal_tree;
    }

    void AWBPlanner::RecalcAdpatedDistribution() {

        std::size_t i = 0;
        float last_accum_prob = 0;
        Eigen::VectorXf features;
        features.resize(feature_num_);
        E_prob_.resize(feature_num_);
        for (int x = 0; x < workspace_graph_.getNumVerticesX(); ++x) {
            for (int y = 0; y < workspace_graph_.getNumVerticesY(); ++y) {
                GraphIndex graph_index(x, y, 0);

                float feature_val = feature_calc_->GetFrvValue(graph_index);
                if (feature_val != 0) {
                    feature_val = 1-feature_val;
                }
                features << feature_val, 0;
                // OLEG TO DO: Enable when generateEllipDistValues is available
                // features << feature_val, feature_calc_->GetEllipPathDist(graph_index);
                //features << 0;

                float curr_prob = std::exp(weights_.dot(features));
                cout << "curr_prob:" << curr_prob << " feature val:" << feature_val << "energy:" << weights_.dot(features) << endl;
                probabilities_(i) = last_accum_prob + curr_prob;
                E_prob_ += curr_prob*features;
                last_accum_prob += curr_prob;
                ++i;
            }
        }

        probabilities_ /= last_accum_prob;
        E_prob_ /= last_accum_prob;
        /*for (auto i = 0; i < probabilities_.size(); ++i) {
            cout << probabilities_(i) << endl;
        }*/
        cout << "Num Probs:" << i << endl;
    }

    void AWBPlanner::GenerateProbabilityBitmap() {
        Eigen::MatrixXf image;
        image.resize(workspace_graph_.getNumVerticesX(),workspace_graph_.getNumVerticesY());

        int numOrient = workspace_graph_.getNumOrient();
        if (numOrient != 1) {
            std::string msg = "Cannot generate Bitmap for 3D image\n";
            cout << msg;
            throw msg;
        }

        GraphIndex index = probabilities_indx_to_graph_indx_[0];
        image(index.x,index.y) = probabilities_(0);
        for (long int i=1; i < probabilities_.size(); ++i) {
            GraphIndex index = probabilities_indx_to_graph_indx_[i];
            float val = probabilities_(i) -probabilities_(i-1);
            image(index.x,index.y) = val;
        }

        cout << " Writing out bitmap: " << endl;
        planning::NormalizedImWrite(image,"SamplingProbability");

    }

    navigation::PoseSE2 AWBPlanner::SampleFromAdaptedDistribution(const Eigen::Vector2f& angle_bounds) {
        float x = generator_.UniformRandom(0,1);

        // Sanity check
        if (probabilities_.size() !=
                (long int)(workspace_graph_.getNumVerticesX()*
                           workspace_graph_.getNumVerticesY())) {
            std::string msg = "AWBPlanner::SampleFromAdaptedDistribution: Internal Error- Discrepancies with array sizes";
            cout << msg << endl;
            throw msg;
        }

        if (x <= probabilities_(0)) {
            GraphIndex graph_index = probabilities_indx_to_graph_indx_[0];
            return PoseSE2(
                    workspace_graph_.GetLocFromVertexIndex(
                            graph_index.x,
                            graph_index.y),
                    generator_.UniformRandom(angle_bounds(0), angle_bounds(1)));
        }

        for(long int i = 1; i < probabilities_.size(); ++i) {
            if (x <=  probabilities_(i)) {
                GraphIndex graph_index = probabilities_indx_to_graph_indx_[i];
                return PoseSE2(
                        workspace_graph_.GetLocFromVertexIndex(
                                graph_index.x,
                                graph_index.y),
                        generator_.UniformRandom(angle_bounds(0), angle_bounds(1)));
            }
        }

        // Sanity check
        std::string msg = "AWBPlanner::SampleFromAdaptedDistribution: some issue with probabilities_, do not sum to 1.";
        cout << msg << endl;
        throw msg;

        return PoseSE2(0,0,0);
    }

    void AWBPlanner::Train() {

        InitWeights();

        //int batch_size = 5;
        float reward = 0;
        for (unsigned int i = 0; i < num_train_episodes_; ++i) {
            PoseSE2 start(-33,19.9,0);
            Vector2f goal(-8,14);
            RecalcAdpatedDistribution();
            //SampleStartAndGoal(start,goal);
            GenerateSampledGraphAdaptive(start, goal);
            reward += -cspace_graph_.GetNumVertices();
            //if (i % batch_size == 0) { // update
            UpdateGradient(reward);
            reward = 0;
            //}
        }
        RecalcAdpatedDistribution();
    }

    void AWBPlanner::UpdateGradient(float reward) {

        Eigen::VectorXf grad_ni;
        Eigen::VectorXf features;
        grad_ni.resize(feature_num_);
        grad_ni << 0, 0;
        features.resize(feature_num_);
        for (auto& node: cspace_graph_.GetVertices()) {
            PoseSE2 vertex_pose = node.pose;
            vertex_pose.angle = 0; // Reduce diminsionality to 2D
            float f_rv = feature_calc_->GetFrvValue(workspace_graph_.GetClosestVertex(vertex_pose));
            float f_elp = 0; // Oleg: Re-enable when Ellip dist is ready
            features << f_rv, f_elp;
            grad_ni += features;
        }

        // OLEG, in this case the R/T -s -1, because the reward function is equal to
        grad_ni = (reward/cspace_graph_.GetNumVertices())*grad_ni - reward*E_prob_;
        weights_ += lr_*grad_ni;
    }

    void AWBPlanner::SampleStartAndGoal(PoseSE2& pose, Vector2f &loc) {
        //;

    }


    void AWBPlanner::InitWeights() {
        weights_.resize(feature_num_);
        weights_ << 10, 10;
    }









}

