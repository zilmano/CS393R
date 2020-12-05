/*
 * global_planner.cc
 *
 *  Created on: Nov 15, 2020
 *      Author: olegzilm
 */


#include <cmath>
#include <iterator>
#include <shared/global_utils.h>
#include "global_planner.h"
#include "vector_map/vector_map.h"
#include "shared/math/line2d.h"


using std::cout;
using std::endl;
namespace planning {
    SimpleGraph::Neighbors SimpleGraph::GetVertexNeighbors(std::size_t index) {
        if (index >= vertices_.size()) {
            std::string msg = "SimpleGraph::GetVertex:: Requested Vertex is out of bounds.";
            cout << msg << endl;
            throw msg;
        }
        return vertices_[index].neighbors;
    }

    std::size_t SimpleGraph::AddVertex(const navigation::PoseSE2& pose) {
        vertices_.push_back(Node(pose));

        return vertices_.size()-1;
    }

    void SimpleGraph::AddEdge(std::size_t vertex, std::size_t neighbor) {
        if (vertex >= vertices_.size()) {
            std::string msg = "SimpleGraph::AddEdge:: Requested Vertex is out of bounds.";
            cout << msg << endl;
            throw msg;
        } else if (neighbor >= vertices_.size()) {
            std::string msg = "SimpleGraph::AddEdge:: Requested nieghbor is out of bounds.";
            cout << msg << endl;
            throw msg;
        }

        vertices_[vertex].neighbors.push_back(NodePtr(&vertices_[neighbor]));
        vertices_[neighbor].neighbors.push_back(NodePtr(&vertices_[vertex]));
    }

    navigation::PoseSE2 SimpleGraph::GetVertexPose(std::size_t index) {
            if (index >= vertices_.size()) {
                std::string msg = "SimpleGraph::GetVertexPose:: Requested Vertex is out of bounds.";
                cout << msg << endl;
                throw msg;
           }

        return vertices_[index].pose;
    }

    navigation::PoseSE2 SimpleGraph::GetVertexPose(const NodePtr& vertex) {
            return vertex->pose;
    }

    SimpleGraph::NodePtr SimpleGraph::GetVertex(std::size_t index) {
        if (index >= vertices_.size()) {
                    std::string msg = "SimpleGraph::GetVertex:: Requested Vertex is out of bounds.";
                    cout << msg << endl;
                    throw msg;
               }
        return NodePtr(&vertices_[index]);
    }

    void SimpleGraph::MergeGraph(const SimpleGraph& other,const NodePtr& edge_vertex,
                                 const NodePtr& edge_vertex_in_other) {
        edge_vertex->neighbors.push_back(edge_vertex_in_other);
        edge_vertex_in_other->neighbors.push_back(edge_vertex);
        const vector<Node>& other_vertices = other.GetVertices();
        vertices_.insert(std::end(vertices_), std::begin(other_vertices),std::end(other_vertices));


    }

    std::size_t SimpleGraph::GetClosestVertex(const navigation::PoseSE2& pose) {
        if (vertices_.size() == 0) {
                    std::string msg = "SimpleGraph::GetClosestVertex:: Graph is empty.";
                    cout << msg << endl;
                    throw msg;
        }
        std::size_t closest_vertex = 0;
        float closest_dist = std::numeric_limits<float>::max();
        for (std::size_t i = 0; i < vertices_.size(); ++i) {
            navigation::PoseSE2 vertex_pose = vertices_[i].pose;
            float dist_to_curr_vertex =
                    (pose.loc-vertex_pose.loc).squaredNorm() + pow((pose.angle-vertex_pose.angle),2);
            if (dist_to_curr_vertex < closest_dist) {
                closest_dist = dist_to_curr_vertex;
                closest_vertex = i;
            }
        }

        return closest_vertex;
    }


    void Graph::GenerateGraph(const vector_map::VectorMap& map) {
        double num_vertices_x, num_vertices_y;
        float frac = modf((x_end_-x_start_)/grid_spacing_, &num_vertices_x);
        if (frac == 0)
            num_vertices_x -= 1;
        frac = modf((y_end_-y_start_)/grid_spacing_, &num_vertices_y);
        if (frac == 0)
            num_vertices_y_ -= 1;

        // Initialize 3D vector. Each member is a list of Indexes of connected nodes.
        num_vertices_x_ = (int) num_vertices_x;
        num_vertices_y_ = (int) num_vertices_y;
        vertices_ = Vertices(
                num_vertices_x_,
                vec_2d(num_vertices_y_,
                vec_1d(num_of_orient_)));

        // Generate Edges between neighbor vertices
        for (int x = 0; x < num_vertices_x_; ++x) {
           for (int y = 0; y < num_vertices_y_; ++y) {
               for (int o = 0; o < num_of_orient_; ++o) {
                   list<GraphIndex> neighbors;
                   GraphIndex curr_vertex(x,y,o);
                   GetConnectedNeighbors(curr_vertex, neighbors);
                   neighbors = removeEdgesWithObstacles(curr_vertex, neighbors, map);
                   vertices_[x][y][o] = neighbors;
               }
           }
       }
    }


    list<GraphIndex> Graph::removeEdgesWithObstacles(const GraphIndex& vertex,
                                                    const list<GraphIndex>& neighbors,
                                                    const vector_map::VectorMap& map){
        list<GraphIndex> result;
        Eigen::Vector2f vertex_loc = GetLocFromVertexIndex(vertex.x,vertex.y);

        for (auto &neighbor : neighbors) {
            if (neighbor.x >= num_vertices_x_ || neighbor.y >= num_vertices_y_
                    || neighbor.x < 0 || neighbor.y < 0)
                continue;
            Eigen::Vector2f neighbor_loc =
                    GetLocFromVertexIndex(neighbor.x,neighbor.y);
            geometry::line2f edge(vertex_loc, neighbor_loc);
            if (!checkEdgeForObstacles(edge, map))
                result.push_back(neighbor);
        }
        return result;
    }

    bool Graph::checkEdgeForObstacles(geometry::line2f& edge,
                                      const vector_map::VectorMap& map) {
        for (auto &line: map.lines) {
            if (line.CloserThan(edge.p0,edge.p1,margin_to_wall_))
                return true;
        }
        return false;
    }

    void Graph::GetConnectedNeighbors(const GraphIndex& index,
                                      list<GraphIndex>& neighbors) {
       if (num_of_orient_ == 1) {
           neighbors.push_back(GraphIndex(index.x+1,index.y-1,index.orient,false));
           neighbors.push_back(GraphIndex(index.x+1,index.y,index.orient, false));
           neighbors.push_back(GraphIndex(index.x+1,index.y+1,index.orient, false));
           neighbors.push_back(GraphIndex(index.x,index.y+1,index.orient, false));
           neighbors.push_back(GraphIndex(index.x-1,index.y+1,index.orient, false));
           neighbors.push_back(GraphIndex(index.x-1,index.y,index.orient,  false));
           neighbors.push_back(GraphIndex(index.x-1,index.y-1,index.orient, false));
           neighbors.push_back(GraphIndex(index.x,index.y-1,index.orient, false));
       } else if (num_of_orient_ == 4) {
           switch (index.orient) {
              case 0:
              neighbors.push_back(GraphIndex(index.x+1,index.y-1,index.orient,false));
              neighbors.push_back(GraphIndex(index.x+1,index.y,index.orient, false));
              neighbors.push_back(GraphIndex(index.x+1,index.y+1,index.orient, false));
              break;
              case 1:
              neighbors.push_back(GraphIndex(index.x-1,index.y+1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x,index.y+1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x+1,index.y+1,index.orient, false));
              break;
              case 2:
              neighbors.push_back(GraphIndex(index.x-1,index.y-1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x-1,index.y,index.orient,  false));
              neighbors.push_back(GraphIndex(index.x-1,index.y+1,index.orient,  false));
              break;
              case 3:
              neighbors.push_back(GraphIndex(index.x-1,index.y-1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x,index.y-1,index.orient, false));
              neighbors.push_back(GraphIndex(index.x+1,index.y-1,index.orient, false));
              break;
            }
        } else if (num_of_orient_ == 8) {
            cout << "GetConnectedNeighbors:: num_of_orient_==8 not implemented";
            throw;
        } else {
            cout << "ERROR: Graph::GetConnectedNeighbors -> number of orientations should be 4, or 8. Others are not supported." << endl;
            throw;
        }
    }

    GraphIndex Graph::GetClosestVertex(const navigation::PoseSE2& pose) {
        double vertex_num_x, vertex_num_y, vertex_num_orient;
        float vertex_frac_x = modf((pose.loc.x()-x_start_)/grid_spacing_, &vertex_num_x);
        float vertex_frac_y = modf((pose.loc.y()-y_start_)/grid_spacing_, &vertex_num_y);

        float angle = NormalizeAngle(pose.angle);
        cout << vertex_num_x << " " << vertex_num_y << endl;
        if (vertex_num_x > num_vertices_x_ || vertex_num_y > num_vertices_y_) {
            cout << "ERROR: planning::Graph::getClosestVertex -> Provided pose is outside (above) of planning map bounds" << endl;
            throw;
        }

        if ((vertex_frac_x > 0.5 && (int) vertex_num_x != num_vertices_x_)
                || vertex_num_x == 0)
            vertex_num_x += 1;
        if ((vertex_frac_y > 0.5 && (int) vertex_num_y != num_vertices_y_)
                || vertex_num_y == 0)
            vertex_num_y += 1;

        if (vertex_num_x < 1 || vertex_num_y < 1) {
            cout << "planning::Graph::getClosestVertex --> Provided pose is outside (below) of planning map bounds" << endl;
            throw;
        }

        float vertex_frac_orient = modf(angle/(2*M_PI/(num_of_orient_)), &vertex_num_orient);
        if (vertex_frac_orient > 0.5)
            vertex_num_orient += 1;

        if (vertex_num_orient == num_of_orient_ || num_of_orient_ == 1)
            vertex_num_orient = 0;

        return GraphIndex((int)vertex_num_x-1,
                          (int)vertex_num_y-1,
                          (int)vertex_num_orient);
    }

    std::list<GraphIndex> Graph::GetVertexNeighbors(const GraphIndex& index) {
        if (index.x >= num_vertices_x_
                || index.y >= num_vertices_y_
                || index.orient > num_of_orient_
                || index.x < 0
                || index.y < 0
                || index.orient < 0) {
            cout << "ERROR: planning::Graph::GetVertexNeigbors -> Vertex Index is out of bounds in the graph" << endl;
            throw;
        }
        return vertices_[index.x][index.y][index.orient];
    }

    Eigen::Vector2f Graph::GetLocFromVertexIndex(int index_x,
                                                 int index_y) {
        if (index_x >= num_vertices_x_
              || index_y >= num_vertices_y_
              || index_x < 0
              || index_y < 0) {
            cout << "ERROR: planning::Graph::GetLocFromVertexIndex -> Vertex Index is out of bounds in the graph" << endl;
            throw;
        }
        return Eigen::Vector2f((index_x+1)*grid_spacing_+x_start_,(index_y+1)*grid_spacing_+y_start_);
    }

    float Graph::NormalizeAngle(float angle) {
        double decimal;
        float frac = modf(angle/(2*M_PI), &decimal);
        if (angle >= 0)
          return frac*2*M_PI;
        else
          return 2*M_PI*(1+frac);
    }

    std::list<GraphIndex> A_star::generatePath(const navigation::PoseSE2& start, const navigation::PoseSE2& goal){

        cout << "\n\nStarting generatePath..." << endl;
                debug::print_loc(start.loc," start loc", false);
                debug::print_loc(goal.loc," goal loc", true);

        findStartAndGoalVertex(start, goal);

        std::priority_queue<element, std::vector<element>, std::greater<element>> frontier;
        std::map<GraphIndex, GraphIndex> came_from;
        std::map<GraphIndex, double> cost_so_far;

        frontier.emplace(0,start_);
        came_from[start_] = start_;
        cost_so_far[start_] = 0;

        while(!frontier.empty()){
            GraphIndex current = frontier.top().second;
            frontier.pop();
           //cout << "Start\t X id:" << start_.x << " Start\t Y id:" << start_.y << std::endl;
           // cout << "Goal\t X id:" << goal_.x << " Goal\t Y id:" << goal_.y << std::endl;
           // cout << "Current X id:" << current.x << " Current Y id:" << current.y << std::endl;
           // cout << "Cost_so_far size: " << cost_so_far.size() << std::endl;
            if(current == goal_){
                break;
            }

            std::list<GraphIndex> neighbors = graph_.GetVertexNeighbors(current);
            for(auto &neighbor : neighbors){
                //cout << "Neighbor: " << "X id:" << neighbor.x << " Y id:" << neighbor.y << std::endl;
                double new_cost = cost_so_far[current] + A_star::calcCost(current, neighbor);
                //cout << "Neighbor cost:" << new_cost << " Current Cost:" << cost_so_far[current] << std::endl;
                if(cost_so_far.find(neighbor) == cost_so_far.end() || new_cost < cost_so_far[neighbor]){
                    cost_so_far[neighbor] = new_cost;
                    double priority = new_cost + A_star::calcHeuristic(neighbor);
                    frontier.emplace(priority, neighbor);
                    came_from[neighbor] = current;
                    //cout << "New cost found" << std::endl;
                }
            }
        }
        cout << "A* start Done." << std::endl;

        std::list<GraphIndex> path;
        GraphIndex curr = goal_;
        while(curr != start_){
            path.push_front(came_from[curr]);
            curr = came_from[curr];
        }

        path_ = path;
        curr_path_vertex_ = path_.begin();
        return path;
    }

    double A_star::calcCost(const GraphIndex& current, const GraphIndex& next){
        return std::sqrt(std::pow(next.x - current.x, 2) + std::pow(next.y - current.y, 2)*1.0);
    }

    double A_star::calcHeuristic(const GraphIndex& next){
        return std::sqrt(std::pow(goal_.x - next.x, 2) + std::pow(goal_.y - next.y, 2)*1.0);
    }

    void A_star::findStartAndGoalVertex(const navigation::PoseSE2& start, const navigation::PoseSE2& goal){
        start_ = graph_.GetClosestVertex(start);
        goal_ = graph_.GetClosestVertex(goal);
    }

    bool A_star::getPurePursuitCarrot(Eigen::Vector2f center,
                                      float radius, Eigen::Vector2f& interim_goal) {
        if (path_.empty())
            return false;
        //cout << " Pure Pursuit Start -- " << endl;
        //debug::print_loc(center, "     Car Loc: ");
        bool intersect_found = false;
        for (auto path_it = curr_path_vertex_;
                path_it != std::prev(path_.end()); ++path_it) {
            GraphIndex curr_vertex_id = *path_it;
            GraphIndex next_vertex_id = *(std::next(path_it));

            Eigen::Vector2f curr_vertex_loc = graph_.GetLocFromVertexIndex(
                    curr_vertex_id.x,
                    curr_vertex_id.y);
            Eigen::Vector2f next_vertex_loc = graph_.GetLocFromVertexIndex(
                                next_vertex_id.x,
                                next_vertex_id.y);
            //debug::print_loc(curr_vertex_loc,"    Curr Vertex:");
            //debug::print_loc(next_vertex_loc,"    Next Vertex:");

            geometry::line2f path_line(curr_vertex_loc, next_vertex_loc);
            Eigen::Vector2f intersect_point_1, intersect_point_2;
            unsigned int num_intersections =
                    geometry::line_circle_intersect(path_line,
                                                    center,
                                                    radius,
                                                    intersect_point_1,
                                                    intersect_point_2);
            if (num_intersections > 0) {
                cout << "        Intersect!" << endl;
                if (num_intersections == 1) {
                   interim_goal = intersect_point_1;
                } else {
                   interim_goal = next_vertex_loc;
                }
                curr_path_vertex_ = path_it;
                cout << "Curr path vertex: (" << curr_vertex_loc.x() << ", "
                        << curr_vertex_loc.y() <<")" << endl;
                intersect_found = true;
            }

        }

      return intersect_found;
    }

}



