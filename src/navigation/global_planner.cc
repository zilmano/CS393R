/*
 * global_planner.cc
 *
 *  Created on: Nov 15, 2020
 *      Author: olegzilm
 */


#include <cmath>
#include <shared/global_utils.h>
#include "global_planner.h"
#include "vector_map/vector_map.h"


namespace planning {

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
           neighbors.push_back(GraphIndex(index.x+1,index.y-1,index.orient, false));
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

    std::list<GraphIndex> A_star::generatePath(){
        frontier_.emplace(0,start_);
        came_from_[start_] = start_;
        cost_so_far_[start_] = 0;

        while(!frontier_.empty()){
            GraphIndex current = frontier_.top().second;
            frontier_.pop();
            cout << "Start\t X id:" << start_.x << " Start\t Y id:" << start_.y << std::endl;
            cout << "Goal\t X id:" << goal_.x << " Goal\t Y id:" << goal_.y << std::endl;
            cout << "Current X id:" << current.x << " Current Y id:" << current.y << std::endl;
            cout << "Cost_so_far size: " << cost_so_far_.size() << std::endl;
            if(current == goal_){
                break;
            }
            
            std::list<GraphIndex> neighbors = graph_.GetVertexNeighbors(current);
            for(auto &neighbor : neighbors){
                cout << "Neighbor: " << "X id:" << neighbor.x << " Y id:" << neighbor.y << std::endl;
                double new_cost = cost_so_far_[current] + A_star::calcCost(current, neighbor);
                cout << "Neighbor cost:" << new_cost << " Current Cost:" << cost_so_far_[current] << std::endl;
                if(cost_so_far_.find(neighbor) == cost_so_far_.end() || new_cost < cost_so_far_[neighbor]){
                    cost_so_far_[neighbor] = new_cost;
                    double priority = new_cost + A_star::calcHeuristic(neighbor);
                    frontier_.emplace(priority, neighbor);
                    came_from_[neighbor] = current;
                    cout << "New cost found" << std::endl;
                }
            }
        } 

        std::list<GraphIndex> path;
        GraphIndex curr = goal_;
        while(curr != start_){
            path.push_front(came_from_[curr]);
            curr = came_from_[curr];
        }
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

}



