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
        num_vertices_x_ = (unsigned int)num_vertices_x;
        num_vertices_x_ = (unsigned int)num_vertices_y;
        vertices_ = Vertices(
                num_vertices_x_,
                vec_2d(num_vertices_y_,
                vec_1d(num_of_orient_)));

        // Generate Edges between neighbor vertices
        for (size_t x = 0; x < num_vertices_x_; ++x) {
           for (size_t y = 0; y < num_vertices_y_; ++y) {
               for (size_t o = 0; o < num_of_orient_; ++o) {
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
        for (auto &neighbor : neighbors) {
            geometry::line2f edge(Eigen::Vector2f(vertex.x,vertex.y),
                                  Eigen::Vector2f(neighbor.x,neighbor.y));
            if (!checkEdgeForObstacles(edge, map))
                result.push_back(neighbor);
        }
        return result;
    }

    bool Graph::checkEdgeForObstacles(geometry::line2f& edge,
                                      const vector_map::VectorMap& map) {
        for (auto &line: map.lines) {
            if (line.CloserThan(edge.p0,edge.p1,margin_to_wall_))
                return false;
        }
        return true;
    }

    void Graph::GetConnectedNeighbors(const GraphIndex& index,
                                      list<GraphIndex>& neighbors) {
       if (num_of_orient_ == 4) {
            switch (index.orient) {
              case 0:
                  neighbors.push_back(GraphIndex(index.x+1,index.y-1,index.orient));
                  neighbors.push_back(GraphIndex(index.x+1,index.y,index.orient));
                  neighbors.push_back(GraphIndex(index.x+1,index.y+1,index.orient));
                  break;
              case 1:
                  neighbors.push_back(GraphIndex(index.x-1,index.y+1,index.orient));
                  neighbors.push_back(GraphIndex(index.x,index.y+1,index.orient));
                  neighbors.push_back(GraphIndex(index.x+1,index.y+1,index.orient));
                  break;
              case 2:
                  neighbors.push_back(GraphIndex(index.x-1,index.y-1,index.orient));
                  neighbors.push_back(GraphIndex(index.x-1,index.y,index.orient));
                  neighbors.push_back(GraphIndex(index.x-1,index.y+1,index.orient));
                  break;
              case 3:
                  neighbors.push_back(GraphIndex(index.x-1,index.y-1,index.orient));
                  neighbors.push_back(GraphIndex(index.x,index.y-1,index.orient));
                  neighbors.push_back(GraphIndex(index.x+1,index.y-1,index.orient));
                  break;
            }
        } else if (num_of_orient_ == 8) {
            cout << "Not implemented";
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
        if (vertex_num_x > num_vertices_x_ || vertex_num_y > num_vertices_y_) {
            cout << "ERROR: planning::Graph::getClosestVertex -> Provided pose is outside (above) of planning map bounds" << endl;
            throw;
        }

        if ((vertex_frac_x > 0.5 && (unsigned int) vertex_num_x != num_vertices_x_)
                || vertex_num_x == 0)
            vertex_num_x += 1;
        if ((vertex_frac_y > 0.5 && (unsigned int) vertex_num_y != num_vertices_y_)
                || vertex_num_y == 0)
            vertex_num_y += 1;

        if (vertex_num_x < 1 || vertex_num_y < 1) {
            cout << "planning::Graph::getClosestVertex --> Provided pose is outside (below) of planning map bounds" << endl;
            throw;
        }

        float vertex_frac_orient = modf(angle/(2*M_PI/(num_of_orient_)), &vertex_num_orient);
        if (vertex_frac_orient > 0.5)
            vertex_num_orient += 1;

        if (vertex_num_orient == num_of_orient_)
            vertex_num_orient = 0;

        return GraphIndex((unsigned int)vertex_num_x,
                          (unsigned int)vertex_num_y,
                          (unsigned int)vertex_num_orient);
    }

    std::list<GraphIndex> Graph::GetVertexNeigbors(const GraphIndex& index) {
        if (index.x > num_vertices_x_
                || index.y > num_vertices_y_
                || index.orient > num_of_orient_
                || index.x < 1
                || index.y < 1) {
            cout << "ERROR: planning::Graph::GetVertexNeigbors -> Vertex Index is out of bounds in the graph" << endl;
            throw;
        }
        return vertices_[index.x-1][index.y-1][index.orient];
    }

    float Graph::NormalizeAngle(float angle) {
        double decimal;
        float frac = modf(angle/(2*M_PI), &decimal);
        if (angle >= 0)
          return frac*2*M_PI;
        else
          return 2*M_PI*(1+frac);
    }



}



