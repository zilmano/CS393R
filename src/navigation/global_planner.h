/*
 * global_planner.h
 *
 *  Created on: Nov 15, 2020
 *      Author: olegzilm
 */
#ifndef SRC_NAVIGATION_GLOBAL_PLANNER_
#define SRC_NAVIGATION_GLOBAL_PLANNER_

#include <vector>
#include <list>

namespace vector_map {
    class VectorMap;
};

namespace navigation {
    class PoseSE2;
};
using std::vector;
using std::list;

namespace planning {

struct GraphIndex {
    GraphIndex(int _x, int _y, int _orient, float check_neg=true) {
        if (check_neg && (_x < 0 || _y < 0 || _orient < 0)) {
            cout << x << " " <<  y << " " << orient << endl;
            cout << "ERROR: GraphIndex::GraphIndex -> indexes cannot be negative " << endl;
            throw;
        }
        x = _x;
        y = _y;
        orient = _orient;
    };

    bool operator==(const GraphIndex& rhs) {
        if (this->x == rhs.x && this->y == rhs.y && this->y == rhs.y)
            return true;
        return false;
    }
    int x;
    int y;
    int orient;
};

typedef vector<list<GraphIndex>> vec_1d;
typedef vector<vec_1d> vec_2d;
typedef vector<vec_2d> vec_3d;
typedef vec_3d Vertices;

class Graph {
public:
    Graph(float grid_spacing, int x_start, int x_end, int y_start,
          int y_end, int num_of_orient, float margin_to_wall, const vector_map::VectorMap& map):
          grid_spacing_(grid_spacing), x_start_(x_start),x_end_(x_end),
          y_start_(y_start),y_end_(y_end),num_of_orient_(num_of_orient),
          margin_to_wall_(margin_to_wall) {
        if (num_of_orient != 1 && num_of_orient != 4 && num_of_orient != 8) {
            cout << "ERROR: Graph::Graph -> In graph constructor, passed number of orientations should be 4, or 8. Others are not supported." << endl;
            throw;
        }
        GenerateGraph(map);
    };

    void GenerateGraph(const vector_map::VectorMap& map);

    std::list<GraphIndex> GetVertexNeighbors(const GraphIndex& index);
    //std::list<GraphIndex> GetVertexNeigbors(const navigation::PoseSE2& pose);

    GraphIndex GetClosestVertex(const navigation::PoseSE2& pose);
    Eigen::Vector2f GetLocFromVertexIndex(int index_x, int index_y);
    void AddWallToGraph(const geometry::line2f&  line){};

    const Vertices& GetVertices() const {
        return vertices_;
    }



private:
    float NormalizeAngle(float angle);
    list<GraphIndex> removeEdgesWithObstacles(const GraphIndex& vertex,
                                 const list<GraphIndex>& neighbors,
                                 const vector_map::VectorMap& map);
    void GetConnectedNeighbors(const GraphIndex& orient,
                                list<GraphIndex>& neigbors);
    bool checkEdgeForObstacles(geometry::line2f& edge,
                               const vector_map::VectorMap& map);
private:

    float grid_spacing_;
    int x_start_;
    int x_end_;
    int y_start_;
    int y_end_;
    int num_of_orient_;
    float margin_to_wall_;

    Vertices vertices_;
    int num_vertices_x_;
    int num_vertices_y_;
};

}
#endif /* SRC_NAVIGATION_GLOBAL_PLANNER_ */

