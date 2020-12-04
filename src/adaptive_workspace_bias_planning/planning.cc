/*
 * planning.cc
 *
 *  Created on: Dec 3, 2020
 *      Author: nathannguyen
 */


#include <cmath>
#include <iterator>
#include <shared/global_utils.h>
#include "planning.h"
#include "vector_map/vector_map.h"
#include "shared/math/line2d.h"


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
}