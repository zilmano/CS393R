#include "planning.h"

using Eigen::Vector2f;
using planning::FeatureCalc;

void test_fvr(const planning::AWBPlanner& planner) {
    cout << "Running fvr caluclation test."<< endl;
    const std::shared_ptr<FeatureCalc> feature_calc = planner.GetFeatureCalc();

    cout << "Generating Frv values..." << endl;
    feature_calc->GenerateFrvValues();
    cout << "Generating Frv Bitmap..." << endl;
    feature_calc->GenerateFrvBitmap();
}

int main(int argc, char** argv) {
    
    Vector2f map_x_bounds(-50,50);
    Vector2f map_y_bounds(0,36);
    float margin_to_wall = 0.1;
    float ws_grid_spacing = 0.05;

    planning::AWBPlanner sample_based_planner(
            map_x_bounds,
            map_y_bounds,
            margin_to_wall,
            ws_grid_spacing);

    sample_based_planner.LoadMap("GDC1");


    // Enter your test code here
    test_fvr(sample_based_planner);
    ////

    return 0;
}
