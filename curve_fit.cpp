#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include "curve_fit.h"
#include <gtsam/inference/FactorGraph.h>
using namespace std;

int main()
{
    vector<double> t = {0, 0.1, 0.2, 0.3, 0.4};
    vector<double> x = {19.2361, 19.0276, 18.8659, 18.5314, 18.3536};
    CurveFit x_t_fit;
    x_t_fit.Fitting(t, x);
    std::vector<double> abcd = x_t_fit.GetParams();
    cout << "abcd: " << abcd[0] << " " << abcd[1] << " " << abcd[2] << " " << abcd[3] << endl;
    cout << "t = 0.5, pre_x: = " << x_t_fit.Predict(0.5) << endl;
    return 0;
}
