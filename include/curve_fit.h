#include <gtsam/base/Vector.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
// #include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
using namespace gtsam;
class curvfitFactor : public gtsam::NoiseModelFactor1<gtsam::Vector4>
{
    double mt, mxy;

public:
    curvfitFactor(gtsam::Key key, double t, double xy, gtsam::SharedNoiseModel model)
        : gtsam::NoiseModelFactor1<gtsam::Vector4>(model, key), mt(t), mxy(xy) {}

    virtual ~curvfitFactor()
    {
    }
    gtsam::Vector evaluateError(const gtsam::Vector4 &abcd, boost::optional<gtsam::Matrix &> H = boost::none) const
    {
        auto val = abcd[0] * mt * mt * mt + abcd[1] * mt * mt + abcd[2] * mt + abcd[3];
        if (H)
        {
            gtsam::Matrix Jac = gtsam::Matrix::Zero(1, 4);
            Jac << mt * mt * mt, mt * mt, mt, 1;
            (*H) = Jac;
        }
        return gtsam::Vector1(val - mxy);
    }
};

class CurveFit
{
private:
    double abcd[4] = {0, 0, 0, 0}; // parameters to be estimated
    NonlinearFactorGraph graph;
    Values initial;
    Values results;
    gtsam::noiseModel::Gaussian::shared_ptr updatenoise;

public:
    void Fitting(std::vector<double> t, std::vector<double> xy)
    {
        double noiseScore = 1e-4;
        gtsam::Vector Vector1(1);
        Vector1 << noiseScore;
        noiseModel::Diagonal::shared_ptr Noise = noiseModel::Diagonal::Variances(Vector1);
        if (t.size() == xy.size())
        {
            for (int i = 0; i < xy.size(); i++)
            {
                graph.emplace_shared<curvfitFactor>(0,t[i], xy[i], Noise);  
            }
            initial.insert(0, gtsam::Vector4(0.0, 0.0, 0.0, 0.0));
        }
        else
        {
            std::cout << "data error!" << std::endl;
        }
        gtsam::LevenbergMarquardtParams parameters;
        parameters.maxIterations = 30;
        gtsam::LevenbergMarquardtOptimizer opt(graph, initial, parameters);
        results = opt.optimize();
        // results.print("final res:");
        gtsam::Vector4 v = results.at<gtsam::Vector4>(0);
        for (int i = 0; i < 4; i++)
            abcd[i] = v[i];
    }
    double Predict(double t)
    {
        return abcd[0] * t * t * t + abcd[1] * t * t + abcd[2] * t +
               abcd[3];
    }

    std::vector<double> GetParams()
    {
        return std::vector<double>(abcd, abcd + 4);
    }
};
