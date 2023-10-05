#ifndef __GRAPH_OPTIMIZATION_H__
#define __GRAPH_OPTIMIZATION_H__
#include <iostream>
#include <string>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/concepts.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
using namespace gtsam;

namespace factorgraph_opt
{
  Matrix6 Adj(Pose3 T)
  {
    Matrix3 R = T.rotation().matrix();
    Matrix3 A = skewSymmetric(T.x(), T.y(), T.z()) * R;
    Matrix6 adj;
    adj.block(0, 0, 3, 3) = R;
    adj.block(3, 3, 3, 3) = R;
    adj.block(0, 3, 3, 3) = A;
    adj.block(3, 0, 3, 3) = Matrix3::Zero(3, 3);
    return adj;
  }

  // ternary factor defination:
  class BTW3Factor : public NoiseModelFactor3<Pose3, Pose3, Pose3>
  {
  private:
    // measurement information
    Pose3 T_;

  public:
    /**
     * Constructor
     * @param poseKey    associated pose varible key
     * @param model      noise model
     * @param T          Point2 measurement
     */
    BTW3Factor(Key poseKey1, Key poseKey2, Key poseKey3, const Pose3 T, SharedNoiseModel model) : NoiseModelFactor3<Pose3, Pose3, Pose3>(model, poseKey1, poseKey2, poseKey3), T_(T) {}

    // error function defination:
    Vector evaluateError(const Pose3 &p1, const Pose3 &p2, const Pose3 &p3, boost::optional<Matrix &> H1 = boost::none, boost::optional<Matrix &> H2 = boost::none, boost::optional<Matrix &> H3 = boost::none) const
    {
      Pose3 error_ = p1.inverse() * p2 * p3.inverse();
      Matrix6 J;
      Vector6 log = Pose3::Logmap(error_);
      Vector3 w{0, 0, 0};
      w << log(0), log(1), log(2);
      Vector3 t{0, 0, 0};
      t << log(3), log(4), log(5);
      J.block(0, 0, 3, 3) = skewSymmetric(log(0), log(1), log(2));
      J.block(0, 3, 3, 3) = skewSymmetric(log(3), log(4), log(5));
      J.block(3, 0, 3, 3) = Matrix3::Zero(3, 3);
      J.block(3, 3, 3, 3) = skewSymmetric(log(0), log(1), log(2));
      J = J * 0.5 + Matrix6::Identity();
      if (H1)
        *H1 = ((-1) * J * Adj((p2 * p3.inverse()).inverse()));
      if (H2)
        *H2 = (J * Adj((p2 * p3.inverse()).inverse()));
      if (H3)
        *H3 = ((-1) * J);
      return log; // return error vector
    }
  };

  class FactorGraph
  {
  public:
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values result;
    noiseModel::Gaussian::shared_ptr updatenoise;
    int f_id = 0;

    void AddBTW2factor(int id1, int id2, Pose3 p, double noiseScore)
    {
      Vector Vector6(6);
      Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
      noiseModel::Diagonal::shared_ptr Noise = noiseModel::Diagonal::Variances(Vector6);
      gtSAMgraph.add(BetweenFactor<Pose3>(id1, id2, p, Noise));
      f_id++;
    }

    void AddBTW3factor(int id1, int id2, int id3, double noiseScore)
    {
      Pose3 p(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
      Vector Vector6(6);
      Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
      noiseModel::Diagonal::shared_ptr Noise = noiseModel::Diagonal::Variances(Vector6);
      gtSAMgraph.add(BTW3Factor(id1, id2, id3, p, Noise));
      f_id++;
    }

    void AddPriorFactor(int id, Pose3 p, double noiseScore)
    {
      Vector Vector6(6);
      Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
      noiseModel::Diagonal::shared_ptr Noise = noiseModel::Diagonal::Variances(Vector6);
      gtSAMgraph.add(PriorFactor<Pose3>(id, p, Noise));
      f_id++;
    }

    void AddMarginalPrior(int id, Pose3 p)
    {
      Marginals marginals(gtSAMgraph, result);
      updatenoise = noiseModel::Gaussian::Covariance(marginals.marginalCovariance(id));
      gtSAMgraph.add(PriorFactor<Pose3>(id, p, updatenoise));
      f_id++;
    }

    void setinitialEstimate(int id, Pose3 p)
    {
      initialEstimate.insert(id, p);
    }

    void Removefactor(int id)
    {
      gtSAMgraph.remove(id);
    }

    void StartOptimiz(int iterations)
    {
      LevenbergMarquardtParams parameters;
      parameters.maxIterations = iterations;
      LevenbergMarquardtOptimizer optimizer(gtSAMgraph, initialEstimate, parameters);
      result = optimizer.optimize();
    }
  };
}
#endif
