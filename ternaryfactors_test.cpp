#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include "factorgraph_opt.h"
#include <gtsam/inference/FactorGraph.h>

using namespace std;
using namespace gtsam;

double egoP_egoP = 1e-6;
double egoP_objP = 1e-2;
double objP_objP_chgP = 1.0;
double chgP_chgP = 1e-2;
int main(int argc, char **argv)
{
  factorgraph_opt::FactorGraph local_graph;
  // add prior factors
  local_graph.AddPriorFactor(0, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 0, 0)), egoP_egoP);
  local_graph.AddPriorFactor(1, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, -1, 0)), egoP_egoP);
  local_graph.AddPriorFactor(4, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(1, 0, 0)), egoP_egoP);
  // add btw2 factors
  local_graph.AddBTW2factor(0, 2, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(2, 0, 0)), egoP_egoP);
  local_graph.AddBTW2factor(2, 5, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(2, 0, 0)), egoP_egoP);
  local_graph.AddBTW2factor(0, 1, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, -1, 0)), egoP_objP);
  local_graph.AddBTW2factor(2, 3, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(-1, -1, 0)), egoP_objP);
  local_graph.AddBTW2factor(5, 6, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(-2, -1, 0)), egoP_egoP);
  local_graph.AddBTW2factor(4, 7, Pose3(Rot3::RzRyRx(0, 0, 0), Point3(0, 0, 0)), chgP_chgP);
  // add btw3 factors
  local_graph.AddBTW3factor(1, 3, 4, objP_objP_chgP); 
  local_graph.AddBTW3factor(3, 6, 7, objP_objP_chgP); 
  
  // set initial value
  // For illustrative purposes, some have been deliberately set to incorrect values
  local_graph.setinitialEstimate(0, Pose3(Rot3::RzRyRx(0.1, 0, 0), Point3(0.1, -0.1, 0)));
  local_graph.setinitialEstimate(1, Pose3(Rot3::RzRyRx(-0.1, 0, 0), Point3(-0.1, -1.1, 0)));
  local_graph.setinitialEstimate(2, Pose3(Rot3::RzRyRx(0.1, 0, 0), Point3(2.2, 0.2, 0)));
  local_graph.setinitialEstimate(3, Pose3(Rot3::RzRyRx(-0.1, 0, 0), Point3(-1.1, -0.9, 0))); // Given the wrong initial value, purposely
  local_graph.setinitialEstimate(4, Pose3(Rot3::RzRyRx(0.1, 0, 0), Point3(0.9, 0.15, 0)));
  local_graph.setinitialEstimate(5, Pose3(Rot3::RzRyRx(-0.1, 0, 0), Point3(4.2, -0.2, 0)));
  local_graph.setinitialEstimate(6, Pose3(Rot3::RzRyRx(0.1, 0, 0), Point3(-2.1, -1.0, 0))); // Given the wrong initial value, purposely
  local_graph.setinitialEstimate(7, Pose3(Rot3::RzRyRx(-0.1, 0, 0), Point3(0.05, -0.05, 0)));

  // start optimize
  local_graph.StartOptimiz(30);
  // print graph
  local_graph.gtSAMgraph.print("\nFactor Graph:\n"); // print
  cout << "-----------------------------------------------------" << endl;
  // print result
  local_graph.result.print("Final Result:\n");
  return 0;
}
