//
// Created by caochao on 7/12/19.
//

#ifndef VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H
#define VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H
#include <cmath>
#include <vector>
#include "ortools/constraint_solver/routing.h"
#include "ortools/constraint_solver/routing_enums.pb.h"
#include "ortools/constraint_solver/routing_index_manager.h"
#include "ortools/constraint_solver/routing_parameters.h"

using namespace operations_research;

namespace tsp_solver_ns
{
struct DataModel;
class TSPSolver;
void PrintSolution(const RoutingIndexManager& manager, const RoutingModel& routing, const Assignment& solution);
}  // namespace tsp_solver_ns

struct tsp_solver_ns::DataModel
{
  std::vector<std::vector<int>> distance_matrix;
  int num_vehicles = 1;
  RoutingIndexManager::NodeIndex depot{ 0 };
};

class tsp_solver_ns::TSPSolver
{
private:
  DataModel data_;
  std::unique_ptr<RoutingIndexManager> manager_;
  std::unique_ptr<RoutingModel> routing_;
  const Assignment* solution_;

public:
  TSPSolver(DataModel data);
  ~TSPSolver() = default;
  void Solve();
  void PrintSolution();
  int getComputationTime();
  void getSolutionNodeIndex(std::vector<int>& node_index, bool has_dummy);
  double getPathLength();
};

#endif  // VISUAL_COVERAGE_PLANNER_TSP_SOLVER_H
