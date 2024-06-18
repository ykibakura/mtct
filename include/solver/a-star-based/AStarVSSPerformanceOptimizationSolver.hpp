#pragma once
#include "probleminstances/GeneralPerformanceOptimizationInstance.hpp"
#include "solver/GeneralSolver.hpp"

#include <filesystem>
#include <string>
#include <utility>

namespace cda_rail::solver::astar_based {
  class AStarVSSPerformanceOptimizationSolver
      : public GeneralSolver<
            instances::GeneralPerformanceOptimizationInstance,
            instances::SolVSSGeneralPerformanceOptimizationInstance<
                instances::GeneralPerformanceOptimizationInstance>> {
  private:
    // TODO: Implement
    //instance variables
    int dt = -1;
    size_t num_t = 0;
    size_t num_tr =0;
    size_t num_edges = 0;
    size_t num_vertices = 0;

    /////////////////////////
    //train_pos, train_speed, train_routed defined in probleminstance Line 262
    ////////////////////////
    struct InitialState
    {
      train_pos =

      /////////////
      ///////////////

    };


  public:
    // Constructors. TODO: Implement
    explicit AStarVSSPerformanceOptimizationSolver(
        const instances::GeneralPerformanceOptimizationInstance& instance);
    explicit AStarVSSPerformanceOptimizationSolver(
        const std::filesystem::path& p);
    explicit AStarVSSPerformanceOptimizationSolver(const std::string& path);
    explicit AStarVSSPer

    formanceOptimizationSolver(const char* path);

    // TODO: Implement missing functions

    using GeneralSolver::solve;
    [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance<
        instances::GeneralPerformanceOptimizationInstance>
    solve(int time_limit, bool debug_input) override;
  };
} // namespace cda_rail::solver::astar_based
