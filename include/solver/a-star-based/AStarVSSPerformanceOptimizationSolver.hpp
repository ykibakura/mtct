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
    struct train_state
    {
      //Define state properties

      std::vector<int> train_pos;
      int pos;
      std::vector<int> train_speed;
      std::vector<bool> train_routed;
      int n;

      ///////////////
      //import the properties for the train for the initial state
      ///////////////

    };



    train_state initial_state() {
      //Initialize and return the initial state
      train_state initial_state;
      initial_state.train_pos = std::vector<int>(num_tr, pos); //pos?
      initial_state.n = 0;
      return initial_state;
    }

    bool goal_state(train_state& other) {
      bool operator == (train_state& other) const{
        return goal_state,train_pos = std::vector<int>(num_tr, pos);
      }
      //change pos to other value!!!
      //write condition

    }


    //functions to check the best option from A-star


  public:
    // Constructors. TODO: Implement
    explicit AStarVSSPerformanceOptimizationSolver(
        const instances::GeneralPerformanceOptimizationInstance& instance);
    //namespace "instances": GeneralPerformance... Line 21
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
