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
      std::vector<int> start_pos;
      std::vector<int> goal_pos;

      ///////////////
      //import the properties for the train for the initial state
      ///////////////

    };



    train_state initial_state() {
      //initialises and return the initial state
      train_state initial_state;
      initial_state.train_pos = std::vector<int>(num_tr, start_pos);
      //start_pos is the vector<num_tr, startpos_of_train>
      initial_state.n = 0;
      return initial_state;
    }

    bool goal_state(train_state& other) {
      //checks if all train_pos match with the pre-defined goal_pos
      bool operator == (train_state& other) const{
        return goal_state,train_pos == std::vector<int>(num_tr, goal_pos);
      }
      //goal_pos is the vector<num_tr, goalpos_of_train>
      //write condition

    }


    //functions to check the best option from A-star


  public:
    // Constructors. TODO: Implement
    explicit AStarVSSPerformanceOptimizationSolver(
        const instances::GeneralPerformanceOptimizationInstance& instance)
        : GeneralSolver(instance) {
          num_t = instance.get_num_t();
          num_tr = instance.get_num_tr();
          num_edges = instance.get_num_edges();
          num_vertices = instance.get_num_vertices();
        }
    explicit AStarVSSPerformanceOptimizationSolver(
        const std::filesystem::path& p)
        : GeneralSolver(p) {
          //
        }
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
