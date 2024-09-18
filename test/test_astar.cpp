#include "datastructure/RailwayNetwork.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "solver/GeneralSolver.hpp"
#include "solver/a-star-based/AStarVSSPerformanceOptimizationSolver.hpp"

#include "gtest/gtest.h"
#include <algorithm>
#include <optional>

using namespace cda_rail::solver::astar_based;


// for initial_state
TEST(AStarVSSPerformanceOptimizationSolverTest, InitialStateTest) {
  const size_t expected_number_of_trains = 3;

  AStarVSSPerformanceOptimizationSolver solver("./example-networks/SimpleStation/");
  //solver.solve(...)
  AStarVSSPerformanceOptimizationSolver::TrainState state(solver.get_instance().get_train_list().size(), 0.0, 15.0, 0.0, solver.get_instance().const_n().number_of_edges());

  solver.update_state(state);  // Use an object to call the member function

  EXPECT_EQ(expected_number_of_trains, solver.get_instance().get_train_list().size());
  EXPECT_EQ(state.num_tr.size(), expected_number_of_trains);

  // Verify initialization for each train
  for (size_t i = 0; i < state.num_tr.size(); ++i) {
    EXPECT_EQ(state.num_tr[i].current_pos, 0);
    EXPECT_EQ(state.num_tr[i].prev_pos, 0);

    EXPECT_GE(state.num_tr[i].entry_vertex, 0); // non-negative, value>=0
    EXPECT_GE(state.num_tr[i].exit_vertex, 0);
    EXPECT_GE(state.num_tr[i].entry_edge, 0);
    EXPECT_GE(state.num_tr[i].current_edge, 0);
    EXPECT_GE(state.num_tr[i].exit_edge, 0);

    EXPECT_TRUE(state.num_tr[i].routed_edges.empty());
    EXPECT_TRUE(state.num_tr[i].routed_edges_current.empty());
  }

  // Check that other members are initialized correctly
  EXPECT_EQ(state.t, 0.0);
  EXPECT_GE(state.delta_t, 15.0);
  EXPECT_EQ(state.counter, 0);
  EXPECT_GE(state.cost, 0.0);

  // Verify that edge_vss is properly resized and cleared
  EXPECT_EQ(state.edge_vss.size(), solver.get_instance().const_n().number_of_edges());
  EXPECT_TRUE(state.edge_vss.empty());
}

// Test to check if the solver can correctly identify the goal state
/*TEST(AStarVSSPerformanceOptimizationSolverTest, GoalStatetest) {
  AStarVSSPerformanceOptimizationSolver::TrainState state;

  state.num_tr[0].current_pos = 200;
  state.num_tr[0].goal_pos = 200;
  state.num_tr[]



}*/


