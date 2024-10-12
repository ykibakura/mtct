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
  // TODO: Check on Heuristics
  const size_t expected_number_of_trains = 3;
  const size_t expected_number_of_edges = 22;

  AStarVSSPerformanceOptimizationSolver solver("/Users/yusuke/github/test/example-networks/SimpleStation");
  //solver.solve(...)
  AStarVSSPerformanceOptimizationSolver::TrainState state(solver.get_instance().get_train_list().size(), 0.0, 15.0, 0, 0.0, solver.get_instance().const_n().number_of_edges());

  // solver.update_state(state);  // Use an object to call the member function
  solver.initial_state(state); // initiate the trains

  EXPECT_EQ(expected_number_of_trains, solver.get_instance().get_train_list().size());
  EXPECT_EQ(expected_number_of_edges, solver.get_instance().const_n().number_of_edges());
  EXPECT_EQ(expected_number_of_trains, state.num_tr.size());
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
  EXPECT_EQ(state.delta_t, 15.0);
  EXPECT_EQ(state.counter, 0);
  EXPECT_GE(state.cost, 0.0);

  // Verify that edge_vss is properly resized and cleared
  EXPECT_EQ(state.edge_vss.size(), solver.get_instance().const_n().number_of_edges());
}

TEST(AStarVSSPerformanceOptimizationSolverTest, SuccessorTest) {
  AStarVSSPerformanceOptimizationSolver solver("/Users/yusuke/github/test/example-networks/SimpleStation");
  AStarVSSPerformanceOptimizationSolver::TrainState state(solver.get_instance().get_train_list().size(), 0.0, 30.0, 0, 0.0, solver.get_instance().const_n().number_of_edges());
  solver.initial_state(state);
  std::vector<AStarVSSPerformanceOptimizationSolver::TrainState> next_states = solver.successors(state);

  // EXPECT_EQ(next_states[1].t, 30);
  // EXPECT_EQ(next_states[1].counter, 1);
  // t, counter will be updated in update_state().
  EXPECT_GE(next_states[1].cost, 178.35563245423987);

  //tr3: 20m/s*30s=600m, edge 8(500m)->10(5m)->12(5m)->17(300m,at90m)
  EXPECT_EQ(next_states[1].num_tr[2].routed_edges[0], 8);
  EXPECT_EQ(next_states[1].num_tr[2].routed_edges[1], 10);
  EXPECT_EQ(next_states[1].num_tr[2].routed_edges[2], 12);
  EXPECT_EQ(next_states[1].num_tr[2].routed_edges[3], 17);
  EXPECT_EQ(next_states[1].num_tr[2].current_edge, 17);
  EXPECT_EQ(next_states[1].num_tr[2].current_pos, 90);

}
// UpdateState test
TEST(AStarVSSPerformanceOptimizationSolverTest, UpdateStateTest) {
  size_t expected_number_of_counter = 3;

  AStarVSSPerformanceOptimizationSolver solver("/Users/yusuke/github/test/example-networks/SimpleNetwork");
  AStarVSSPerformanceOptimizationSolver::TrainState state(solver.get_instance().get_train_list().size(), 0.0, 30.0, 0, 0.0, solver.get_instance().const_n().number_of_edges());
  solver.initial_state(state);
  solver.update_state(state);

  EXPECT_EQ(1, state.counter);
  EXPECT_EQ(30.0, state.t);

}

TEST(AStarVSSPerformanceOptimizationSolverTest, SolverTest) {
  AStarVSSPerformanceOptimizationSolver solver("/Users/yusuke/github/test/example-networks/SimpleNetwork");
  AStarVSSPerformanceOptimizationSolver::TrainState state(solver.get_instance().get_train_list().size(), 0.0, 60.0, 0, 0.0, solver.get_instance().const_n().number_of_edges());
  solver.initial_state(state);
  solver.solve(state);

  // EXPECT_EQ(1,1);
}


