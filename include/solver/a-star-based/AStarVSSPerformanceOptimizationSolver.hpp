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
    // instance variables
    int    dt    = -1;
    size_t num_t = 0; // no need? 7.7.24
    // std::vector<int> num_tr; //struct Train has Train.name but not vector! needed
    size_t num_edges    = 0; // no need? 7.7.24
    size_t num_vertices = 0; // no need? 7.7.24

    /////////////////////////
    // train_pos, train_speed, train_routed defined in probleminstance GeneralPerformanceOptim Line 262?
    ////////////////////////

    struct train_state {
      // Define state properties

      std::vector<Train> train_current; // train properties
      // std::vector<Train> train_previous;
      std::vector<int>  train_pos_current; // train_pos vector
      std::vector<Edge> train_edges;       // which edge is the train at
      // std::vector<int> train_pos_previous;
      int              time;   // time stamp t0,t1 etc
      std::vector<int> num_tr; // struct Train has Train.name but not vector!
      // TODO: include cost, prev status, VSS info

      //////////////// TO DELETE ////////
      std::vector<int>  train_pos;
      int               pos;
      std::vector<int>  train_speed;
      std::vector<bool> train_routed;
      int               n;
      std::vector<int>  start_pos;
      std::vector<int>  goal_pos;
      ///////////////////////////

      ///////////////
      // import the properties for the train for the initial state
      ///////////////
    };

    train_state initial_state() {
      // initialises and return the initial state
      train_state initial_state;
      initial_state.train_pos    = std::vector<int>(num_tr, start_pos);
      initial_state.train_speed  = std::vector<int>(num_tr, 0);
      initial_state.train_routed = std::vector<int>(num_tr, false);
      // start_pos is the vector<num_tr, startpos_of_train>
      initial_state.n = 0;
      return initial_state;
    }

    bool goal_state(train_state& other) {
      // checks if all train_pos match with the pre-defined goal_pos
      bool operator==(train_state& state) const {
        return state.train_pos == state.goal_pos
      }
      // start_pos is the vector<num_tr, startpos_of_train>
      // goal_pos is the vector<num_tr, goalpos_of_train>
      // goal_state, when start_pos=goal_pos
    }

    // TODO: heuristic function

    // TODO: cost function

    // TODO: successor function

    // used in pot_collision_check
    bool collision_vss_check(const Edge& edge, const TrainList& tr_list, int tr1,
                             int tr2) {
      // used in pot_collision_check
      // when two trains are in the same TDD, then it has to be checked if they collide
      const auto tr1        = tr_list.get_train(tr1); // get list of tr1
      const auto tr1_length = tr1.length;             // length tr1
      int        tr1_start  = train_state.train_pos_current[tr1]; // start
      int        tr1_end =
          tr1 +
          tr1_length; // end: ATTENTION: FOR IMPLEMENTATION ADD DISTANCE TRAVELED!
      ///////TODO: work: add distance traveled, check if + is correct
      ///////////////////////////////////////////////////////////
      const auto tr2        = tr_list.get_train(tr2);
      const auto tr2_length = tr2.length;
      int        tr2_start  = train_state.train_pos_current[tr2];
      int        tr2_end    = tr2 + tr2_length;

      // VERGLEICHE DIE tr1_length & tr2_length:
      //(tr1_start, tr1_end) is section occupied
      if ((tr1_start < tr2_end && tr1_end > tr2_start) ||
          (tr1_start > tr2_end && tr1_end < tr2_start)) {
        // always compare tr1_start,tr2_end && tr1_end,tr2_start. depends on the positive direction: <>either way
        return true; // collision. if false: VSS can solve the problem
      }
    }

    // potential collision check function - checks if multiple trains exist in a TDD
    bool pot_collision_check(const TrainState& train_s) const {
      int collision = 0; // default collision = 0. when collision detected: =1

      for (size_t i = 0; i < num_tr; ++i) {
        ////////////TODO: check if train_pos is correct defined
        //////////////////////////////////////////////////
        int         train_pos = train_state.train_pos_current[i];
        const Edge& edge      = train_state.train_edges[i];
        int         counter;
        // if for any two trains, if the edge is the same, then further search for position.
        //->first, edge check then position.
        for (size_t j = i + 1; j < num_tr; ++j) {
          if (train_state.train_edges[j] == edge) {
            int pot_collision = 1;
            if (collision_vss_check(edge, tr_list, i, j) ==
                1) { // checking if collision or VSS-situation
              /////////TODO: HOW TO DEFINE TrainList???
              //////////////////////////////////////////
              collision = 1; // collision happens
              break;         // collision detected. not valid successor. break
            }
          }
          ///////// TODO: edge with other direction is not yet considered!!!
          //////////////////////////////////////////////////////////////
        }
      }

      return true;
      ////////TODO: eliminate the successor if collision=1
      //////////////////////////////////////////////////////
    }

    // TODO: priority queue
  }


  public:
    // Constructors. TODO: Implement
    explicit AStarVSSPerformanceOptimizationSolver(
        const instances::GeneralPerformanceOptimizationInstance& instance)
        : GeneralSolver(instance) {
          num_t = instance.get_num_t();
          num_tr = instance.get_num_tr();
          num_edges = instance.get_num_edges();
          num_vertices = instance.get_num_vertices();


          const auto tr_list = instance.get_train_list();
          const auto numtr = tr_list.size();
          for (size_t i = 0; i < numtr; i++) {
            const auto tr = tr_list.get_train(i);
            const auto trl = tr.length;

            const auto trs = instance.get_schedule(i);
            const auto entry_vertex = trs.get_entry();

            const auto network = instance.const_n();
            const auto v0 = network.get_edge(0).source;
            const auto v1 = network.get_edge(0).target;
            // Edge 0 = v0 -> v1

            const auto er = instance.const_n().get_reverse_edge_index(0);
            if (er.has_value()) {
              //er.value() ist index von v1 -> v0
            } else {
              // v1 -> v0 existiert nicht
            }

            // Zwei Importvarianten
            // SimpleStation ist sehr überschaubar
            // -------------------------------
            //      \--------------/
            const auto instance_imported = cda_rail::instances::GeneralPerformanceOptimizationInstance("path_to_instance_example_gen-po");
            const auto instance_imported_2_vss = cda_rail::instances::VSSGenerationTimetable("path_to_example-network");
            const auto instance_imported_2 = cda_rail::instances::GeneralPerformanceOptimizationInstance::cast_from_vss_generation(instance_imported_2_vss);


            // Etwas ganz zum Schluss
            bool tr_has_route = instance.has_route(tr.name); // true wenn die Kanten, die tr nutzt bereits festgelegt sind
            // Aber, die sollten ignoriert werden
            const auto tr_route = instance.get_route(tr.name);

            // Wichtiger am Ende in der Lösung. Im Lösungsobjekt:
            // sol.reset_routes();
            // Für jeden Zug: .add_empty_route(...) und .push_back_edge_to_route(...)

          }
        }
    explicit AStarVSSPerformanceOptimizationSolver(
        const std::filesystem::path& p)
        : GeneralSolver(p) {
          //
        }
    explicit AStarVSSPerformanceOptimizationSolver(const std::string& path);
    explicit AStarVSSPerformanceOptimizationSolver(const char* path);

    // TODO: Implement missing functions

    using GeneralSolver::solve;
    [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance<
        instances::GeneralPerformanceOptimizationInstance>
    solve(int time_limit, bool debug_input) override;
  };
} // namespace cda_rail::solver::astar_based
