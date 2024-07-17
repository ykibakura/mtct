#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "solver/GeneralSolver.hpp"

#include <filesystem>
#include <string>
#include <utility>

namespace cda_rail::solver::astar_based {
  class AStarVSSPerformanceOptimizationSolver
      : public GeneralSolver<
            instances::VSSGenerationTimetable,
            instances::SolVSSGenerationTimetable> {
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






    /////////////////////////
    // Knoten(vertex) is defined but doesnt mean theyre TDD, eg. junction
    ////////////////////////

    struct properties { //function of all the properties needed for train_state etc
      Train train; // train properties defined in train.hpp
      int train_pos_current; // Current position
      Edge train_edge; // current edge
      double cost; // Cost
      bool VSS; // VSS info
      // TODO: use index?
    };

    struct train_state {
      std::vector<properties> num_tr; //vector for every train: properties

      // Constructor
      train_state(size_t n): num_tr(n) { // constructor
      }

      // Define state properties
      // TODO: Do I need (const Edge& edge, const TrainList& tr_list)?
      // DEFINED: Train(std::string name, int length, double max_speed, double acceleration,
      //        double deceleration, bool tim = true) ******I need tr_list.get_train(index)???
      // for train_state: Train(), edge, time, start, end, prev_status, cost value, VSS

      /*std::vector<Train> train_current; // train properties
      // TODO: change to Train()?->for Train() is position not defined.
      // std::vector<Train> train_previous;
      std::vector<int>  train_pos_current; // train_pos vector
      std::vector<Edge> train_edge;       // which edge is the train at
      // std::vector<int> train_pos_previous;
      int              time;   // time stamp t0,t1 etc
      std::vector<int> num_tr; // struct Train has Train.name but not vector!
      // TODO: include cost, prev status, VSS info
      */
    };


    train_state initial_state(const TrainList& tr_list) { //TrainList is defined in train.hpp
      size_t n = tr_list.size(); // n here is local variable for initial_state
      train_state state(n);

      for (size_t i = 0; i < n; ++i) {
        state.num_tr[i].train = tr_list.get_train(i);
        state.num_tr[i].cost = 0;
        state.num_tr[i].VSS = false;
        // TODO: train_pos_current: defined in timetable.hpp? train_edge: defined in RailwayNetwork.hpp?
      }

      return state;
    }


    bool goal_state(const train_state& state) {
      size_t n = state.num_tr.size(); // n is local variable for goal_state. get size from train_state

      for (size_t i = 0; i < num_tr; ++i) {
        // TODO: if edge[i] is the same for goal->if pos[i] is same for goal->true. more like: if edge is not same: return false, else if pos is not same: return false
        // TODO: ALTERNATIV! End Knoten ist definiert. -> CHECK: "const auto exit_vertex = trs.get_exit();" by public
      }

      return true; // edge and pos same for all trains
    }

    // TODO: make fucntion: train_state update_state?????????????
    // Previous state?

    // TODO: heuristic function
    // USE all_edge_pairs_shortest_paths() by RailwayNetwork.hpp L543
    // h(t)=SIGMA(d/s), where d is the shortest path to Ziel, s is max velocity
    // d=shortest_path()+distance to next Knoten
    // ACHTUNG:beachte shortest_path von nächstmögliche Knoten?

    // TODO: cost function

    // TODO: successor function


    // used in pot_collision_check
    bool collision_vss_check(const train_state& state, const TrainList& tr_list, int tr1_nr, int tr2_nr) {
      // used in pot_collision_check
      // when two trains are in the same TDD, then it has to be checked if they collide
      // TrainList is defined in train.hpp line 33
      const auto tr1 = tr_list.get_train(tr1_nr); // get list of tr1
      const auto tr1_length = tr1.length;             // length tr1
      int        tr1_start = train_state.train_pos_current[tr1_nr]; // start
      int        tr1_end = tr1_start + tr1_length; // end: ATTENTION: FOR IMPLEMENTATION ADD DISTANCE TRAVELED!
      ///////TODO: work:add distance traveled, check if + is correct->Anmerkung:only one edge is displayed!what if train goes more edges

      ///////////////////////////////////////////////////////////
      const auto tr2 = tr_list.get_train(tr2_nr);
      const auto tr2_length = tr2.length;
      int        tr2_start = train_state.train_pos_current[tr2_nr];
      int        tr2_end = tr2_start + tr2_length;

      // VERGLEICHE DIE tr1_length & tr2_length:
      //(tr1_start, tr1_end) is section occupied
      if ((tr1_start < tr2_end && tr1_end > tr2_start) ||
          (tr1_start > tr2_end && tr1_end < tr2_start)) {
        // always compare tr1_start,tr2_end && tr1_end,tr2_start. depends on the positive direction: <>either way
        return true; // collision. if false: VSS can solve the problem
      }
      return false;
    }


    // potential collision check function - checks if multiple trains exist in a TDD
    bool pot_collision_check(const TrainState& train_state) const {
      int collision = 0; // default collision = 0. when collision detected: =1

      for (size_t i = 0; i < num_tr; ++i) {
       //int train_pos = train_state.train_pos_current[i]; NOT NEEDED!
        const Edge& edge = train_state.train_edge[i];

        // if for any two trains, position further search if edge is the same
        // ->first, edge check then position.
        for (size_t j = i + 1; j < num_tr; ++j) {
          if (train_state.train_edge[j] == edge) {
            int pot_collision = 1;
            if (collision_vss_check(edge, tr_list, i, j) == 1) {
              // checking if collision or VSS-situation
              /////////TODO: HOW TO DEFINE TrainList???
              //////////////////////////////////////////
              collision = 1; // collision happens
              return true; // collision detected. not valid successor. (break)
            }
          }
          ///////// TODO: edge with other direction is not yet considered!!!
          //////////////////////////////////////////////////////////////
        }
        return false; //When no collision(potential VSS):return false
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

            const Schedule trs = instance.get_schedule(i);
            const auto entry_vertex = trs.get_entry();
            const auto t_0 = trs.get_t_0(); // Zeit wenn Zug i bei entry_vertex auftaucht
            const auto v_0 = trs.get_v_0();

            const auto exit_vertex = trs.get_exit(); // Zielknoten, wo der Zug möglichst schnell ankommen will

            const Network network = instance.const_n();
            const auto v0 = network.get_edge(0).source;
            const auto v1 = network.get_edge(0).target;
            // Edge 0 = v0 -> v1

            network.get_edge(0).length;

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
            const auto instance_imported_vss = cda_rail::instances::VSSGenerationTimetable("path_to_example-network");


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
