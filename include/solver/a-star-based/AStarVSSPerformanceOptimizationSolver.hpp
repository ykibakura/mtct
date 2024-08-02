#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "solver/GeneralSolver.hpp"

#include <filesystem>
#include <string>
#include <utility>


namespace cda_rail::solver::astar_based {

  struct CompareTrainState {
    bool operator()(const TrainState& t1, const TrainState& t2) {
      // COM ADDED: Compare based on cost, lower cost has higher priority
      return t1.cost > t2.cost;
    }
  };

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

    /*
     * 29.7 Meeting
     * out_edges()<vector>:bei initial_state, um von der Startknoten die erste Kante zu finden.
     * in_edges()<vector>:bei goal_state, um von der Exitknoten die letzte Kante zu finden.
     * get_successors()<vector>:von Kante zu nextmögliche Kante zu finden: bei successors() gut
     * all_paths_of_length_starting_in_edge()<vector>:finde alle mögliche Wege für bestimmten Länge
     * (neibouring_ecges()<vector>:finde alle verbundene Kanten zu einem Knoten
     * */

    // priority queue for managing TrainState objects
    std::priority_queue<TrainState, std::vector<TrainState>, CompareTrainState> pq;


    struct Properties { //function of all the properties needed for tr_state etc
      // Train train; // train properties defined in train.hpp
      // Edge train_edge; // current edge
      bool VSS; // VSS info

      double current_pos; // Current position

      int entry_vertex; // entry vertex
      int exit_vertex; // goal vertex

      int entry_edge;
      int current_edge; // current edge
      int exit_edge;
      // TODO: use index?
    };


    struct TrainState {
      std::vector<Properties> num_tr; //vector for every train: properties
      int t; // time
      int delta_t = 1; // delta t
      int counter; // counter: how many times the state is updated
      double cost;

      // Constructor
      TrainState(size_t n): num_tr(n), t(0), counter(0), { // constructor. TODO:// state update time can be changed here!!!
      }

    };


    // TODO: BETTER VERSION INITIAL STATE
    TrainState initial_state(TrainState& tr_state, const TrainList& tr_list, const Network& network, const GeneralSolver& instance) {
      size_t n = tr_list.size(); // n here is local variable for initial_state
      tr_state.cost = 0;

      for (size_t i = 0; i < n; ++i) {
        const Schedule tr_schedule = instance.get_schedule(i);
        // variables:cost,vss,current_pos,entry/exit_vertex,entry/current/exit_edge
        tr_state.num_tr[i].VSS = false;

        tr_state.num_tr[i].current_pos = 0; // assuming starting point is always on vertices

        tr_state.num_tr[i].entry_vertex = tr_schedule.get_entry(); // entry vertex index: start pos initialised.
        tr_state.num_tr[i].exit_vertex = tr_schedule.get_exit(); // exit vertex

        tr_state.num_tr[i].entry_edge = network.out_edges(tr_state.num_tr[i].entry_vertex); // entry edge
        tr_state.num_tr[i].current_edge = tr_state.num_tr[i].entry_edge; // current edge
        tr_state.num_tr[i].exit_edge = network.in_edges(tr_state.num_tr[i].exit_vertex); // exit edge

        // TODO: evtl add speed at t=0?
        // TODO: prev status
      }
      return tr_state;
    }


    // TODO: BETTER VERSION GOAL STATE
    bool goal_state(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
      size_t n = tr_state.num_tr.size(); // n is local variable for goal_state. get size from tr_state
      double exitedge_len;

      for (size_t i = 0; i < n; ++i) { // i: train index
        // tr_state.num_tr[i].exit_vertex = tr_schedule.get_exit(); //get exit vertex WURDE IN INI_STATE GEMACHT
        exitedge_len = network.get_edge(tr_state.num_tr[i].exit_edge).length;

        if (tr_state.num_tr[i].exit_vertex != network.get_edge(tr_state.num_tr[i].exit_edge).target || tr_state.num_tr[i].current_pos < exitedge_len + tr_list.get_train(i).length) {
          // if train is NOT at exit_edge OR current_pos<(edge.length+tr.length(___._[]_>))
          return false;
        }
      }
      return true;
    }


    // TODO: make fucntion: tr_state update_state
    // Previous state?
    TrainState update_state(TrainState& tr_state, const TrainList& tr_list, const Network& network) {
      // 1.find successors 2.check collision,vss 3.check cost
      tr_state.counter++; // for each state, counter will be added. start=0->1->2->3->...
      tr_state.t += tr_state.delta_t;

      std::vector<TrainState> next_states = successors(tr_state, tr_list, network);

      for (size_t i = 0; i < next_states.size(); ++i) { // for loop for every path
        if (pot_collision_check(next_states[i], tr_list) == 1) { // collision
          // TODO: Delete from potential successors
        }
        else { // no collision
          next_states[i].cost = cost(next_states[i], tr_list, network);
          pq.push(next_states[i]); // Add the valid state to the priority queue
        }

      }
      return next_states;
    }


    // TODO: priority queue
    struct PriorityQueue {
      bool operator()(const TrainState& t1, const TrainState& t2) {
        // COM ADDED: Compare based on cost, lower cost has higher priority
        return t1.cost > t2.cost;
      }
    };

    // TODO: heuristic function
    // USE all_edge_pairs_shortest_paths() by RailwayNetwork.hpp L543
    // h(t)=h(t)=SIGMA(h_idx)=SIGMA(d/s), where d is the shortest path to Ziel, s is max velocity
    // d=shortest_path(size_t source_edge_id,size_t target_vertex_id)+distance to next Knoten
    // ACHTUNG:beachte shortest_path von nächstmögliche Knoten?
    double heuristic(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
      double h = 0; // heuristic wert
      double d = 0; // distance
      size_t n = tr_state.num_tr.size(); // n is local variable for goal_state. get size from tr_state

      for (size_t i = 0; i < n; ++i) { // nächstmögliche Knoten,Länge bis da,Endknoten nötig!
        int current_edge = tr_state.num_tr[i].current_edge;
        int exit_vertex = tr_state.num_tr[i].exit_vertex; // ENDKNOTEN

        //const auto v_next = network.get_edge(edge).target; // Nächstmögliche Knoten

        double d = network.shortest_path(current_edge, exit_vertex); // shortest path TODO:WHY NOT WORKING?
        double l_to_v_next = network.get_edge(current_edge).length - tr_state.num_tr[i].current_pos; // LÄNGE BIS DA
        d += l_to_v_next; // add length to nearest vertex
        double h_index = d / tr_list.get_train(i).max_speed;
        h += h_index;
        d = 0;
        h_index = 0;// reset d=0 for next index
        // repeat for the next index(h will be continuously added)

        // TODO:Maybe implement cost fkt here? Want to save cost to tr_state: tr_state.num_tr[i].cost=f->in for-Schleife
      }

      return h;  // total h
      // TODO:evtl use h[i] and add together at the end?
    }


    // TODO: cost function
    double cost(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
      size_t n = tr_state.num_tr.size();
      double g = 0; // SIGMA(g_index)
      double g_index = 0; // cost till current state

      for (size_t i = 0; i < n; ++i) {
        g_index = tr_list.get_train(i).max_speed * tr_state.t; // sum of total time each train has travelled
        g += g_index;
        // TODO: max_speed can evtl not be used, since max_speed edge is different for each Kanten
      }
      double f = g + heuristic(tr_state, tr_list, network); // f=g+h
      return f;
    }


    // TODO: successor function
    std::vector<TrainState> successors(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
      size_t n = tr_state.num_tr.size();
      std::vector<TrainState> successor_state; //TODO: work on return value, Now it is wrong

      for (size_t i = 0; i < n; ++i) {
        // for all trains, they move to next point by the max speed
        double total_length = tr_list.get_train(i).max_speed * tr_state.delta_t;
        double remain_length = total_length;
        TrainState new_state = tr_state; // copies the current state

        std::vector<std::vector<size_t>> paths = network.all_paths_of_length_starting_in_edge(tr_state.num_tr[i].current_edge, tr_list.get_train(i).max_speed * tr_state.t + tr_state.num_tr[i].current_pos, tr_state.num_tr[i].exit_edge);
        // get the vector of all routes from prev to current state, Bsp {{1,2,3},{1,2,4}}. **length: from pos0 of the edge!
        size_t m = paths.size();

        for (size_t j = 0; j < m; ++j) { // [j] shows for each possible path. j is index for new_state[j].num_tr...
          size_t l = paths[j].size;

          for (size_t k = 0; k < l; ++k) { // looking at every possible path: for each Kantenindex:0-(l-1)
            remain_length -= network.get_edge(k).length; //remain_length always get subtracted by the edgelength
          }

          new_state.num_tr[i].current_edge = paths[j][l-1]; // for each possible path: the last edge is the current one
          new_state.num_tr[i].current_pos = network.get_edge(new_state.num_tr[i].current_edge).length + remain_length;
          // remain_length=-(distance of current edge which is not yet travelled)
          remain_length = total_length; //initialise again for next for-schleife
          successor_state.push_back(new_state); // add new state to successor_state
        }
        // get_successors(tr_state.num_tr[i].current_edge)

      }
      // cost:cost(),vss:vss(comes after pot_collision_check,
      // current_pos:HERE(),entry/exit_vertex:NO UPDATE,entry/current/exit_edge:current:HERE()

        /* tr_state.num_tr[i].current_vertex shows the current vertex.
         * 1. find current edge with current_vertex. (vertex.headway?)
         * 2. get length by ??? = network.get_edge(0).length;
         * 3. if max_speed*t<<edgelength:the train is in the same edge. only one possible option here
         * 4. else: all the next edges need to be checked. auto v_next = network.get_edge((currentedge)).target;
         * 5. check how many vertices(edges) are there: sizeof v_next
         * 6. get possible next edges, like in 1.
         * 7. check length
         * 8. so on...
         * */

      return successor_state;
    }


    // used in pot_collision_check
    bool collision_vss_check(const TrainState& tr_state, const TrainList& tr_list, int tr1_nr, int tr2_nr) {
      // used in pot_collision_check
      // when two trains are in the same TDD, then it has to be checked if they collide
      // TrainList is defined in train.hpp line 33

      double tr1_length = tr_list.get_train(tr1_nr).length;             // length tr1
      double tr1_start = tr_state.num_tr[tr1_nr].current_pos; // start
      double tr1_end = tr1_start + tr1_length; // end: ATTENTION: FOR IMPLEMENTATION ADD DISTANCE TRAVELED!
      // TODO: by tr1_end&tr2_end calculation: assumed theyre in the same edge - d.h. theyre in the same direction!
      // TODO: work:add distance traveled, check if + is correct->Anmerkung:only one edge is displayed!what if train goes more edges

      double tr2_length = tr_list.get_train(tr2_nr).length;
      double tr2_start = tr_state.num_tr[tr2_nr].current_pos;
      double tr2_end = tr2_start + tr2_length;

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
    bool pot_collision_check(const TrainState& tr_state, const TrainList& tr_list) {
      size_t n = tr_state.num_tr.size();

      for (size_t i = 0; i < n; ++i) { // if for any two trains, position further search if edge is the same
        // ->first, edge check then position.
        for (size_t j = i+1; j < n; ++j) {
          if (tr_state.num_tr[j].current_edge == tr_state.num_tr[i].current_edge) {
            // TODO: assumed theyre in the same edge!!! Not in the same TDD
            if (collision_vss_check(tr_state, tr_list, i, j) == 1) { // checking if collision or VSS-situation
              // TODO: eliminate the successor if collision=1
              return true; // collision detected. not valid successor. (break)
            }
            else {
              // else: VSS can solve the problem. Still remains as potential successor
            }
          }
        }
      }
      // TODO: implement TDD functions HERE OR BY successor()
      return false; //When no collision(potential VSS):return false
    }

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
