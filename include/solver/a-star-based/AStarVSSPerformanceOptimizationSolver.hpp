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

  struct Properties { // function of all the properties needed for tr_state etc
    std::vector<size_t> routed_edges;         // edges travelled
    std::vector<size_t> routed_edges_current; // edges travelled current state

    double current_pos;  // Current position
    double prev_pos;     // position from previous state
    int    entry_vertex; // entry vertex
    int    exit_vertex;  // goal vertex
    int    entry_edge;
    int    current_edge; // current edge
    int    exit_edge;
  };

  struct TrainState {
    std::vector<Properties> num_tr;  // vector for every train: properties
    double                  t;       // time
    double                  delta_t; // delta t
    int    counter; // counter: how many times the state is updated
    double cost;
    std::vector<std::vector<double>> edge_vss;

    // Constructor
    TrainState(size_t n)
        : num_tr(n), t(0.0), delta_t(0.0), counter(0), cost(0.0),
          edge_vss(n) { // constructor.
    }
  };

private:
  // Private struct and properties


  std::vector<std::vector<TrainState>> prev_states; // List of previous states

  double heuristic(TrainState& tr_state);
  double cost(TrainState& tr_state);
  std::vector<TrainState> successors(TrainState& tr_state);
  int collision_vss_check(TrainState& tr_state, int tr1, int tr2,
                                              size_t edge_idx);
  int two_tr_pos_check(TrainState& tr_state, int tr1, int tr2, size_t edge_idx);
  bool pot_collision_check(TrainState& tr_state);
  bool insert_new_vss(TrainState& tr_state, int i, int j, size_t edge_idx);
  bool new_vss_middle_of_edge(TrainState& tr_state, int tr1, double tr1_pos,
                                                 int tr2, double tr2_pos, size_t edge_idx);



public:
  // Public methods
  void initial_state(TrainState& tr_state);
  bool goal_state(TrainState& tr_state);
  bool update_state(TrainState& tr_state);
  bool solve(TrainState& tr_state);




  bool solve(TrainState& initial_state) {
    // priority queue. "double" for "double cost"
    std::priority_queue<std::pair<double, std::pair<double, size_t>>, std::vector<std::pair<double, std::pair<double, size_t>>, std::greater<>> pq;

    // double initial_cost = initial_state.cost; // prev_states[0][0].cost;
    pq.emplace(initial_state.cost, {0.0, 0});  // (cost,(t,idx))

    // Main loop
    while (!pq.empty()) {
      auto [time, index] = state_index;
      auto [current_cost, state_index] = pq.top();
      pq.pop();

      TrainState current_state = prev_states[time][index]; // the state which its successor will be examined

      if (goal_state(current_state,,,,,,,,,,,,,,,,,,,,,,,,,)) {
        return true; // goal reached. TODO: evtl return the trainstate itself?
      }
      else if (update_state(current_state,,,,,,,,,,,,,,,)) {
        // theres 1+ successor.
        for (size_t i = 0; i < prev_states[current_state.t].size(); ++i) {
          pq.emplace(prev_states[current_state.t][i].cost, {current_state.t + current_state.delta_t, i}); // push the new prev_states to pq
        }
      } // do pq for next step
      else { // theres no successor from that state. Delete this state
        prev_states[time][index].clear();
      }
    }
    return false; // return false when no pq and it does not reach goal
  }


  void initial_state(TrainState& tr_state) {
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();

    tr_state.num_tr.resize(tr_list.size());  // Ensure the TrainState is properly sized

    for (size_t i = 0; i < tr_list.size(); ++i) {
      const Schedule tr_schedule = instance.get_schedule(i);

      // Initialize variables: cost, vss, current_pos, entry/exit_vertex, entry/current/exit_edge
      tr_state.num_tr[i].routed_edges.clear();
      tr_state.num_tr[i].routed_edges_current.clear();

      tr_state.num_tr[i].current_pos = 0;  // Starting point is always on vertices
      tr_state.num_tr[i].prev_pos = 0;     // Set previous position

      tr_state.num_tr[i].entry_vertex = tr_schedule.get_entry();  // Entry vertex index
      tr_state.num_tr[i].exit_vertex = tr_schedule.get_exit();    // Exit vertex

      tr_state.num_tr[i].entry_edge = network.out_edges(tr_state.num_tr[i].entry_vertex)[0];  // Entry edge
      tr_state.num_tr[i].current_edge = tr_state.num_tr[i].entry_edge;  // Current edge
      tr_state.num_tr[i].exit_edge = network.in_edges(tr_state.num_tr[i].exit_vertex)[0];  // Exit edge
    }

    tr_state.cost = heuristic(tr_state);  // Calculate the heuristic cost
    tr_state.edge_vss.resize(network.edges.size());  // Resize edge_vss to match the number of edges
    tr_state.edge_vss.clear();  // Clear any existing VSS information

    prev_states[0].clear();  // Clear any existing initial states at t=0
    prev_states[0].push_back(tr_state);  // Add the initial state to prev_states at t=0
  }

  bool goal_state(TrainState& tr_state) {
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();

    double exitedge_len;

    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // i: train index
      // tr_state.num_tr[i].exit_vertex = tr_schedule.get_exit(); //get exit vertex WURDE IN INI_STATE GEMACHT
      exitedge_len = network.get_edge(tr_state.num_tr[i].exit_edge).length;

      if (tr_state.num_tr[i].exit_vertex != network.get_edge(tr_state.num_tr[i].exit_edge).target || tr_state.num_tr[i].current_pos < exitedge_len + tr_list.get_train(i).length) {
        // if train is NOT at exit_edge OR current_pos<(edge.length+tr.length(___._[]_>))
        return false;
      }
    }
    return true;
  }

  bool update_state(TrainState& tr_state) {
    // 1.find successors 2.check collision,vss 3.check cost
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();

    tr_state.counter++; // for each state, counter will be added. start=0->1->2->3->...
    tr_state.t += tr_state.delta_t; // TODO: IS IT HERE? NOT AFTER EXAMINED?

    std::vector<TrainState> next_states = successors(tr_state); // set of next states
    std::vector<TrainState> next_states_valid; // list of valid next states

    for (size_t i = 0; i < next_states.size(); ++i) { // for loop for every path
      if (pot_collision_check(next_states[i]) == 0) { // no collision
        next_states[i].cost = cost(next_states[i]);
        next_states_valid.push_back(next_states[i]); // Add the valid state to the list of next_states_valid
      }
    }

    if (!next_states_valid.empty()) {
      if (prev_states.size() <= tr_state.t / tr_state.delta_t) { // Bsp 15s with 0,5,10,15: size()<=3:add
        prev_states.resize(tr_state.t / tr_state.delta_t + 1); // Resize prev_states
      }
      if (prev_states[tr_state.t].size() < next_states_valid.size()) { // if less size than next states valid
        prev_states[tr_state.t].resize(next_states_valid.size() + 1); // Resize prev_states
      }
      prev_states[tr_state.t] = next_states_valid; // copy the valid states to prev_states[t]
      return true;
      // TODO: check size() or capacity()
    }
    return false;
  }


  // Private function implementation
  double heuristic(TrainState& tr_state) {
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();

    double h = 0; // heuristic wert
    double d = 0; // distance

    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // nächstmögliche Knoten,Länge bis da,Endknoten nötig!
      //const auto v_next = network.get_edge(edge).target; // Nächstmögliche Knoten

      double d = network.shortest_path(tr_state.num_tr[i].current_edge, tr_state.num_tr[i].exit_vertex); // shortest path
      double l_to_v_next = network.get_edge(tr_state.num_tr[i].current_edge).length - tr_state.num_tr[i].current_pos; // LÄNGE BIS DA
      d += l_to_v_next; // add length to nearest vertex
      double h_index = d / tr_list.get_train(i).max_speed;
      h += h_index;
      d = 0;
      h_index = 0;// reset d=0 for next index
      // repeat for the next index(h will be continuously added)
    }
    return h;  // total h
              // TODO:evtl use h[i] and add together at the end?
  }

  double cost(TrainState& tr_state) {
    double g; // SIGMA(g_index)
    g = tr_state.num_tr.size() * tr_state.t; // Cost till here=no.trains*time

    return g + heuristic(tr_state); // f=g+h
  }

  std::vector<TrainState> successors(TrainState& tr_state) {
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();

    std::vector<TrainState> next_states; // list of next states.
    std::vector<TrainState> succ_state; // copy of tr_state for editing
    // reason to make succ_state is so that when something goes wrong the next_states is returned empty.
    double remain_time = tr_state.delta_t;

    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // for each trains
      // for all trains, they move to next point by the max speed
      std::vector<std::vector<size_t>> paths = network.all_paths_of_length_starting_in_edge(tr_state.num_tr[i].current_edge, tr_list.get_train(i).max_speed * tr_state.t + tr_state.num_tr[i].current_pos, tr_state.num_tr[i].exit_edge);
      // get the vector of all routes from prev to current state, Bsp {{1,2,3},{1,2,4}}. **length: from pos0 of the edge!

      for (size_t j = 0; j < paths.size(); ++j) { // [j] shows for each possible path. j is index for new_state[j].num_tr...
        size_t l = paths[j].size;
        succ_state[j] = tr_state; // copy tr_state to succ_state to edit
        succ_state[j].num_tr[i].prev_pos = tr_state.num_tr[i].current_pos; // update the starting position as prev position
        succ_state[j].num_tr[i].routed_edges_current.clear();
        size_t m = succ_state[j].num_tr[i].routed_edges.size();

        for (size_t k = 0; k < l; ++k) { // looking at every possible path: for each Kantenindex:0-(l-1). k=edgeindex
          if (m + l > succ_state[j].num_tr[i].routed_edges.capacity()) {
            succ_state[j].num_tr[i].routed_edges.resize(m + l);
          }
          succ_state[j].num_tr[i].routed_edges[m + k] = network.get_edge(k); // store the edge travelling
          succ_state[j].num_tr[i].routed_edges_current[k] = network.get_edge(k); // store the edge travelling current to succ_state

          if (tr_list.get_train(i).max_speed <= network.get_edge(k).max_speed) {
            // if max_speed.train is slower equal to max_speed.edge: train run by its max speed
            remain_time -= network.get_edge(k).length / tr_list.get_train(i).max_speed;
            if (remain_time < 0) { // if remain time is negative: has exceeded the time till next state
              succ_state[j].num_tr[i].current_edge = paths[j][k]; // train is at this edge[k]
              succ_state[j].num_tr[i].current_pos = tr_list.get_train(i).max_speed * remain_time;
              // current_pos = speed * remain_time
              goto label; // Skip the entire "for (size_t k = 0; k < l; ++k)"
            }
          }
          else {
            remain_time -= network.get_edge(k).length / network.get_edge(k).max_speed;
            if (remain_time < 0) { // if remain time is negative: has exceeded the time till next state
              succ_state[j].num_tr[i].current_edge = paths[j][k]; // train is at this edge[k]
              succ_state[j].num_tr[i].current_pos = network.get_edge(k).max_speed * remain_time;
              // current_pos = speed * remain_time
              goto label; // Skip the entire "for (size_t k = 0; k < l; ++k)"
            }
          }
        }
        label: // skipped
        remain_time = tr_state.delta_t; //initialise again for next for-schleife
        succ_state.push_back(tr_state); // add new state to successor_state(tr_state copy)
        next_states.push_back(tr_state); // add new state to next_states(yet empty)
        // TODO: Check push_back function
      }
    }
    next_states = succ_state;
    if (!next_states.empty()) {
      return next_states;
    }
    else {
      return false;
    }
  }

  bool collision_vss_check(TrainState& tr_state, int tr1, int tr2, size_t edge_idx) {
    // used in pot_collision_check
    // when two trains are in the same TDD, then it has to be checked if they collide
    // TrainList is defined in train.hpp line 33
    /* TODO: EXPLANATION. this fkt is called ONLY when theyre in the same edge (dh going to same direction)
         * so, no need to refer on other direction calculation! */


    // TODO: find() does not give iterator back?
    // .size()=1:startend. nth_path=.begin():start neth_path=end():end
    int tr1_nth_path = std::find(tr_state.num_tr[tr1].routed_edges_current.begin(), tr_state.num_tr[tr1].routed_edges_current.end(), edge_idx);
    int tr2_nth_path = std::find(tr_state.num_tr[tr2].routed_edges_current.begin(), tr_state.num_tr[tr2].routed_edges_current.end(), edge_idx);
    // trX_nth_path: edge is n-th: needs to be either 0 OR .size()

    if (tr_state.num_tr[tr1].routed_edges_current.size() == 1 && tr_state.num_tr[tr2].routed_edges_current.size() == 1) {
      // for except startend;
      if ((tr1_nth_path == 0 && tr2_nth_path == 0) || (tr1_nth_path == tr_state.num_tr[tr1].routed_edges_current.end() && tr2_nth_path == tr_state.num_tr[tr2].routed_edges_current.end())) {
        // startstart or endend
        return 0; // collision
      }
    }

    if (tr1_nth_path == tr_state.num_tr[tr1].routed_edges_current.begin() && tr2_nth_path != tr_state.num_tr[tr2].routed_edges_current.begin()) {
      // tr1 and tr2 both starts from this edge_idx: bedingung für 2. (stend-stst & stend-stend combi)

      if (tr1_nth_path != tr_state.num_tr[tr1].routed_edges_current.end()) {
        // combi stend-stst with tr1 stst: tr1 vorn
        return two_tr_pos_check(tr_state, tr1, tr2, edge_idx);
      }
      else if (tr2_nth_path != tr_state.num_tr[tr2].routed_edges_current.end()) {
        // combi stend-stst with tr2 stst: tr2 vorn
        return two_tr_pos_check(tr_state, tr2, tr1, edge_idx);
      }
      else { // both stend
        if (tr_state.num_tr[tr1].current_pos > tr_state.num_tr[tr1].current_pos) { // tr1 vorne
          return two_tr_pos_check(tr_state, tr1, tr2, edge_idx);
        }
        else if (tr_state.num_tr[tr1].current_pos < tr_state.num_tr[tr1].current_pos) { // tr2 vorne
          return two_tr_pos_check(tr_state, tr2, tr1, edge_idx);
        }
        else { // tr1.current_pos = tr2.current_pos
          return 0; // collision
        }
      }
    }
    else { // theres at least 1 enend. bedingung für 1. (stst-enend & stend-enend combi)
      if (tr1_nth_path != tr_state.num_tr[tr1].routed_edges_current.begin()) { // tr1 enend, tr2 vorne
        return two_tr_pos_check(tr_state, tr2, tr1, edge_idx);
      }
      else { // tr2 enend, tr1 vorne
        return two_tr_pos_check(tr_state, tr1, tr2, edge_idx);
      }
    }
  }

  bool two_tr_pos_check(const TrainState& tr_state, int tr1, int tr2, size_t edge_idx) {
    const TrainList& tr_list = instance.get_train_list();

    // tr1 vorne, tr2 hinten
    double front_end = tr_state.num_tr[tr1].prev_pos - tr_list.get_train(tr1).length;
    double back_start = tr_state.num_tr[tr2].current_pos;

    if (tr_state.edge_vss[edge_idx].size() == 0) { // no vss implemented on edge_idx. Bedingung 1.
      if (front_end <= back_start) {
        return 0; // collision
      }
      else {
        return 1; // new VSS
      }
    }
    else { // theres 1+ vss.
      for (int i = 0; i < tr_state.edge_vss[edge_idx].size(); ++i) { // go through every vss on edge_idx
        if (back_start <= tr_state.edge_vss[edge_idx][i]) { // check which VSS section is back_start at
          if (front_end <= tr_state.edge_vss[edge_idx][i]) { // theyre in the same VSS section
            return 1; // new VSS
          }
          else {
            return 2; // already separated by vss exists. No collision, No VSS needed
          }
        }
      }
      return 0; // back_start is at the last vss section of the edge. collision
    }
  }

  bool pos_collision_check(const TrainState& tr_state) {
    const Network& network = instance.const_n();

    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // if for any two trains, position further search if edge is the same
      // ->first, edge check then position.
      for (size_t j = i+1; j < tr_state.num_tr.size(); ++j) {
        // i,j: two trains' index
        if (network.is_on_same_unbreakable_section(tr_state.num_tr[i].current_edge, tr_state.num_tr[j].current_edge) == 1 || tr_state.num_tr[i].current_edge == network.get_reverse_edge_index(tr_state.num_tr[j].current_edge)) {
          // theyre in an unbreakable section OR theyre in the same section going other way
          return true; // two trains cannot be in unbreakable section. not valid successor. (break)
        }
        else { // theyre not in unbreakable edge
          for (size_t k = 0; l < tr_state.num_tr[i].routed_edges_current.size(); ++k){ // going for every edge in a path
            for (size_t l = 0; n < tr_state.num_tr[j].routed_edges_current.size(); ++l) {
              if (tr_state.num_tr[i].routed_edges_current[k] == tr_state.num_tr[j].routed_edges_current[l]) {
                // if same edge index found; d.h. if they go through the same edge
                if ((k != 0 && k != tr_state.num_tr[i].routed_edges_current.size()) || (l != 0 && l != tr_state.num_tr[j].routed_edges_current.size())) {
                  // m and n are not start or end edge
                  return true; // not valid successor
                }
                else {
                  size_t common_edge = tr_state.num_tr[i].routed_edges_current[k];
                  if (collision_vss_check(tr_state, i, j, common_edge) == 0) { // if collision happening
                    return true; // collision detected. not valid successor
                  }
                  else if (collision_vss_check(tr_state, i, j, common_edge) == 1) {
                    insert_new_vss(tr_state, i, j, common_edge); // TDD section
                  }
                }
              }
            }
          }
        }
      }
    }
    return false; // When no collision: add VSS is needed, then return false
  }

  void AStarVSSPerformanceOptimizationSolver::insert_new_vss(TrainState& tr_state, int i, int j, size_t edge_idx) {
    // TODO: middle of trains OR middle of strecke? - do with middle of strecke.
    // by stst: current_edge!=edge_idx. Then use prev_pos
    if (edge_idx != tr_state.num_tr[i].current_edge) {
      new_vss_middle_of_edge(tr_state, i, tr_state.num_tr[i].prev_pos, j, tr_state.num_tr[j].current_pos, edge_idx);
    }
    else if (edge_idx != tr_state.num_tr[j].current_edge) {
      new_vss_middle_of_edge(tr_state, i, tr_state.num_tr[i].current_pos, j, tr_state.num_tr[j].prev_pos, edge_idx);
    }
    else { // if both are ending in this edge; d.h. both's current_edge (stend,enend)
      new_vss_middle_of_edge(tr_state, i, tr_state.num_tr[i].current_pos, j, tr_state.num_tr[j].current_pos, edge_idx);
    }
    std::sort(tr_state.edge_vss[edge_idx].begin, tr_state.edge_vss[edge_idx].end()); // sort the new added vss
    return true;
  }

  double new_vss_middle_of_edge(TrainState& tr_state, int tr1, double tr1_pos, int tr2, double tr2_pos, size_t edge_idx) {
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();

    if (tr_state.edge_vss[edge_idx].size() == 0) { // no VSS implemented in the edge yet
      double middle_point = network.get_edge(edge_idx).length / 2;
      if (tr1_pos + tr_list.get_train(tr1).length > tr2_pos) {
        // if tr1 vorne, tr2 hinten
        if (tr1_pos + tr_list.get_train(tr1).length > middle_point && tr2_pos < middle_point) {
          // if the middle point is between the trains
          tr_state.edge_vss[edge_idx].push_back(middle_point);
        }
        else if (tr1_pos + tr_list.get_train(tr1).length > middle_point) {
          // two trains are past middle point
          tr_state.edge_vss[edge_idx].push_back(tr2_pos); // add VSS by the second train
        }
        else { // two trains are before middle point
          tr_state.edge_vss[edge_idx].push_back(tr1_pos + tr_list.get_train(tr1).length); // add VSS by the first train
        }
      }
      else {
        if (tr2_pos + tr_list.get_train(tr2).length > middle_point && tr1_pos < middle_point) {
          // if the middle point is between the trains
          tr_state.edge_vss[edge_idx].push_back(middle_point);
        }
        else if (tr2_pos + tr_list.get_train(tr2).length > middle_point) {
          // two trains are past middle point
          tr_state.edge_vss[edge_idx].push_back(tr1_pos); // add VSS by the second train
        }
        else { // two trains are before middle point
          tr_state.edge_vss[edge_idx].push_back(tr2_pos + tr_list.get_train(tr2).length); // add VSS by the first train
        }
      }
    }

    else { // exists 1+ VSS in the edge
      for (int i = 0; i < tr_state.edge_vss[edge_idx].size(); ++i) { // go through every vss on edge_idx
        if (back_start <= tr_state.edge_vss[edge_idx][i]) { // check which VSS section is back_start at
          double middle_section = (tr_state.edge_vss[edge_idx][i-1] + tr_state.edge_vss[edge_idx][i]) / 2;
          if (tr1_pos + tr_list.get_train(tr1).length > tr2_pos) {
            // if tr1 vorne, tr2 hinten
            if (tr1_pos + tr_list.get_train(tr1).length > middle_section && tr2_pos < middle_section) {
              // if the middle point is between the trains
              tr_state.edge_vss[edge_idx].push_back(middle_section);
            }
            else if (tr1_pos + tr_list.get_train(tr1).length > middle_section) {
              // two trains are past middle point
              tr_state.edge_vss[edge_idx].push_back(tr2_pos); // add VSS by the second train
            }
            else { // two trains are before middle point
              tr_state.edge_vss[edge_idx].push_back(tr1_pos + tr_list.get_train(tr1).length); // add VSS by the first train
            }
          }
          else {
            if (tr2_pos + tr_list.get_train(tr2).length > middle_section && tr1_pos < middle_section) {
              // if the middle point is between the trains
              tr_state.edge_vss[edge_idx].push_back(middle_section);
            }
            else if (tr2_pos + tr_list.get_train(tr2).length > middle_section) {
              // two trains are past middle point
              tr_state.edge_vss[edge_idx].push_back(tr1_pos); // add VSS by the second train
            }
            else { // two trains are before middle point
              tr_state.edge_vss[edge_idx].push_back(tr2_pos + tr_list.get_train(tr2).length); // add VSS by the first train
            }
          }
        }
      }
    }
    return true;
  }



}; // class
} // namespace cda_rail::solver::astar_based

