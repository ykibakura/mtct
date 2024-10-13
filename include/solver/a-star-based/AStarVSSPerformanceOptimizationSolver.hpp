#pragma once
#include "datastructure/RailwayNetwork.hpp"
#include "probleminstances/VSSGenerationTimetable.hpp"
#include "solver/GeneralSolver.hpp"

#include <filesystem>
#include <string>
#include <utility>
#include <queue>
#include <vector>


namespace cda_rail::solver::astar_based {

class AStarVSSPerformanceOptimizationSolver
    : public GeneralSolver<
          instances::VSSGenerationTimetable,
          instances::SolVSSGenerationTimetable> {

private:
  // Private struct and properties

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
    int goal_reached;
    double goal_reached_t;
  };

public:

  explicit AStarVSSPerformanceOptimizationSolver(const std::string& instance_path) {
    instance = instances::VSSGenerationTimetable::import_instance(instance_path);
  };

  struct TrainState {
    std::vector<Properties> num_tr;  // vector for every train: properties
    double                  t;       // time
    double                  delta_t; // delta t
    size_t    counter; // counter: how many times the state is updated
    double cost;
    std::vector<std::vector<double>> edge_vss;

    // Constructor
    TrainState()
        : t(0.0), delta_t(0.0), counter(0), cost(0.0) {}
    // default const

    TrainState(size_t num_tr_input, double t_input, double dt_input, size_t counter_input, double cost_input, size_t num_edges_input)
        : num_tr(num_tr_input), t(t_input), delta_t(dt_input), counter(counter_input), cost(cost_input),
          edge_vss(num_edges_input) {}
    // constructor
    // https://stackoverflow.com/questions/5498937/when-do-we-need-to-have-a-default-constructor
  };

  std::vector<std::vector<TrainState>> prev_states = std::vector<std::vector<TrainState>>(100); // List of previous states


  bool solve(TrainState& initial_state) {
    // priority queue. "double" for "double cost"
    std::priority_queue<std::pair<double, std::pair<double, size_t>>, std::vector<std::pair<double, std::pair<double, size_t>>>, std::greater<std::pair<double, std::pair<double, size_t>>>> pq;

    // double initial_cost = initial_state.cost; // prev_states[0][0].cost;
    pq.emplace(std::make_pair(initial_state.cost, std::make_pair(0.0, 0)));  // (cost,(t,idx))

    // Main loop
    while (!pq.empty()) {
      auto [current_cost, state_index] = pq.top();
      auto [time, index] = state_index;
      pq.pop();

      TrainState current_state = prev_states[time / initial_state.delta_t][index]; // the state which its successor will be examined
      if (current_state.t == 0) { // check if init_state = current_state
        if (goal_state(current_state)) {
          return true; // goal reached. TODO: evtl return the trainstate itself?
        }
      }
      if (update_state(current_state)) {
        // theres 1+ successor.
        for (size_t i = 0; i < prev_states[current_state.t / current_state.delta_t].size(); ++i) {
          pq.emplace(std::make_pair(prev_states[current_state.t / current_state.delta_t][i].cost, std::make_pair(current_state.t, i)));
          // push the new prev_states to pq
        }
        /*if (goal_state(current_state)) {
          return true; // goal reached.
          // Removed by debugging: as current_state.cost is not updated in cost function!
        }*/
        for (size_t i = 0; i < prev_states[current_state.t / current_state.delta_t].size(); ++i) {
          if (goal_state(prev_states[current_state.t / current_state.delta_t][i])) { // push the new prev_states to pq
            return true; // goal reached.
          }
        }
      } // do pq for next step
      else { // theres no successor from that state. Delete this state
        prev_states[time].erase(prev_states[time].begin() + index);  // Erase the state
        // https://stackoverflow.com/questions/42523981/erase-element-of-struct-vector
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

      tr_state.num_tr[i].current_pos = 0.0;  // Starting point is always on vertices
      tr_state.num_tr[i].prev_pos = 0.0;     // Set previous position

      tr_state.num_tr[i].entry_vertex = tr_schedule.get_entry();  // Entry vertex index
      tr_state.num_tr[i].exit_vertex = tr_schedule.get_exit();    // Exit vertex

      tr_state.num_tr[i].entry_edge = network.out_edges(tr_state.num_tr[i].entry_vertex)[0];  // Entry edge
      tr_state.num_tr[i].current_edge = tr_state.num_tr[i].entry_edge;  // Current edge
      tr_state.num_tr[i].exit_edge = network.in_edges(tr_state.num_tr[i].exit_vertex)[0];  // Exit edge

      tr_state.num_tr[i].goal_reached = 0;
      tr_state.num_tr[i].goal_reached_t = 9999999; // TODO: change to 0?
    }

    tr_state.cost = heuristic(tr_state);  // Calculate the heuristic cost
    tr_state.edge_vss.resize(network.number_of_edges());  // Resize edge_vss to match the number of edges
    tr_state.edge_vss.empty();  // Clear any existing VSS information

    prev_states[0].empty();  // Clear any existing initial states at t=0
    prev_states[0].push_back(tr_state);  // Add the initial state to prev_states at t=0
  }


  bool goal_state(TrainState& tr_state) {
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();

    double exitedge_len;

    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // i: train index
      // tr_state.num_tr[i].exit_vertex = tr_schedule.get_exit(); //get exit vertex WURDE IN INI_STATE GEMACHT
      exitedge_len = network.get_edge(tr_state.num_tr[i].exit_edge).length;

      // if (tr_state.num_tr[i].exit_vertex != network.get_edge(tr_state.num_tr[i].exit_edge).target || tr_state.num_tr[i].current_pos < exitedge_len + tr_list.get_train(i).length) {
      if (tr_state.num_tr[i].exit_vertex != network.get_edge(tr_state.num_tr[i].current_edge).target || tr_state.num_tr[i].current_pos < exitedge_len) {
        // if train is NOT at exit_edge OR current_pos<(edge.length+tr.length(___._[]_>))
        return false;
      }
    }
    return true;
  }


  bool update_state(TrainState& tr_state) {
    // 1.find successors 2.check collision,vss 3.check cost

    std::vector<TrainState> next_states = successors(tr_state); // set of next states
    std::vector<TrainState> next_states_valid; // list of valid next states
    std::vector<TrainState> next_states_pos_adjusted;

    tr_state.counter++; // for each state, counter will be added. start=0->1->2->3->...
    tr_state.t += tr_state.delta_t;

    for (size_t i = 0; i < next_states.size(); ++i) { // for loop for every path
      next_states[i].t += next_states[i].delta_t;
      next_states[i].counter++;
      next_states[i].cost = cost(next_states[i]);
      if (pos_collision_check(next_states[i])) { // no collision
        next_states_valid.push_back(next_states[i]); // Add the valid state to the list of next_states_valid
      }
      if (!pos_collision_check(next_states[i])) { // collision
        next_states_pos_adjusted = adjust_tr_pos(next_states[i]);
        for (size_t j = 0; j < next_states_pos_adjusted.size(); ++j) {
          next_states_pos_adjusted[j].cost = cost(next_states_pos_adjusted[j]);
          if (pos_collision_check(next_states_pos_adjusted[j])) { // no collision
            next_states_valid.push_back(next_states_pos_adjusted[j]); // Add the valid state to the list of next_states_valid
          }
        }

        // next_states_valid.push_back(next_states[i]); // Add the valid state to the list of next_states_valid
      }
    }

    if (!next_states_valid.empty()) { // Anmerkung: tr_state.t is already updated.
      if (prev_states.size() <= tr_state.t / tr_state.delta_t) { // Bsp 15s with 0,5,10,15: size()<=3:add
        prev_states.resize(tr_state.t / tr_state.delta_t + 1); // Resize prev_states
      }
      if (prev_states[tr_state.t / tr_state.delta_t].size() < next_states_valid.size()) { // if less size than next states valid
        prev_states[tr_state.t / tr_state.delta_t].resize(next_states_valid.size() + 1); // Resize prev_states
      }
      prev_states[tr_state.t / tr_state.delta_t] = next_states_valid; // copy the valid states to prev_states[t]

      return true;
    }
    return false;
  }


  // Private function implementation
  double heuristic(TrainState& tr_state) {
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();

    double h = 0; // heuristic wert
    double d; // distance
    double h_index; // heuristic for train i

    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // nächstmögliche Knoten,Länge bis da,Endknoten nötig!
      //const auto v_next = network.get_edge(edge).target; // Nächstmögliche Knoten

      if (std::optional<double> shortest_path_optional = network.shortest_path(tr_state.num_tr[i].current_edge, tr_state.num_tr[i].exit_vertex); shortest_path_optional.has_value()) {
        d = shortest_path_optional.value(); // shortest path
        // https://stackoverflow.com/questions/27384571/c-convert-boostoptionaldouble-to-double
        double l_to_v_next = network.get_edge(tr_state.num_tr[i].current_edge).length - tr_state.num_tr[i].current_pos; // Length till end edge:start from shortest_path
        d += l_to_v_next; // add length to nearest vertex. d=shortest_path from current_pos
        h_index = d / tr_list.get_train(i).max_speed;
      }
      else { // no valid path exists. either tr is at goal_pos or no path exists
        if (tr_state.num_tr[i].goal_reached == 1) {
          //h_index=0, thus h stays
          h_index = 0;
        }
        else {
          h_index = 9999999999;
        }
      }
      h += h_index;
      h_index = 0;// reset d=0 for next index
      // repeat for the next index(h will be continuously added)
    }
    return h;  // total h
  }


  double cost(TrainState& tr_state) {
    double g = 0; // SIGMA(g_index)
    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) {
      if (tr_state.num_tr[i].goal_reached == 0) {
        if (instance.get_schedule(i).get_t_0() >= tr_state.t) {
          // g is 0
        }
        else {
          g += (tr_state.t - instance.get_schedule(i).get_t_0()); // Cost till here=no.trains*time
        }
      }
      if (tr_state.num_tr[i].goal_reached == 1) {
        // TODO: what time did the train arrive?
        g += (tr_state.num_tr[i].goal_reached_t - instance.get_schedule(i).get_t_0()); // Cost till here=no.trains*time
      }
    }
    return g + heuristic(tr_state); // f=g+h
  }


  std::vector<TrainState> successors(TrainState& tr_state) {
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();

    TrainState succ_state = tr_state; // next state candidate: if valid successor->copied to next_states.
    // as a default value; copy tr_state to succ_state to edit
    std::vector<std::vector<Properties>> paths_sorted_with_num_tr = std::vector<std::vector<Properties>>(tr_state.num_tr.size());
    // paths_sorted_with_num_tr[i][n] with i=num_tr, n for each paths
    // https://stackoverflow.com/questions/10559283/how-to-create-a-vector-of-user-defined-size-but-with-no-predefined-values

    double remain_time = tr_state.delta_t;
    std::vector<size_t> path_copied_counter(tr_state.num_tr.size(), 0);
    size_t next_states_counter = 1;

    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // for each trains
      // if t_0 is not reached: train stays at the initial point
      if (tr_state.num_tr[i].goal_reached == 0) {
        if ((tr_state.t + tr_state.delta_t) >
            instance.get_schedule(i)
                .get_t_0()) { // if t for next state is past t_0
          std::vector<std::vector<size_t>> paths;
          if (tr_state.delta_t >
              (tr_state.t - instance.get_schedule(i).get_t_0())) {
            // if train does not run for full delta_t
            paths = network.all_paths_of_length_starting_in_edge(
                tr_state.num_tr[i].current_edge,
                tr_list.get_train(i).max_speed *
                        (tr_state.t + tr_state.delta_t -
                         instance.get_schedule(i).get_t_0()) +
                    tr_state.num_tr[i].current_pos,
                tr_state.num_tr[i].exit_vertex);
          }
          else {
            paths = network.all_paths_of_length_starting_in_edge(
                tr_state.num_tr[i].current_edge,
                tr_list.get_train(i).max_speed * tr_state.delta_t +
                    tr_state.num_tr[i].current_pos,
                tr_state.num_tr[i].exit_vertex);
          }
          // get the vector of all routes from prev to current state, Bsp {{1,2,3},{1,2,4}}. **length: from pos0 of the edge!

          for (size_t j = 0; j < paths.size();
               ++j) { // [j] shows for each possible path. j is index for new_state[j].num_tr...
            size_t l = paths[j].size();
            paths_sorted_with_num_tr[i].resize(
                paths.size());     // resize to no. path available
            succ_state = tr_state; // copy tr_state to succ_state to edit
            succ_state.num_tr[i].prev_pos =
                tr_state.num_tr[i].current_pos; // update the starting position as prev position
            succ_state.num_tr[i].routed_edges_current.clear();
            size_t m = succ_state.num_tr[i].routed_edges.size();

            for (size_t k = 0; k < l;
                 ++k) { // looking at every possible path: for each Kantenindex:0-(l-1). k=edgeindex
              if (m + l > succ_state.num_tr[i].routed_edges.size()) {
                succ_state.num_tr[i].routed_edges.resize(m + l);
              }
              if (l > succ_state.num_tr[i].routed_edges_current.size()) {
                succ_state.num_tr[i].routed_edges_current.resize(l);
              }
              succ_state.num_tr[i].routed_edges[m + k] =
                  paths[j][k]; // store the edge travelling
              succ_state.num_tr[i].routed_edges_current[k] =
                  paths[j][k]; // store the edge travelling current to succ_state

              if (tr_list.get_train(i).max_speed <=
                  network.get_edge(paths[j][k]).max_speed) {
                // if max_speed.train is slower equal to max_speed.edge: train run by its max speed
                remain_time -= (network.get_edge(paths[j][k]).length -
                                succ_state.num_tr[i].current_pos) /
                               tr_list.get_train(i).max_speed;
                succ_state.num_tr[i].current_pos =
                    0; // update current_pos as the train ran the edge completely
                if (remain_time <= 0) { // if remain time is negative: has exceeded the time till next state
                  succ_state.num_tr[i].current_edge =
                      paths[j][k]; // train is at this edge[k]
                  succ_state.num_tr[i].current_pos =
                      network.get_edge(paths[j][k]).length - (tr_list.get_train(i).max_speed *
                      (-remain_time));
                  // current_pos = speed * remain_time
                  succ_state.num_tr[i].routed_edges.resize(m + k + 1);
                  succ_state.num_tr[i].routed_edges_current.resize(
                      k + 1); // resize routed_edges and _current to delete unused array
                  goto label; // Skip the entire "for (size_t k = 0; k < l; ++k)"
                } else if (paths[j][k] ==
                           tr_state.num_tr[i]
                               .exit_edge) { // tr is exit_edge & remain_time>0. Goal arrived
                  // goal done.
                  succ_state.num_tr[i].current_edge = paths[j][k];
                  succ_state.num_tr[i].current_pos = network.get_edge(succ_state.num_tr[i].current_edge).length;
                  succ_state.num_tr[i].goal_reached_t = succ_state.t + succ_state.delta_t - remain_time;
                  succ_state.num_tr[i].goal_reached = 1;
                }
              } else {
                remain_time -= (network.get_edge(paths[j][k]).length -
                                succ_state.num_tr[i].current_pos) /
                               network.get_edge(paths[j][k]).max_speed;
                succ_state.num_tr[i].current_pos = 0;
                if (remain_time <= 0) { // if remain time is negative: has exceeded the time till next state
                  succ_state.num_tr[i].current_edge =
                      paths[j][k]; // train is at this edge[k]
                  succ_state.num_tr[i].current_pos =
                      network.get_edge(paths[j][k]).length - (network.get_edge(paths[j][k]).max_speed *
                                                              (-remain_time));
                  // current_pos = total_length - speed * remain_time: remain_time from the last edge
                  succ_state.num_tr[i].routed_edges.resize(m + k + 1);
                  succ_state.num_tr[i].routed_edges_current.resize(
                      k + 1); // resize routed_edges and _current to delete unused array
                  goto label; // Skip the entire "for (size_t k = 0; k < l; ++k)"
                } else if (paths[j][k] ==
                           tr_state.num_tr[i]
                               .exit_edge) { // tr is exit_edge & remain_time>0. Goal arrived
                  succ_state.num_tr[i].current_edge = paths[j][k];
                  succ_state.num_tr[i].current_pos = network.get_edge(succ_state.num_tr[i].current_edge).length;
                  succ_state.num_tr[i].goal_reached_t = succ_state.t + succ_state.delta_t - remain_time;
                  succ_state.num_tr[i].goal_reached = 1;
                }
              }
            }
          label: // skipped
            remain_time =
                tr_state.delta_t; // initialise again for next for-schleife
            for (size_t n = 0; n <= j;
                 ++n) { // if routed_edges and _current already matches with other candidates
              if (paths_sorted_with_num_tr[i][n].routed_edges ==
                  succ_state.num_tr[i].routed_edges) {
                // the found path is already investigated. Do not copy to paths_sorted_with_num_tr
                break;
              }
              if (n == j) {
                paths_sorted_with_num_tr[i][j] =
                    succ_state.num_tr[i]; // copy routed_edges and _current
                path_copied_counter[i]++;
              }
            }
            // succ_state.clear();
            succ_state = tr_state; // reset the succ_state to default; since clear() doesnot work for struct
          }
          next_states_counter *=
              path_copied_counter[i]; // mutiply the next path candidate for every num_tr
        }
      }
    }
    std::vector<TrainState> next_states(next_states_counter, succ_state); // list of valid next states
    for (size_t o = 0; o < next_states_counter; ++o) { // for every possible path o
      size_t p = 1;
      size_t q = 1;
      for (size_t i = 0; i < tr_state.num_tr.size(); ++i) {
        if (tr_state.num_tr[i].goal_reached == 0) {
          if (tr_state.t >= instance.get_schedule(i).get_t_0()) {
            q *= path_copied_counter[i];
            size_t write_nth_path = (o / p) % q;
            next_states[o].num_tr[i] =
                paths_sorted_with_num_tr[i][write_nth_path];
            p *= path_copied_counter[i];
          }
        }
      }
    }
    return next_states;
  }

  TrainState rollback_tr_pos_opposite(TrainState& tr_state, size_t i, size_t j) {
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();
    TrainState next_states_pos_adjusted = tr_state;

    if (tr_state.num_tr[i].routed_edges_current.size() == 1) {
      // the tr i position can/would not be moved.
      for (size_t n = 0; n < tr_state.num_tr[j].routed_edges_current.size(); ++n) {
        if (network.get_predecessors(tr_state.num_tr[j].routed_edges_current[tr_state.num_tr[j].routed_edges_current.size()-1-n]).size() > 1) {
          // if the previous edge has 2+ branches-> go back there (assumed there is a 5m unbreakable edge too!)
          next_states_pos_adjusted.num_tr[j].current_edge = tr_state.num_tr[j].routed_edges_current[tr_state.num_tr[j].routed_edges_current.size()-1-n-2];
          next_states_pos_adjusted.num_tr[j].current_pos = network.get_edge(tr_state.num_tr[j].current_edge).length - 0.001;
          next_states_pos_adjusted.num_tr[j].routed_edges_current.resize(tr_state.num_tr[j].routed_edges_current.size()-n-2);
          next_states_pos_adjusted.num_tr[j].routed_edges.resize(tr_state.num_tr[j].routed_edges.size()-n-2);
          return next_states_pos_adjusted;
        }
      }
    }
    if (tr_state.num_tr[j].routed_edges_current.size() == 1) {
      // the tr i position can/would not be moved.
      for (size_t n = 0; n < tr_state.num_tr[i].routed_edges_current.size(); ++n) {
        if (network.get_predecessors(tr_state.num_tr[i].routed_edges_current[tr_state.num_tr[i].routed_edges_current.size()-1-n]).size() > 1) {
          // if the previous edge has 2+ branches-> go back there (assumed there is a 5m unbreakable edge too!)
          next_states_pos_adjusted.num_tr[i].current_edge = tr_state.num_tr[i].routed_edges_current[tr_state.num_tr[i].routed_edges_current.size()-1-n-2];
          next_states_pos_adjusted.num_tr[i].current_pos = network.get_edge(tr_state.num_tr[i].current_edge).length - 0.001;
          next_states_pos_adjusted.num_tr[i].routed_edges_current.resize(tr_state.num_tr[i].routed_edges_current.size()-n-2);
          next_states_pos_adjusted.num_tr[i].routed_edges.resize(tr_state.num_tr[i].routed_edges.size()-n-2);
          return next_states_pos_adjusted;
        }
      }
    }
    else { // both train comes from other edge
      for (size_t n = 0; n < tr_state.num_tr[i].routed_edges_current.size(); ++n) {
        if (tr_list.get_train(i).max_speed >= tr_list.get_train(j).max_speed) {
          // i is faster. move tr i
          if (network.get_predecessors(tr_state.num_tr[i].routed_edges_current[tr_state.num_tr[i].routed_edges_current.size()-1-n]).size() > 1) {
            // if the previous edge has 2+ branches-> go back there (assumed there is a 5m unbreakable edge too!)
            next_states_pos_adjusted.num_tr[i].current_edge = tr_state.num_tr[i].routed_edges_current[tr_state.num_tr[i].routed_edges_current.size()-1-n-2];
            next_states_pos_adjusted.num_tr[i].current_pos = network.get_edge(tr_state.num_tr[i].current_edge).length - 0.001;
            next_states_pos_adjusted.num_tr[i].routed_edges_current.resize(tr_state.num_tr[i].routed_edges_current.size()-n-2);
            next_states_pos_adjusted.num_tr[i].routed_edges.resize(tr_state.num_tr[i].routed_edges.size()-n-2);
            return next_states_pos_adjusted;
          }
        }
        else {
          // j is faster. move tr j
          if (network.get_predecessors(tr_state.num_tr[j].routed_edges_current[tr_state.num_tr[j].routed_edges_current.size()-1-n]).size() > 1) {
            next_states_pos_adjusted.num_tr[j].current_edge = tr_state.num_tr[j].routed_edges_current[tr_state.num_tr[j].routed_edges_current.size()-1-n-2];
            next_states_pos_adjusted.num_tr[j].current_pos = network.get_edge(tr_state.num_tr[j].current_edge).length - 0.001;
            next_states_pos_adjusted.num_tr[j].routed_edges_current.resize(tr_state.num_tr[j].routed_edges_current.size()-n-2);
            next_states_pos_adjusted.num_tr[j].routed_edges.resize(tr_state.num_tr[j].routed_edges.size()-n-2);
            return next_states_pos_adjusted;
          }
        }
      }
    }
    return next_states_pos_adjusted;
  }

  /*bool rollback_tr_pos_same(TrainState& tr_state, size_t i, size_t j, size_t edge_idx) {
    int tr1_nth_path = std::distance(tr_state.num_tr[i].routed_edges_current.begin(), std::find(tr_state.num_tr[i].routed_edges_current.begin(), tr_state.num_tr[i].routed_edges_current.end(), edge_idx));
    int tr2_nth_path = std::distance(tr_state.num_tr[j].routed_edges_current.begin(), std::find(tr_state.num_tr[j].routed_edges_current.begin(), tr_state.num_tr[j].routed_edges_current.end(), edge_idx));
    // meaning tr_state.num_tr[i].routed_edges_current[tr1_nth_path]=tr_state.num_tr[j].routed_edges_current[tr2_nth_path]
    // TODO: welcher Zug ist hinter?
  }*/

  std::vector<TrainState> adjust_tr_pos(TrainState& tr_state) {
    // TODO
    const Network& network = instance.const_n();
    const TrainList& tr_list = instance.get_train_list();
    std::vector<TrainState> next_states_pos_adjusted = std::vector<TrainState>(1);
    next_states_pos_adjusted[0] = tr_state;
    TrainState state_zwischenlager = tr_state;

    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // if for any two trains, position further search if edge is the same
      // ->first, edge check then position.
      for (size_t j = i+1; j < tr_state.num_tr.size(); ++j) {
        // i,j: two trains' index
        if (network.is_on_same_unbreakable_section(tr_state.num_tr[i].current_edge, tr_state.num_tr[j].current_edge) == 1 || tr_state.num_tr[i].current_edge == network.get_reverse_edge_index(tr_state.num_tr[j].current_edge)) {
          // theyre in an unbreakable section OR theyre in the same section going other way
          // return false; // two trains cannot be in unbreakable section. not valid successor. (break)
          // TODO: same edge is not considered as same unbreakable section!
          if (!ignore_collision(tr_state, i, j)) {
            state_zwischenlager = rollback_tr_pos_opposite(tr_state, i, j);
            next_states_pos_adjusted.push_back(state_zwischenlager);
            goto label;
          }
          else {
            // collision can be ignored. if no other pos_adjusted are added:return tr_state
          }
        }
        else { // theyre not in unbreakable edge
          for (size_t k = 0; k < tr_state.num_tr[i].routed_edges_current.size(); ++k){ // going for every edge in a path
            for (size_t l = 0; l < tr_state.num_tr[j].routed_edges_current.size(); ++l) {
              if (tr_state.num_tr[i].routed_edges_current[k] == tr_state.num_tr[j].routed_edges_current[l]) {
                // if same edge index found; d.h. if they go through the same edge
                // TODO: memo. theyre going same direction!
                if (k == 0 && l == 0) {
                  //  both edges started from this edge
                  if (tr_state.num_tr[i].current_pos > tr_state.num_tr[j].current_pos) { // tr i vorne, j hinten
                    if (tr_state.num_tr[i].current_pos - tr_list.get_train(i).length - 1 > 0 && tr_state.num_tr[j].current_pos < tr_state.num_tr[i].current_pos - tr_list.get_train(i).length - 1) {
                      state_zwischenlager.num_tr[j].current_pos = tr_state.num_tr[i].current_pos - tr_list.get_train(i).length - 1;
                      state_zwischenlager.num_tr[i].routed_edges_current.resize(1);
                      state_zwischenlager.num_tr[i].routed_edges.resize(tr_state.num_tr[i].routed_edges.size() - tr_state.num_tr[i].routed_edges_current.size() + 1);
                      state_zwischenlager.num_tr[j].routed_edges_current.resize(1);
                      state_zwischenlager.num_tr[j].routed_edges.resize(tr_state.num_tr[j].routed_edges.size() - tr_state.num_tr[j].routed_edges_current.size() + 1);
                      next_states_pos_adjusted.push_back(state_zwischenlager);
                      goto label;
                    }
                    else {
                    }
                  }
                  if (tr_state.num_tr[i].current_pos < tr_state.num_tr[j].current_pos) { // tr i vorne, j hinten
                    if (tr_state.num_tr[j].current_pos - tr_list.get_train(j).length - 1 > 0 && tr_state.num_tr[i].current_pos < tr_state.num_tr[j].current_pos - tr_list.get_train(i).length - 1) {
                      state_zwischenlager.num_tr[i].current_pos = tr_state.num_tr[j].current_pos - tr_list.get_train(j).length - 1;
                      state_zwischenlager.num_tr[i].routed_edges_current.resize(1);
                      state_zwischenlager.num_tr[i].routed_edges.resize(tr_state.num_tr[i].routed_edges.size() - tr_state.num_tr[i].routed_edges_current.size() + 1);
                      state_zwischenlager.num_tr[j].routed_edges_current.resize(1);
                      state_zwischenlager.num_tr[j].routed_edges.resize(tr_state.num_tr[j].routed_edges.size() - tr_state.num_tr[j].routed_edges_current.size() + 1);
                      next_states_pos_adjusted.push_back(state_zwischenlager);
                      goto label;
                    }
                    else {
                    }
                  }
                }
                else if (k == 0) {
                  // k->i started this edge, front. adjust l->j position
                  state_zwischenlager.num_tr[j].current_pos = tr_state.num_tr[i].current_pos - tr_list.get_train(i).length - 1;
                  state_zwischenlager.num_tr[j].routed_edges_current.resize(l+1);
                  state_zwischenlager.num_tr[j].routed_edges.resize(tr_state.num_tr[j].routed_edges.size() - tr_state.num_tr[j].routed_edges_current.size() + 1);
                  next_states_pos_adjusted.push_back(state_zwischenlager);
                  goto label;
                }
                else if (l == 0) {
                  // l->j started this edge, front. adjust k->i position
                  state_zwischenlager.num_tr[i].current_pos = tr_state.num_tr[j].current_pos - tr_list.get_train(j).length - 1;
                  state_zwischenlager.num_tr[i].routed_edges_current.resize(k+1);
                  state_zwischenlager.num_tr[i].routed_edges.resize(tr_state.num_tr[i].routed_edges.size() - tr_state.num_tr[i].routed_edges_current.size() + 1);
                  next_states_pos_adjusted.push_back(state_zwischenlager);
                  goto label;
                }
                else {
                  // both did not start from this edge. Should not exist!
                }
              }
              else {
                // TODO: memo. other direction: unbreakable/get_reverse
                if (network.is_on_same_unbreakable_section(tr_state.num_tr[i].routed_edges_current[k], tr_state.num_tr[j].routed_edges_current[l]) == 1 || tr_state.num_tr[i].routed_edges_current[k] == network.get_reverse_edge_index(tr_state.num_tr[j].routed_edges_current[l])) {
                  // collision.
                  if (!ignore_collision(tr_state, i, j)) {
                    state_zwischenlager = rollback_tr_pos_opposite(tr_state, i, j);
                    next_states_pos_adjusted.push_back(state_zwischenlager);
                    goto label;
                  }
                  else {
                    // collision can be ignored. if no other pos_adjusted are added:return tr_state
                  }
                }
              }

            }
          }
        }
        label:
        state_zwischenlager = tr_state;
      }
    }
    return next_states_pos_adjusted;
  }

  int collision_vss_check(TrainState& tr_state, int tr1, int tr2, size_t edge_idx) {
    // used in pot_collision_check
    // when two trains are in the same TDD, then it has to be checked if they collide
    // TrainList is defined in train.hpp line 33
    /* TODO: EXPLANATION. this fkt is called ONLY when theyre in the same edge (dh going to same direction)
         * so, no need to refer on other direction calculation! */

    // .size()=1:startend. nth_path = 0 :start(stst OR stend). nth_path = size()-1: end(enend OR stend)
    int tr1_nth_path = std::distance(tr_state.num_tr[tr1].routed_edges_current.begin(), std::find(tr_state.num_tr[tr1].routed_edges_current.begin(), tr_state.num_tr[tr1].routed_edges_current.end(), edge_idx));
    int tr2_nth_path = std::distance(tr_state.num_tr[tr2].routed_edges_current.begin(), std::find(tr_state.num_tr[tr2].routed_edges_current.begin(), tr_state.num_tr[tr2].routed_edges_current.end(), edge_idx));
    // https://stackoverflow.com/questions/6136362/convert-iterator-to-int : begin() & end() gives iterator back
    // trX_nth_path: edge is n-th: needs to be either 0 OR .size()

    if (tr_state.num_tr[tr1].routed_edges_current.size() != 1 && tr_state.num_tr[tr2].routed_edges_current.size() != 1) {
      // for except startend & startend;
      // if ((tr1_nth_path == 0 && tr2_nth_path == 0) || (tr1_nth_path == tr_state.num_tr[tr1].routed_edges_current.end() && tr2_nth_path == tr_state.num_tr[tr2].routed_edges_current.end())) {
      if ((tr1_nth_path == 0 && tr2_nth_path == 0) || (tr1_nth_path == tr_state.num_tr[tr1].routed_edges_current.size() - 1 && tr2_nth_path == tr_state.num_tr[tr2].routed_edges_current.size() - 1)) {
          // startstart or endend
        return 0; // collision
      }
    }

    if (tr1_nth_path == 0 && tr2_nth_path == 0) {
      // tr1 and tr2 both starts from this edge_idx: bedingung für 2. (stend-stst & stend-stend combi)
      if (tr1_nth_path != tr_state.num_tr[tr1].routed_edges_current.size() - 1) {
        // combi stend-stst with tr1 stst: tr1 vorn
        return two_tr_pos_check(tr_state, tr1, tr2, edge_idx);
      }
      if (tr2_nth_path != tr_state.num_tr[tr2].routed_edges_current.size() - 1) {
        // combi stend-stst with tr2 stst: tr2 vorn
        return two_tr_pos_check(tr_state, tr2, tr1, edge_idx);
      }
      else { // both stend
        if (tr_state.num_tr[tr1].current_pos > tr_state.num_tr[tr2].current_pos) { // tr1 vorne
          return two_tr_pos_check(tr_state, tr1, tr2, edge_idx);
        }
        if (tr_state.num_tr[tr1].current_pos < tr_state.num_tr[tr2].current_pos) { // tr2 vorne
          return two_tr_pos_check(tr_state, tr2, tr1, edge_idx);
        }
        if (tr_state.num_tr[tr1].current_pos == tr_state.num_tr[tr2].current_pos) { // tr1.current_pos = tr2.current_pos
          return 0; // collision
        }
      }
    }
    else { // theres at least 1 enend. bedingung für 1. (stst-enend & stend-enend combi)
      if (tr1_nth_path != 0) { // tr1 enend, tr2 vorne
        return two_tr_pos_check(tr_state, tr2, tr1, edge_idx);
      }
      if (tr2_nth_path != 0) { // tr2 enend, tr1 vorne
        return two_tr_pos_check(tr_state, tr1, tr2, edge_idx);
      }
    }
  }


  int two_tr_pos_check(TrainState& tr_state, int tr1, int tr2, size_t edge_idx) {
    // tr1 vorne, tr2 hinten
    const TrainList& tr_list = instance.get_train_list();

    double front_end = tr_state.num_tr[tr1].prev_pos - tr_list.get_train(tr1).length;
    // TODO: if prev_pos = 0? then
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
        else if (front_end >= tr_state.edge_vss[edge_idx][i]) {
          return 1;
        }
      }
      return 0; // back_start is at the last vss section of the edge. collision
    }
  }


  bool ignore_collision(TrainState& tr_state, int tr1, int tr2) {
    if (tr_state.t <= instance.get_schedule(tr1).get_t_0() || tr_state.t <= instance.get_schedule(tr2).get_t_0() || tr_state.num_tr[tr1].goal_reached == 1 || tr_state.num_tr[tr2].goal_reached == 1) {
      // ignore if one train is not departed, or already reached goal
      return true; // ignore. valid successor
    }
    return false; // do not ignore the collision. invalid
  }


  bool pos_collision_check(TrainState& tr_state) {
    const Network& network = instance.const_n();

    for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // if for any two trains, position further search if edge is the same
      // ->first, edge check then position.
      for (size_t j = i+1; j < tr_state.num_tr.size(); ++j) {
        // i,j: two trains' index
        if (network.is_on_same_unbreakable_section(tr_state.num_tr[i].current_edge, tr_state.num_tr[j].current_edge) == 1 || tr_state.num_tr[i].current_edge == network.get_reverse_edge_index(tr_state.num_tr[j].current_edge)) {
          // theyre in an unbreakable section OR theyre in the same section going other way
          // return false; // two trains cannot be in unbreakable section. not valid successor. (break)
          return ignore_collision(tr_state, i, j);
        }
        else { // theyre not in unbreakable edge
          for (size_t k = 0; k < tr_state.num_tr[i].routed_edges_current.size(); ++k){ // going for every edge in a path
            for (size_t l = 0; l < tr_state.num_tr[j].routed_edges_current.size(); ++l) {
              if (tr_state.num_tr[i].routed_edges_current[k] == tr_state.num_tr[j].routed_edges_current[l]) {
                // if same edge index found; d.h. if they go through the same edge
                if ((k != 0 && k != tr_state.num_tr[i].routed_edges_current.size() - 1) || (l != 0 && l != tr_state.num_tr[j].routed_edges_current.size() - 1)) {
                  // k and l are not start or end edge
                  // return false; // not valid successor
                  return ignore_collision(tr_state, i, j);
                }
                else {
                  size_t common_edge = tr_state.num_tr[i].routed_edges_current[k];
                  if (collision_vss_check(tr_state, i, j, common_edge) == 0) { // if collision happening
                    // return false; // collision detected. not valid successor
                    return ignore_collision(tr_state, i, j);
                  }
                  if (collision_vss_check(tr_state, i, j, common_edge) == 1) {
                    insert_new_vss(tr_state, i, j, common_edge); // TDD section
                  }
                }
              }
              else {
                // TODO: reverse or unbreakable
                if (network.is_on_same_unbreakable_section(tr_state.num_tr[i].routed_edges_current[k], tr_state.num_tr[j].routed_edges_current[l]) == 1 || tr_state.num_tr[i].routed_edges_current[k] == network.get_reverse_edge_index(tr_state.num_tr[j].routed_edges_current[l])) {
                  // collision.
                  return ignore_collision(tr_state, i, j);
                }
              }
            }
          }
        }
      }
    }
    return true; // When no collision
  }


  bool insert_new_vss(TrainState& tr_state, int i, int j, size_t edge_idx) {
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
    std::sort(tr_state.edge_vss[edge_idx].begin(), tr_state.edge_vss[edge_idx].end()); // sort the new added vss
    return true;
  }


  bool new_vss_middle_of_edge(TrainState& tr_state, int tr1, double tr1_pos, int tr2, double tr2_pos, size_t edge_idx) {
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
      for (int i = 0; i < tr_state.edge_vss[edge_idx].size() + 1; ++i) { // go through every vss on edge_idx
        double middle_point;
        if (i == 0) {
          middle_point = tr_state.edge_vss[edge_idx][i] / 2; // between position 0 and first vss
        }
        else if (i == tr_state.edge_vss[edge_idx].size()) {
          middle_point = (tr_state.edge_vss[edge_idx][i - 1] + network.get_edge(edge_idx).length) / 2;
        }
        else {
          middle_point = (tr_state.edge_vss[edge_idx][i - 1] + tr_state.edge_vss[edge_idx][i]) / 2;
        }

        if (tr1_pos + tr_list.get_train(tr1).length > tr2_pos) { // if tr1 vorne, tr2 hinten
          double back_start = tr_state.num_tr[tr2].current_pos;
          if (back_start <= tr_state.edge_vss[edge_idx][i]) { // check which VSS section is back_start at
            if (tr1_pos + tr_list.get_train(tr1).length > middle_point && tr2_pos < middle_point) {
              // if the middle point is between the trains
              tr_state.edge_vss[edge_idx].push_back(middle_point);
              break;
            }
            else if (tr1_pos + tr_list.get_train(tr1).length > middle_point) {
              // two trains are past middle point
              tr_state.edge_vss[edge_idx].push_back(tr2_pos); // add VSS by the second train
              break;
            }
            else { // two trains are before middle point
              tr_state.edge_vss[edge_idx].push_back(tr1_pos + tr_list.get_train(tr1).length); // add VSS by the first train
              break;
            }
          }
          else if (i == tr_state.edge_vss[edge_idx].size()) {
            if (tr1_pos + tr_list.get_train(tr1).length > middle_point && tr2_pos < middle_point) {
              // if the middle point is between the trains
              tr_state.edge_vss[edge_idx].push_back(middle_point);
              break;
            }
            else if (tr1_pos + tr_list.get_train(tr1).length > middle_point) {
              // two trains are past middle point
              tr_state.edge_vss[edge_idx].push_back(tr2_pos); // add VSS by the second train
              break;
            }
            else { // two trains are before middle point
              tr_state.edge_vss[edge_idx].push_back(tr1_pos + tr_list.get_train(tr1).length); // add VSS by the first train
              break;
            }
          }
        }
        else { // tr2 vorne, tr1 hinten
          double back_start = tr_state.num_tr[tr1].current_pos;
          if (back_start <= tr_state.edge_vss[edge_idx][i]) { // check which VSS section is back_start at
            if (tr2_pos + tr_list.get_train(tr2).length > middle_point && tr1_pos < middle_point) {
              // if the middle point is between the trains
              tr_state.edge_vss[edge_idx].push_back(middle_point);
              break;
            }
            else if (tr2_pos + tr_list.get_train(tr2).length > middle_point) {
              // two trains are past middle point
              tr_state.edge_vss[edge_idx].push_back(tr1_pos); // add VSS by the second train
              break;
            }
            else { // two trains are before middle point
              tr_state.edge_vss[edge_idx].push_back(tr2_pos + tr_list.get_train(tr2).length); // add VSS by the first train
              break;
            }
          }
          else if (i == tr_state.edge_vss[edge_idx].size()) {
            // implement the vss right hand sind of the last vss
            if (tr2_pos + tr_list.get_train(tr2).length > middle_point && tr1_pos < middle_point) {
              // if the middle point is between the trains
              tr_state.edge_vss[edge_idx].push_back(middle_point);
              break;
            }
            else if (tr2_pos + tr_list.get_train(tr2).length > middle_point) {
              // two trains are past middle point
              tr_state.edge_vss[edge_idx].push_back(tr1_pos); // add VSS by the second train
              break;
            }
            else { // two trains are before middle point
              tr_state.edge_vss[edge_idx].push_back(tr2_pos + tr_list.get_train(tr2).length); // add VSS by the first train
              break;
            }
          }
        }
      }
    }
    return true;
  }


instances::SolVSSGenerationTimetable solve(int time_limit, bool debug_input) override {return instances::SolVSSGenerationTimetable(instance, 15);};

}; // class
} // namespace cda_rail::solver::astar_based

