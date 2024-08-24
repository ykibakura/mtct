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

    /*
     * 29.7 Meeting
     * out_edges()<vector>:bei initial_state, um von der Startknoten die erste Kante zu finden.
     * in_edges()<vector>:bei goal_state, um von der Exitknoten die letzte Kante zu finden.
     * get_successors()<vector>:von Kante zu nextmögliche Kante zu finden: bei successors() gut
     * all_paths_of_length_starting_in_edge()<vector>:finde alle mögliche Wege für bestimmten Länge
     * (neibouring_ecges()<vector>:finde alle verbundene Kanten zu einem Knoten
     *
     * 14.8 Meeting
     * priority_queue Bsp: look at shortest path by .cpp
     * std::priority_queue<std::pair<double, size_t>, std::vector<std::pair<double, size_t>>, std::greater<>>
     * 2+ edges in same TTD: get_unbreakable_section_containing_edge()
     * is_on_same_unbreakable_section(e1,e2) gives if theyre in same ttd
     * ->after collision check:if TTD möglich->this fkt and if false then you can implement the ttd section
     * (new function?)
     * fork the file!
     * for the next_edges_valid: maybe save it to some public(?) list other than train_states?
     * then identify as [t][index]
     * VSS: after the edges are broken: do not change edge feature, d.h. has same edge nr.
     * identify them as length.
     * Mitte der Züge OR Mitte der Strecke(möglichst): some research!
     * */



    // priority queue. "double" for "double cost"
    std::priority_queue<std::pair<double, std::pair<double, size_t>>, std::vector<std::pair<double, std::pair<double, size_t>>, std::greater<>> pq;

    std::vector<std::vector<size_t>> prev_states;


    struct Properties { //function of all the properties needed for tr_state etc
      // Train train; // train properties defined in train.hpp
      // Edge train_edge; // current edge
      bool VSS; // VSS info
      std::vector<size_t> routed_edges; // edges travelled
      std::vector<size_t> routed_edges_current; // edges travelled current state

      double current_pos; // Current position

      int entry_vertex; // entry vertex
      int exit_vertex; // goal vertex

      int entry_edge;
      int current_edge; // current edge
      int exit_edge;
    };


    struct TrainState {
      std::vector<Properties> num_tr; //vector for every train: properties
      int t; // time
      int delta_t = 1; // delta t
      int counter; // counter: how many times the state is updated
      double cost;

      // Constructor
      TrainState(size_t n): num_tr(n), t(0), counter(0), { // constructor.
      }

    };


    // initial state
    TrainState initial_state(TrainState& tr_state, const TrainList& tr_list, const Network& network, const GeneralSolver& instance) {

      for (size_t i = 0; i < tr_list.size(); ++i) {
        const Schedule tr_schedule = instance.get_schedule(i);
        // variables:cost,vss,current_pos,entry/exit_vertex,entry/current/exit_edge
        tr_state.num_tr[i].VSS = false;
        tr_state.num_tr[i].routed_edges.clear();

        tr_state.num_tr[i].current_pos = 0; // assuming starting point is always on vertices

        tr_state.num_tr[i].entry_vertex = tr_schedule.get_entry(); // entry vertex index: start pos initialised.
        tr_state.num_tr[i].exit_vertex = tr_schedule.get_exit(); // exit vertex

        tr_state.num_tr[i].entry_edge = network.out_edges(tr_state.num_tr[i].entry_vertex); // entry edge
        tr_state.num_tr[i].current_edge = tr_state.num_tr[i].entry_edge; // current edge
        tr_state.num_tr[i].exit_edge = network.in_edges(tr_state.num_tr[i].exit_vertex); // exit edge


      }

      tr_state.cost = heuristic(tr_state, tr_list, network);

      TrainState init_state = tr_state;
      prev_states[0][0] = init_state; // add initial state to prev_states[t=0][idx=0]
      return tr_state;
    }


    // goal state
    bool goal_state(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
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


    // update_state
    bool update_state(TrainState& tr_state, const TrainList& tr_list, const Network& network) {
      // 1.find successors 2.check collision,vss 3.check cost
      tr_state.counter++; // for each state, counter will be added. start=0->1->2->3->...
      tr_state.t += tr_state.delta_t; // TODO: IS IT HERE? NOT AFTER EXAMINED?

      std::vector<TrainState> next_states = successors(tr_state, tr_list, network); // set of next states
      std::vector<TrainState> next_states_valid; // list of valid next states

      for (size_t i = 0; i < next_states.size(); ++i) { // for loop for every path
        if (pot_collision_check(next_states[i], tr_list, network) == 0) { // no collision
          next_states[i].cost = cost(next_states[i], tr_list, network);
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
    // cost:update(),vss:vss(comes after pot_collision_check,
    // current_pos:successors(),entry/exit_vertex:NO UPDATE,entry/current/exit_edge:current:successors()


    // heuristic function
    // USE all_edge_pairs_shortest_paths() by RailwayNetwork.hpp L543
    // h(t)=h(t)=SIGMA(h_idx)=SIGMA(d/s), where d is the shortest path to Ziel, s is max velocity
    // d=shortest_path(size_t source_edge_id,size_t target_vertex_id)+distance to next Knoten
    // ACHTUNG:beachte shortest_path von nächstmögliche Knoten?
    // TODO: Heuristic is assuming that all the edges are runned with max_speed.train
    double heuristic(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
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


    // cost function
    // cost is defined as:" the costs are defined by the sum of total travel times"
    // https://www.cda.cit.tum.de/files/eda/2022_rssrail_optimal_railway_routing_using_virtual_subsections.pdf
    double cost(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
      double g; // SIGMA(g_index)
      g = tr_state.num_tr.size() * tr_state.t; // Cost till here=no.trains*time

      return g + heuristic(tr_state, tr_list, network); // f=g+h
    }


    // successor function
    /*std::vector<TrainState> successors(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
      size_t n = tr_state.num_tr.size();
      std::vector<TrainState> next_states; // list of next states.

      for (size_t i = 0; i < n; ++i) {
        // for all trains, they move to next point by the max speed
        double total_length = tr_list.get_train(i).max_speed * tr_state.delta_t;
        double remain_length = total_length;
        TrainState state = tr_state; // copies the current state to new_state

        std::vector<std::vector<size_t>> paths = network.all_paths_of_length_starting_in_edge(tr_state.num_tr[i].current_edge, tr_list.get_train(i).max_speed * tr_state.t + tr_state.num_tr[i].current_pos, tr_state.num_tr[i].exit_edge);
        // get the vector of all routes from prev to current state, Bsp {{1,2,3},{1,2,4}}. **length: from pos0 of the edge!
        size_t m = paths.size();

        for (size_t j = 0; j < m; ++j) { // [j] shows for each possible path. j is index for new_state[j].num_tr...
          size_t l = paths[j].size;

          for (size_t k = 0; k < l; ++k) { // looking at every possible path: for each Kantenindex:0-(l-1)
            remain_length -= network.get_edge(k).length; //remain_length always get subtracted by the edgelength
          }

          state.num_tr[i].current_edge = paths[j][l-1]; // for each possible path: the last edge is the current one
          state.num_tr[i].current_pos = network.get_edge(state.num_tr[i].current_edge).length + remain_length;
          // remain_length=-(distance of current edge which is not yet travelled)
          remain_length = total_length; //initialise again for next for-schleife
          next_states.push_back(state); // add new state to successor_state
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
         *

      return next_states;
    }*/


    // TODO:BETTER SUCCESSOR

    std::vector<TrainState> successors(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
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


    // used in pot_collision_check
    bool collision_vss_check(const TrainState& tr_state, const TrainList& tr_list, const Network& network, int tr1_nr, int tr2_nr) {
      // used in pot_collision_check
      // when two trains are in the same TDD, then it has to be checked if they collide
      // TrainList is defined in train.hpp line 33
      /* TODO: EXPLANATION. this fkt is called ONLY when theyre in the same edge (dh going to same direction)
       * so, no need to refer on other direction calculation! */

      /*if (tr_state.num_tr[tr1_nr].current_pos < tr_list.get_train(tr1_nr).length) {
        int n = tr_state.num_tr[tr1_nr].routed_edges_current.size();

      }*/


      double tr1_length = tr_list.get_train(tr1_nr).length; // length tr1
      double tr1_start = tr_state.num_tr[tr1_nr].current_pos; // start
      double tr1_end = tr1_start + tr1_length; // end: ATTENTION: FOR IMPLEMENTATION ADD DISTANCE TRAVELED!
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
    /* check if they collide: if any tr goes through Gegenrichtungskante
     * 1.If any two trs have passed during the state: collision bc tr_length=all travelled strecke
     * 2.If any are in same edge:needs to be checked.
     * ***In that case: Either current_pos from prev_state OR current_pos should be considered***
     * Bc inbetween edges are all belegt!!!
     * 1.idea?
     * 2.tr_state.num_tr[tr1_nr].routed_edges_current[0], tr_state.num_tr[tr1_nr].current_pos
     * */
    bool pot_collision_check(const TrainState& tr_state, const TrainList& tr_list, const Network& network) {
      for (size_t i = 0; i < tr_state.num_tr.size(); ++i) { // if for any two trains, position further search if edge is the same
        // ->first, edge check then position.
        for (size_t j = i+1; j < tr_state.num_tr.size(); ++j) {
          // i,j: two trains' index
          if (network.is_on_same_unbreakable_section(tr_state.num_tr[i].current_edge, tr_state.num_tr[j].current_edge) == 1 || tr_state.num_tr[i].current_edge == network.get_reverse_edge_index(tr_state.num_tr[j].current_edge))
            // theyre in an unbreakable section OR theyre in the same section going other way
            return true; // two trains cannot be in unbreakable section. not valid successor. (break)
          else if (tr_state.num_tr[j].current_edge == tr_state.num_tr[i].current_edge) {
            // theyre in the same edge (same direction)
            if (collision_vss_check(tr_state, tr_list, network, i, j) == 1) { // if collision happening
              return true; // collision detected. not valid successor. (break)
            }
            else {
              insert_new_vss(tr_state, tr_list, network, i, j); // TDD section
              // TODO: vss added needs to be marked somewhere?
            }
          }
        }
      }
      return false; //When no collision(potential VSS):return false
    }

    // insert VSS
    // TODO: save edges with index: {(100,150),(200),(),(),(200),...}: [0] has vss at 100&150,...
    // -> collision check fkts needs to be changed too!
    bool insert_new_vss(TrainState& tr_state, const TrainList& tr_list, const Network& network, int i, int j) {
      // TODO: middle of trains OR middle of strecke? - do with middle of strecke.
      double middle_point = network.get_edge(tr_state.num_tr[i].current_edge).length / 2;
      if (tr_state.num_tr[i].current_pos + tr_list.get_train(i).length > tr_state.num_tr[j].current_pos) {
        // if i vorne, j hinten
        if (tr_state.num_tr[i].current_pos + tr_list.get_train(i).length > middle_point && tr_state.num_tr[j].current_pos < middle_point) {
          // if the middle point is between the trains
          // TODO: add VSS in the middle: Bsp vss_point = middle_point;
        }
        else if (tr_state.num_tr[i].current_pos + tr_list.get_train(i).length > middle_point) {
          // two trains are past middle point
          // TODO: add VSS by the second train: Bsp vss_point = tr_state.num_tr[j].current_pos;
        }
        else {
          // two trains are before middle point
          // TODO: add VSS by the first train: Bsp vss_point = tr_state.num_tr[i].current_pos + tr_list.get_train(i).length;
        }
      }
      else {
        // j vorne, i hinten
        if (tr_state.num_tr[j].current_pos + tr_list.get_train(j).length > middle_point && tr_state.num_tr[i].current_pos < middle_point) {
          // if the middle point is between the trains
          // TODO: add VSS in the middle: Bsp vss_point = middle_point;
        }
        else if (tr_state.num_tr[j].current_pos + tr_list.get_train(j).length > middle_point) {
          // two trains are past middle point
          // TODO: add VSS by the second train: Bsp vss_point = tr_state.num_tr[i].current_pos;
        }
        else {
          // two trains are before middle point
          // TODO: add VSS by the first train: Bsp vss_point = tr_state.num_tr[j].current_pos + tr_list.get_train(j).length;
        }
      }
      return true;
    }






  }



  public:
    // Constructors. TODO: Implement
    explicit AStarVSSPerformanceOptimizationSolver(const instances::GeneralPerformanceOptimizationInstance& instance): GeneralSolver(instance) {

      /*
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

      }*/
    }
    explicit AStarVSSPerformanceOptimizationSolver(const std::filesystem::path& p): GeneralSolver(p) {
      //
    }
    explicit AStarVSSPerformanceOptimizationSolver(const std::string& path);
    explicit AStarVSSPerformanceOptimizationSolver(const char* path);

    // TODO: Implement missing functions
    bool solve(const TrainState& tr_state, const TrainList& tr_list, const Network& network, const GeneralSolver& instance) {
      TrainState initial_state = initial_state(tr_state, tr_list, network, instance);
      // double initial_cost = initial_state.cost; // prev_states[0][0].cost;
      pq.emplace(initial_state.cost, {0.0, 0});  // TODO: (cost,(t,idx))?

      // Main loop
      while (!pq.empty()) {
        auto [current_cost, {indices}] = pq.top(); // TODO: Copied from .cpp: work on variables
        pq.pop();

        double time = indices.first;
        size_t index = indices.second;

        TrainState current_state = prev_states[time][index]; // the state which its successor will be examined

        if (goal_state(current_state, tr_list, network)) {
          return true; // goal reached. TODO: evtl return the trainstate itself?
        }

        if (update_state(current_state, tr_list, network)) {
          // theres 1+ successor. TODO: save the new state to pq
          for (size_t i = 0; i < prev_states[current_state.t].size(); ++i) {
            pq.emplace(prev_states[current_state.t][i].cost,
                       {current_state.t + current_state.delta_t, i}); // push the new prev_states to pq
          }
        } // do pq for next step

        else {
          // theres no successor from that state. Delete this state
          prev_states[time][index].clear();
        }
      }
      return false; // return false when no pq and it does not reach goal
    }



    using GeneralSolver::solve;
    [[nodiscard]] instances::SolGeneralPerformanceOptimizationInstance<
        instances::GeneralPerformanceOptimizationInstance>
    solve(int time_limit, bool debug_input) override;
  };
} // namespace cda_rail::solver::astar_based





