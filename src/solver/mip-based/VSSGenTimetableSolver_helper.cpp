#include "CustomExceptions.hpp"
#include "solver/mip-based/VSSGenTimetableSolver.hpp"

#include <cmath>
#include <unordered_map>

std::vector<size_t>
cda_rail::solver::mip_based::VSSGenTimetableSolver::unbreakable_section_indices(
    size_t train_index) const {
  /**
   * This function returns the indices of the unbreakable sections that are
   * traversed by the train with index train_index
   * @param train_index index of the train
   * @return vector of indices
   */

  std::vector<size_t> indices;
  const auto& tr_name  = instance.get_train_list().get_train(train_index).name;
  const auto& tr_route = instance.get_route(tr_name).get_edges();
  for (size_t i = 0; i < unbreakable_sections.size(); ++i) {
    bool edge_found = false;
    // If unbreakable_section[i] (of type vector) and tr_route (of type vector)
    // overlap (have a common element), add i to indices
    for (size_t j0 = 0; j0 < unbreakable_sections[i].size() && !edge_found;
         ++j0) {
      for (size_t j1 = 0; j1 < tr_route.size() && !edge_found; ++j1) {
        if (unbreakable_sections[i][j0] == tr_route[j1]) {
          indices.push_back(i);
          edge_found = true;
        }
      }
    }
  }

  return indices;
}

cda_rail::solver::mip_based::VSSGenTimetableSolver::TemporaryImpossibilityStruct
cda_rail::solver::mip_based::VSSGenTimetableSolver::
    get_temporary_impossibility_struct(const size_t& tr, const int& t) const {
  /**
   * This returns a struct containing information about the previous and
   * following station.
   *
   * @param tr index of the train
   * @param t time index
   *
   * @return struct containing information about the previous and following
   * station
   */

  // Initialize struct
  TemporaryImpossibilityStruct s;

  const auto& train_list  = instance.get_train_list();
  const auto  tr_name     = train_list.get_train(tr).name;
  const auto& tr_schedule = instance.get_schedule(tr_name);

  s.to_use   = true;
  s.t_before = train_interval[tr].first;
  s.t_after  = train_interval[tr].second + 1;
  s.v_before = tr_schedule.v_0;
  s.v_after  = tr_schedule.v_n;

  for (const auto& tr_stop : tr_schedule.stops) {
    const auto t0 = tr_stop.begin / dt;
    const auto t1 =
        static_cast<int>(std::ceil(static_cast<double>(tr_stop.end) / dt));
    if (t >= t0 && t <= t1) {
      s.to_use = false;
      return s;
    }
    if (t0 < t && t0 > s.t_before) {
      s.t_before = t0;
      s.edges_before =
          instance.get_station_list().get_station(tr_stop.station).tracks;
      s.v_before = 0;
    }
    if (t1 > t && t1 < s.t_after) {
      s.t_after = t1;
      s.edges_after =
          instance.get_station_list().get_station(tr_stop.station).tracks;
      s.v_after = 0;
    }
  }

  return s;
}

double
cda_rail::solver::mip_based::VSSGenTimetableSolver::max_distance_travelled(
    const size_t& tr, const int& time_steps, const double& v0,
    const double& a_max, const bool& braking_distance) const {
  const auto& train_object = instance.get_train_list().get_train(tr);
  const auto& v_max        = train_object.max_speed;
  const auto  time_diff    = time_steps * dt;
  double      ret_val      = 0;
  double      final_speed  = NAN;
  if (!this->include_train_dynamics) {
    ret_val += time_diff * v_max;
    final_speed = v_max;
  } else if (time_diff < (v_max - v0) / a_max) {
    ret_val +=
        0.5 * time_diff *
        (a_max * time_diff + 2 * v0); // int_{0}^{time_diff} (a_max*t + v0) dt
    final_speed = a_max * time_diff + v0;
  } else {
    ret_val += (v_max - v0) * (v_max + v0) /
               (2 * a_max); // int_{0}^{(v_max-v0)/a_max} (a_max*t + v0) dt
    ret_val += (time_diff - (v_max - v0) / a_max) *
               v_max; // int_{(v_max-v0)/a_max}^{time_diff} v_max dt
    final_speed = v_max;
  }
  if (braking_distance) {
    ret_val += final_speed * final_speed / (2 * train_object.deceleration);
  }
  return ret_val;
}

std::pair<std::vector<std::vector<size_t>>, std::vector<std::vector<size_t>>>
cda_rail::solver::mip_based::VSSGenTimetableSolver::common_entry_exit_vertices()
    const {
  /**
   * Returns trains that have common entry or exit vertices sorted by entry/exit
   * time
   */

  auto compare_entry = [this](size_t tr1, size_t tr2) {
    return train_interval[tr1].first < train_interval[tr2].first;
  };
  auto compare_exit = [this](size_t tr1, size_t tr2) {
    return train_interval[tr1].second > train_interval[tr2].second;
  };

  std::pair<std::vector<std::vector<size_t>>, std::vector<std::vector<size_t>>>
                                                  ret_val;
  std::unordered_map<size_t, std::vector<size_t>> entry_vertices;
  std::unordered_map<size_t, std::vector<size_t>> exit_vertices;

  for (size_t tr = 0; tr < num_tr; ++tr) {
    entry_vertices[instance.get_schedule(tr).entry].push_back(tr);
    exit_vertices[instance.get_schedule(tr).exit].push_back(tr);
  }

  for (auto& [_, tr_list] : entry_vertices) {
    if (tr_list.size() > 1) {
      std::sort(tr_list.begin(), tr_list.end(), compare_entry);
      ret_val.first.push_back(tr_list);
    }
  }
  for (auto& [_, tr_list] : exit_vertices) {
    if (tr_list.size() > 1) {
      std::sort(tr_list.begin(), tr_list.end(), compare_exit);
      ret_val.second.push_back(tr_list);
    }
  }

  return ret_val;
}

double cda_rail::solver::mip_based::VSSGenTimetableSolver::lower_bound_frac(
    size_t relevant_edge_index, cda_rail::SeparationType type,
    size_t vss_index) {
  if (relevant_edge_index > relevant_edges.size()) {
    throw exceptions::EdgeNotExistentException(relevant_edge_index);
  }

  const auto& e            = relevant_edges.at(relevant_edge_index);
  const auto  vss_number_e = instance.n().max_vss_on_edge(e);

  if (vss_index >= vss_number_e) {
    throw exceptions::ConsistencyException("vss_index too large");
  }

  if (type == SeparationType::UNIFORM) {
    // frac = (vss_index + 1) / n
    // 1 <= n <= vss_number_e + 1
    // frac >= (vss_index + 1) / (vss_number_e + 1)
    return (static_cast<double>(vss_index) + 1) /
           (static_cast<double>(vss_number_e) + 1);
  }

  throw std::logic_error("Not implemented for type specified.");
}

double cda_rail::solver::mip_based::VSSGenTimetableSolver::upper_bound_frac(
    size_t relevant_edge_index, cda_rail::SeparationType type,
    size_t vss_index) {
  if (relevant_edge_index > relevant_edges.size()) {
    throw exceptions::EdgeNotExistentException(relevant_edge_index);
  }

  const auto& e            = relevant_edges.at(relevant_edge_index);
  const auto  vss_number_e = instance.n().max_vss_on_edge(e);

  if (vss_index >= vss_number_e) {
    throw exceptions::ConsistencyException("vss_index too large");
  }

  if (type == SeparationType::UNIFORM) {
    // frac = (vss_index + 1) / n
    // 1 <= n <= vss_number_e + 1
    // frac <= (vss_index + 1) / 1
    return (static_cast<double>(vss_index) + 1);
  }

  throw std::logic_error("Not implemented for type specified.");
}

double cda_rail::solver::mip_based::VSSGenTimetableSolver::lower_bound_bpos(
    size_t edge_index, size_t vss_index, bool consider_reverse) {
  if (vss_model == VSSModel::DISCRETE) {
    throw exceptions::ConsistencyException(
        "bpos does not exist for discrete VSS model");
  }
  if (!instance.n().has_edge(edge_index)) {
    throw exceptions::EdgeNotExistentException(edge_index);
  }

  const auto  vss_number_e = instance.n().max_vss_on_edge(edge_index);
  const auto& e_len        = instance.n().get_edge(edge_index).length;

  if (vss_index >= vss_number_e) {
    throw exceptions::ConsistencyException("vss_index too large");
  }

  if (vss_model == VSSModel::CONTINUOUS) {
    return 0;
  }

  if (vss_model != VSSModel::LIMITED) {
    throw std::logic_error("Not implemented for VSS model specified.");
  }

  const auto relevant_edge_index =
      std::find(relevant_edges.begin(), relevant_edges.end(), edge_index);
  if (relevant_edge_index != relevant_edges.end()) {
    // bpos >= e_len * min_{type \in separation_types}
    // lower_bound_frac(relevant_edge_index, type, vss_index)
    double min_frac = INF;
    for (const auto& type : separation_types) {
      min_frac = std::min(min_frac, lower_bound_frac(relevant_edge_index -
                                                         relevant_edges.begin(),
                                                     type, vss_index));
    }
    return e_len * min_frac;
  }

  if (!consider_reverse) {
    throw exceptions::ConsistencyException(
        "Edge is not relevant but has to be");
  }

  const auto& reverse_e = instance.n().get_reverse_edge_index(edge_index);
  if (!reverse_e.has_value()) {
    throw exceptions::ConsistencyException(
        "Edge has no reverse edge, but is also not relevant.");
  }
  return e_len - upper_bound_bpos(reverse_e.value(), vss_index, false);
}

double cda_rail::solver::mip_based::VSSGenTimetableSolver::upper_bound_bpos(
    size_t edge_index, size_t vss_index, bool consider_reverse) {
  if (vss_model == VSSModel::DISCRETE) {
    throw exceptions::ConsistencyException(
        "bpos does not exist for discrete VSS model");
  }
  if (!instance.n().has_edge(edge_index)) {
    throw exceptions::EdgeNotExistentException(edge_index);
  }

  const auto  vss_number_e = instance.n().max_vss_on_edge(edge_index);
  const auto& e_len        = instance.n().get_edge(edge_index).length;

  if (vss_index >= vss_number_e) {
    throw exceptions::ConsistencyException("vss_index too large");
  }

  if (vss_model == VSSModel::CONTINUOUS) {
    return e_len;
  }

  if (vss_model != VSSModel::LIMITED) {
    throw std::logic_error("Not implemented for VSS model specified.");
  }

  const auto relevant_edge_index =
      std::find(relevant_edges.begin(), relevant_edges.end(), edge_index);
  if (relevant_edge_index != relevant_edges.end()) {
    // bpos <= e_len * max_{type \in separation_types}
    // upper_bound_frac(relevant_edge_index, type, vss_index)
    double max_frac = 0;
    for (const auto& type : separation_types) {
      max_frac = std::max(max_frac, upper_bound_frac(relevant_edge_index -
                                                         relevant_edges.begin(),
                                                     type, vss_index));
    }
    return e_len * max_frac;
  }

  if (!consider_reverse) {
    throw exceptions::ConsistencyException(
        "Edge is not relevant but has to be");
  }

  const auto& reverse_e = instance.n().get_reverse_edge_index(edge_index);
  if (!reverse_e.has_value()) {
    throw exceptions::ConsistencyException(
        "Edge has no reverse edge, but is also not relevant.");
  }
  return e_len - lower_bound_bpos(reverse_e.value(), vss_index, false);
}
