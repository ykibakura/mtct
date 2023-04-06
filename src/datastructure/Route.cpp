#include "datastructure/Route.hpp"
#include "datastructure/RailwayNetwork.hpp"

void cda_rail::Route::push_back_edge(int edge_index, cda_rail::Network network) {
    /**
     * Adds the edge to the end of the route.
     * Throws an error if the edge does not exist in the network or is not a valid successor of the last edge.
     *
     * @param edge_index The index of the edge to add.
     * @param network The network to which the edge belongs.
     */

    if (!network.has_edge(edge_index)) {
        throw std::out_of_range("Edge does not exist.");
    }
    if (edges.size() > 0 && !network.is_valid_successor(edges.back(), edge_index)) {
        throw std::out_of_range("Edge is not a valid successor.");
    }
    edges.push_back(edge_index);
}

template<typename T>
void cda_rail::Route::push_back_edge(T source, T target, cda_rail::Network network) {
    /**
     * Adds the edge to the end of the route.
     * Throws an error if the edge does not exist in the network or is not a valid successor of the last edge.
     *
     * @param source The source of the edge to add.
     * @param target The target of the edge to add.
     * @param network The network to which the edge belongs.
     */

    if (!network.has_edge(source, target)) {
        throw std::out_of_range("Edge does not exist.");
    }
    if (edges.size() > 0 && !network.is_valid_successor(edges.back(), std::pair<T, T>(source, target))) {
        throw std::out_of_range("Edge is not a valid successor.");
    }
    edges.push_back(network.get_edge_index(source, target));
}
