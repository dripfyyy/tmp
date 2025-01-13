/******************************************************************************\
* Distance vector routing protocol without reverse path poisoning.             *
\******************************************************************************/

#include <stdlib.h>
#include "routing-simulator.h"

// Structure to represent messages exchanged between nodes
typedef struct data_t {
  cost_t data[MAX_NODES];
} data_t;

// Structure to hold the state of a node
typedef struct state_t {
  cost_t cost[MAX_NODES][MAX_NODES];
  node_t next_hop[MAX_NODES];
} state_t;

// Function to allocate and initialize a node's state
state_t *init_state() {
  state_t *node_state = (state_t *)malloc(sizeof(state_t));

  int start_node = get_first_node();
  int end_node = get_last_node();

  for (int src = start_node; src <= end_node; src++) {
    for (int dest = start_node; dest <= end_node; dest++) {
      if (src == dest) {
        node_state->cost[src][dest] = 0; // Distance to self is 0
      } else {
        node_state->cost[src][dest] = COST_INFINITY; // Initially unreachable
      }
    }
    node_state->next_hop[src] = -1; // No forwarding information
  }
  return node_state;
}

// Implementation of the Bellman-Ford algorithm
int compute_routes(state_t *node_state) {
  int has_changes = 0;

  int current = get_current_node();
  int first = get_first_node();
  int last = get_last_node();

  for (int destination = first; destination <= last; destination++) {
    if (destination != current) {
      cost_t shortest_distance = COST_INFINITY;
      node_t best_next_hop = -1;

      for (int neighbor = first; neighbor <= last; neighbor++) {
        if (neighbor != current && get_link_cost(neighbor) != COST_INFINITY) {
          cost_t total_cost = COST_ADD(get_link_cost(neighbor), node_state->cost[neighbor][destination]);

          if (total_cost < shortest_distance) {
            shortest_distance = total_cost;
            best_next_hop = neighbor;
          }
        }
      }

      if (node_state->cost[current][destination] != shortest_distance) {
        node_state->cost[current][destination] = shortest_distance;
        node_state->next_hop[destination] = best_next_hop;
        set_route(destination, best_next_hop, shortest_distance);
        has_changes = 1;
      } else {
        set_route(destination, best_next_hop, shortest_distance);
      }
    }
  }
  return has_changes;
}

// Notify neighboring nodes of changes
void broadcast_updates(state_t *node_state) {
  if (compute_routes(node_state)) {
    int start = get_first_node();
    int stop = get_last_node();
    int current_node = get_current_node();

    for (int neighbor = start; neighbor <= stop; neighbor++) {
      if (neighbor != current_node && get_link_cost(neighbor) != COST_INFINITY) {
        data_t *update_message = (data_t *)malloc(sizeof(data_t));

        for (int destination = start; destination <= stop; destination++) {
          update_message->data[destination] = node_state->cost[current_node][destination];
        }

        send_message(neighbor, update_message, sizeof(data_t));
      }
    }
  }
}

// Handle link cost changes
void notify_link_change(node_t neighbor, cost_t new_cost) {
  state_t *node_state = (state_t *)get_state();
  broadcast_updates(node_state);
}

// Handle incoming messages from neighbors
void notify_receive_message(node_t sender, void *message, size_t size) {
  state_t *node_state = (state_t *)get_state();
  data_t *received_message = (data_t *)message;

  int first_node = get_first_node();
  int last_node = get_last_node();

  for (int destination = first_node; destination <= last_node; destination++) {
    node_state->cost[sender][destination] = received_message->data[destination];
  }

  broadcast_updates(node_state);
}