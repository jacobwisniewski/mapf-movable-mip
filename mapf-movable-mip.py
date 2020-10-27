#!/usr/bin/env python3.7

# Copyright 2020, Gurobi Optimization, LLC

# This example formulates and solves the following simple MIP model:
#  maximize
#        x +   y + 2 z
#  subject to
#        x + 2 y + 3 z <= 4
#        x +   y       >= 1
#        x, y, z binary

import gurobipy as gp
from gurobipy import GRB

width = 3
height = 3
VALID_ID = [1, 2, 3, 4, 5, 6, 7, 9]
max_agents = 1
agent_start = [1]
target_start = [1]
adj_list = [[1, 2, 4], [1, 2, 3, 5], [2, 3, 6], [1, 4, 5, 7], [2, 4, 5, 6], [9, 3, 5, 6], [4, 7], [], [9, 6]]

grid_id = [i for i in range(width * height)]
max_time = (width + height) * 2
TIME = [i for i in range((width + height) * 2)]
AGENTS = [i for i in range(max_agents)]


m = gp.Model("mapf-movable-mip")

agents = m.addMVar(shape=(max_agents, len(grid_id), len(grid_id), max_time), vtype=GRB.BINARY)
targets = m.addMVar(shape=(max_agents, len(grid_id), len(grid_id), max_time), vtype=GRB.BINARY)
target_positions = m.addMVar(shape=(max_agents, max_time), vtype=GRB.INTEGER)
dummy = m.addMVar(shape=(max_agents, max_time), vtype=GRB.BINARY)

m.addConstrs(((target_positions[a, t] != node) >> (
        agents[a, adj_list[node], node, t].sum() == agents[a, node, adj_list[node], t].sum())
             for a in AGENTS
             for node in VALID_ID
             for t in range(max_time - 1)), name="agent-flow-constraint")

m.addConstrs((agents[a, adj_list[target_positions[a, t]], target_positions[a, t], t].sum() ==
              agents[a, target_positions[a, t], adj_list[target_positions[a, t]], t].sum() + dummy[a, t + 1]
             for a in AGENTS
             for t in range(max_time - 1)), name="dummy-entry-constraint")

m.addConstr((targets[a, adj_list[node], node, t - 1].sum() == targets[a, node, adj_list[node], t].sum()
             for a in AGENTS
             for node in VALID_ID
             for t in range(1, max_time)), name="target-flow-constraint")

m.addConstrs(
    (agents[a, agent_start[a], adj_list[a], 0].sum() == 1 and targets[a, target_start[a], adj_list[a], 0].sum() == 1
             for a in AGENTS), name="start-position-agents-targets")

m.addConstrs((agents[AGENTS, node, adj_list[node], t].sum() <= 1 and targets[AGENTS, adj_list[node], t].sum() <= 1
             for node in VALID_ID
             for t in TIME), name="restrict-agents-targets-in-same-grid-id")

m.addConstrs((dummy[a1, TIME[0:t]].sum() + agents[AGENTS, node, adj_list[node], t].sum() <= 1
             for a1 in AGENTS
             for node in VALID_ID
             for t in TIME if target_positions[a1, t] == node), name="restrict-agents-visiting-dummy-nodes")

m.addConstrs((not (agents[a1, node1, node2, t] == 1 and agents[a2, node2, node1, t] == 1)
             for a1 in AGENTS
             for a2 in AGENTS
             for node1 in VALID_ID
             for node2 in VALID_ID
             for t in TIME if a1 != a2), name="prevent-swap-conflicts")

m.addConstrs(((targets[target, node, outgoing, t] <= agents[AGENTS, node, outgoing, t].sum() or targets[
    target, node, node, t] == 1
              for outgoing in adj_list[node])
             for target in AGENTS
             for node in VALID_ID
             for t in TIME), name="restrict-target-moving-along-with-agent")

m.addConstrs((targets[a, node, outgoing, t] == 1 >> target_positions[a, t] == node
             for a in AGENTS
             for node in VALID_ID
             for outgoing in VALID_ID
             for t in TIME), name="update-target-pos")

m.setObjective((agents[AGENTS, VALID_ID, VALID_ID, TIME].sum()), GRB.MINIMIZE)

