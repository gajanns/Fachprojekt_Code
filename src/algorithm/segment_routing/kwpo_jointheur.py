from algorithm.generic_sr import GenericSR
import time
import numpy as np
from .demand_first_waypoints import DemandsFirstWaypoints

class MultiWayPoints(GenericSR):

    seed = 42

    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, k: int=1, oc: bool=False, **kwargs):
        super().__init__(nodes, links, demands, weights, None)

        self.__nodes = nodes  # [i, ..., n-1]
        self.__links = links  # [(i, j, capacity), ...]
        self.__demands = demands  # [(src, dst, demand), ...]
        self.__max_waypoints = k
        self.__oc=oc

    def solve(self) -> dict:
        """
        sequential combination of two arbitrary sr algorithms to compute
        first optimal weight setting and then waypoints or vice versa
        """
        from algorithm.sr_factory import get_algorithm

        solution_list=list()
        current_demands = self.__demands

        # route on shortest paths
        heur_ospf = get_algorithm(algorithm_name="heur_ospf_weights",demands=current_demands, nodes=self.__nodes,
                              links=self.__links)
        solution_ospf = heur_ospf.solve()
        current_weights = solution_ospf['weights']
        solution_list.append(solution_ospf)

        for i in range(self.__max_waypoints):
            single_waypoint = SingleWayPointHeur(nodes=self.__nodes,links=self.__links, demands=current_demands,weights=current_weights, oc=self.__oc)
            solution_wp = single_waypoint.solve()

            current_solution= dict(solution_list[-1])
            current_demands=solution_wp['waypoints_demand']
            current_solution["execution_time"] += solution_wp["execution_time"]
            current_solution["process_time"] += solution_wp["process_time"]
            current_solution["objective"] = solution_wp["objective"]
            current_solution["loads"] = solution_wp["loads"]
            solution_list.append(current_solution)

        return solution_list

    def get_name(self):
        """ returns name of algorithm """
        return f"multi_waypoints"


class SingleWayPointHeur(DemandsFirstWaypoints):

    SORT_CHUNKS = 1

    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, waypoints: dict = None,oc:bool = False, **kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)
        self.__node_capacities = self.extract_nodes_capacities(links)
        self.__oc=oc

    def extract_nodes_capacities(self, links):
        nodes_capacity = dict()
        for i,j,c in links:
            if i in nodes_capacity:
                nodes_capacity[i] += c
            else:
                nodes_capacity[i] = c
            if j in nodes_capacity:
                nodes_capacity[j] += c
            else:
                nodes_capacity[j] = c
        return nodes_capacity

    def calc_pair_capacity(self, pair):
        return self.__node_capacities[pair[0]] + self.__node_capacities[pair[1]]

    def extract_sorted_ids(self, demands: dict, sortbycap: bool = False):

        sorted_ids = dict(zip(range(len(self._DemandsFirstWaypoints__demands)),
                 np.array(self._DemandsFirstWaypoints__demands)[:, 2].argsort()[::-1]))
        sorted_ids = list(sorted_ids.values())

        if sortbycap:
            tmp= sorted_ids.copy()
            chunk_size=np.floor_divide(len(tmp),self.SORT_CHUNKS)
            if chunk_size > 0:
                tmp[0:chunk_size+len(tmp)%chunk_size-1]=sorted(tmp[0:chunk_size+len(tmp)%chunk_size-1],
                                                               key = lambda x: -self.calc_pair_capacity(demands[tmp[x]]))
                for i in range(1,self.SORT_CHUNKS-1):
                    left_lim = i*chunk_size+len(tmp)%chunk_size
                    tmp[left_lim:left_lim+chunk_size-1] = sorted(tmp[left_lim:left_lim+chunk_size-1],
                                                                 key = lambda x: -self.calc_pair_capacity(demands[tmp[x]]))
                sorted_ids=tmp
        return sorted_ids

    def __demands_first_waypoints(self):
        """ main procedure """
        distances = self._DemandsFirstWaypoints__compute_distances()
        sp_fraction_map = self._DemandsFirstWaypoints__get_shortest_path_fraction_map(distances)
        best_flow_map = self._DemandsFirstWaypoints__get_flow_map(sp_fraction_map)
        best_util_map, best_objective = self._DemandsFirstWaypoints__compute_utilization(best_flow_map)

        waypoints = dict()
        waypoints_demand=list()
        sorted_demand_idx_map = self.extract_sorted_ids(self._DemandsFirstWaypoints__demands,self.__oc)
        for d_map_idx in range(len(self._DemandsFirstWaypoints__demands)):
            d_idx = sorted_demand_idx_map[d_map_idx]
            s, t, d = self._DemandsFirstWaypoints__demands[d_idx]
            best_waypoint = None
            for waypoint in range(self._DemandsFirstWaypoints__n):
                if waypoint == s or waypoint == t:
                    continue
                flow_map = self._DemandsFirstWaypoints__update_flow_map(sp_fraction_map, best_flow_map, s, t, d, waypoint)
                util_map, objective = self._DemandsFirstWaypoints__compute_utilization(flow_map)

                if objective < best_objective:
                    best_flow_map = flow_map
                    best_util_map = util_map
                    best_objective = objective
                    best_waypoint = waypoint

            if best_waypoint is not None:
                waypoints[d_idx] = [(s, best_waypoint), (best_waypoint, t),len(waypoints_demand)]
                if d!=0:
                    waypoints_demand.append((s,best_waypoint,d))
                    waypoints_demand.append((best_waypoint, t, d))
            else:
                #waypoints[d_idx] = (s, t)
                waypoints_demand.append((s,t,d))
        loads = {(u, v): best_util_map[u][v] for u, v, in self._DemandsFirstWaypoints__links}
        return loads, waypoints, best_objective, waypoints_demand

    def solve(self) -> dict:
        """ compute solution """

        self.__start_time = t_start = time.time()  # sys wide time
        pt_start = time.process_time()  # count process time (e.g. sleep excluded and count per core)
        loads, waypoints, objective,waypoints_demand = self.__demands_first_waypoints()
        pt_duration = time.process_time() - pt_start
        t_duration = time.time() - t_start

        solution = {
            "objective": objective,
            "execution_time": t_duration,
            "process_time": pt_duration,
            "waypoints": waypoints,
            "weights": self._DemandsFirstWaypoints__weights,
            "loads": loads,
            "waypoints_demand":waypoints_demand
        }

        return solution