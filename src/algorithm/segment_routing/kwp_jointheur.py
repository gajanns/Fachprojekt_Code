from ..generic_sr import GenericSR
import time
import numpy as np
from .demand_first_waypoints import DemandsFirstWaypoints
from ..sr_factory import get_algorithm


class kWayPointHeur(DemandsFirstWaypoints):

    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, k: int = 1, waypoints: dict = None ,**kwargs):
        super().__init__(nodes, links, demands, weights, waypoints)
        self.k = k


    def __demands_first_waypoints(self):
        """ main procedure """
        distances = self.__compute_distances()
        sp_fraction_map = self.__get_shortest_path_fraction_map(distances)
        best_flow_map = self.__get_flow_map(sp_fraction_map)
        best_util_map, best_objective = self.__compute_utilization(best_flow_map)

        waypoints = dict()
        sorted_demand_idx_map = dict(zip(range(len(self.__demands)), np.array(self.__demands)[:, 2].argsort()[::-1]))
        for d_map_idx in range(len(self.__demands)):
            d_idx = sorted_demand_idx_map[d_map_idx]
            s, t, d = self.__demands[d_idx]
            best_waypoint = None
            if self.k <= 0:
                break
            for waypoint in range(self.__n):
                if waypoint == s or waypoint == t:
                    continue
                flow_map = self.__update_flow_map(sp_fraction_map, best_flow_map, s, t, d, waypoint)
                util_map, objective = self.__compute_utilization(flow_map)

                if objective < best_objective:
                    best_flow_map = flow_map
                    best_util_map = util_map
                    best_objective = objective
                    best_waypoint = waypoint

            if best_waypoint is not None:
                waypoints[d_idx] = [(s, best_waypoint), (best_waypoint, t)]
                self.k = self.k - 1
            else:
                waypoints[d_idx] = [(s, t)]

        loads = {(u, v): best_util_map[u][v] for u, v, in self.__links}
        return loads, waypoints, best_objective

    def solve(self) -> dict:
        """ compute solution """

        t_start = time.time()  # sys wide time
        pt_start = time.process_time()  # count process time (e.g. sleep excluded and count per core)
        loads, waypoints, objective, waypoints_demand = self.__demands_first_waypoints()
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

class kWPJOINTHeur(GenericSR):

    seed = 42

    def __init__(self, nodes: list, links: list, demands: list, weights: dict = None, k: int = 1, **kwargs):
        super().__init__(nodes, links, demands, weights, None)

        self.__nodes = nodes  # [i, ..., n-1]
        self.__links = links  # [(i, j, capacity), ...]
        self.__demands = demands  # [(src, dst, demand), ...]
        self.__allowed_waypoints = k

    def solve(self) -> dict:
        """
        sequential combination of two arbitrary sr algorithms to compute
        first optimal weight setting and then waypoints or vice versa
        """

        solution_list=list()
        current_demands = self.__demands

        # route on shortest paths
        heur_ospf = get_algorithm(algorithm_name="heur_ospf_weights",demands=current_demands, nodes=self.__nodes,
                                  links=self.__links)
        solution_ospf = heur_ospf.solve()
        solution_list.append(solution_ospf)


        single_waypoint = kWayPointHeur(nodes=self.__nodes,links=self.__links, demands=current_demands,
                                                 weights=solution_ospf['weights'], k=self.__allowed_waypoints)
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
        return f"k_waypoints"