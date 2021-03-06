import os

from algorithm.segment_routing.kwpo_jointheur import KWPOJointHeur,SortSetting
from demand import dp_factory
from algorithm import sr_factory
from topology import topology_factory
from utility.json_result_handler import JsonResultWriter
from utility.utility import HIGHLIGHT, CEND, FAIL, error_solution, get_setup_dict, get_fpp

OUT_DIR = os.path.abspath("../out/")
LOG_DIR = os.path.join(OUT_DIR, "log/")

# demands settings
SEED = 318924135
DEMANDS_SAMPLES = 10
ALGORITHM_TIME_OUT = 3600 * 4
ACTIVE_PAIRS_FRACTION = 0.2

# number of iterations of greedywpo
K = 4


def get_demands_generator_mcf_maximal(n, links, active_pairs_fraction, seed):
    """ Creates a set of 10 samples of demands fitted to the capacity of the topology with MCF maximal """
    flows_per_pair = get_fpp(links)
    demands_provider = "mcf"
    mcf_method = "maximal"
    mcf_dp = dp_factory.get_demand_provider(
        n=n, provider=demands_provider, number_samples=DEMANDS_SAMPLES, links=links,
        active_pairs_fraction=active_pairs_fraction,
        mcf_method=mcf_method, flows_per_pair=flows_per_pair, seed=seed)
    for sample_idx, demands in enumerate(mcf_dp.demand_sequences()):
        yield demands, demands_provider, sample_idx

def get_demands_generator_scaled_snd(n, links, topology, seed):
    """ Creates a set of 10 samples of demands from sndlib and scales the demand using MCF concurrent """
    # Get demands from snd_lib demand provider
    snd_lib_dp = dp_factory.get_demand_provider(
        provider="snd_lib", topology_name=topology, number_samples=DEMANDS_SAMPLES)
    unscaled_demand_matrices = list(snd_lib_dp.demand_matrices())

    # Scale demands with mcf maximal concurrent
    flows_per_pair = get_fpp(links)
    mcf_dp = dp_factory.get_demand_provider(
        n=n, provider="mcf", number_samples=DEMANDS_SAMPLES, links=links,
        unscaled_demands_sets=unscaled_demand_matrices,
        mcf_method="MAXIMAL_CONCURRENT", flows_per_pair=flows_per_pair, seed=seed)
    for sample_idx, demands in enumerate(mcf_dp.demand_sequences()):
        yield demands, "snd_lib_mcf_scaled", sample_idx


def get_topology_generator(top_provider, tops_names, max_edges=None):
    """ Retrieves topology file data from src top_provider """
    top_provider = topology_factory.get_topology_factory(top_provider)
    for topology_name in tops_names:
        links, n = top_provider.get_topology(topology_name)
        if max_edges and len(links) > max_edges:
            continue
        yield links, n, topology_name


def work(algorithm_name, links, n, demands, ilp_method, setup, time_out, res_handler,
         sortStrat:SortSetting=SortSetting.ByDemandValue):
    """ Thread worker method: starts a single test instance, i.e.,
        creates algorithm object and solves problem, appends the result to a json file """
    success = False
    result_dict = dict()
    result_dict.update(setup)
    try:
        nodes = list(range(n))
        if algorithm_name.startswith("kwpo_jointheur"):
            algorithm = KWPOJointHeur(nodes=nodes, links=links, demands=demands, weights=None,
                                      k=K, sortStrat=sortStrat)
            solutions=algorithm.solve()
            result_dict.update(solutions[K])
        else:
            algorithm = sr_factory.get_algorithm(
                algorithm_name, nodes=nodes, links=links, demands=demands, ilp_method=ilp_method, time_out=time_out)
            solution = algorithm.solve()
            result_dict.update(solution)
        success = True
    except Exception as ex:
        err_solution = error_solution()
        result_dict.update(err_solution)
        print(f"{HIGHLIGHT}Error on: {setup}\n msg: {str(ex)}{CEND}")
    res_handler.insert_result(result_dict)
    return success, result_dict["objective"]

def work_kwpo_cumulative(links, n, demands, ilp_method, setup, time_out, res_handler,
                         sortStrat:SortSetting=SortSetting.ByDemandValue):
    """ Thread worker method: starts multiple kwpo test instances, i.e.,
        creates algorithm object and solves problem, appends the results to a json file """
    success = False
    result_dict_list=list()

    try:
        nodes = list(range(n))
        algorithm = KWPOJointHeur(nodes=nodes, links=links, demands=demands, weights=None,
                                  k=K, sortStrat=sortStrat)
        solutions = algorithm.solve()
        for i,solution in enumerate(solutions):
            result_dict = dict()
            result_dict.update(setup)
            algorithm_name="kwpo_jointheur_" + str(i)
            algorithm_name+="d" if sortStrat==SortSetting.ByDemandValue else "c"
            result_dict["algorithm"]=algorithm_name
            result_dict["test_idx"]+=i
            result_dict.update(solution)
            result_dict_list.append(result_dict)
        success = True
    except Exception as ex:
        err_solution = error_solution()
        result_dict_list.append(err_solution)
        print(f"{HIGHLIGHT}Error on: {setup}\n msg: {str(ex)}{CEND}")
    for result in result_dict_list:
        res_handler.insert_result(result)
    return success, result_dict_list[-1]["objective"]

def kwpo_all_topologies_synthetic_demands():
    """ Sets up tests using all topology data having complete link capacity data from SNDLib.
        For these tests synthetic demands generated with MCF MAXIMAL are used.
        Each test instance is executed on 4 heuristic algorithms """

    # algorithm settings
    algorithms = [
        "demand_first_waypoints",
        "heur_ospf_weights",
        "sequential_combination",
        "kwpo_jointheur_4c"
    ]
    ilp_method = ""

    # topology provider settings
    topology_map = {
        # SNDLib with complete capacity information
         "snd_lib": [
             "abilene",  #: |E|: 30 , |V|: 12
             "polska",  #: |E|: 36 , |V|: 12
             "nobel-us",  #: |E|: 42 , |V|: 14
             "atlanta",  #: |E|: 44 , |V|: 15
             "nobel-germany",  #: |E|: 52 , |V|: 17
             "pdh",  #: |E|: 68 , |V|: 11
             "geant",  #: |E|: 72 , |V|: 22
             "nobel-eu",  #: |E|: 82 , |V|: 28
             "di",  #: |E|: 84 , |V|: 11
             "janos-us",  #: |E|: 84 , |V|: 26
             "dfn-bwin",  #: |E|: 90 , |V|: 10
             "france",  #: |E|: 90 , |V|: 25
        #     "dfn-gwin",  #: |E|: 94 , |V|: 11     # no time for that
        #     "newyork",  #: |E|: 98 , |V|: 16
        #     "norway",  #: |E|: 102, |V|: 27
        #     "sun",  #: |E|: 102, |V|: 27
        #     "ta1",  #: |E|: 102, |V|: 24
        #     "cost266",  #: |E|: 114, |V|: 37
        #     "janos-us-ca",  #: |E|: 122, |V|: 39
        #     "india35",  #: |E|: 160, |V|: 35
        #     "zib54",  #: |E|: 160, |V|: 54
        #     "giul39",  #: |E|: 172, |V|: 39
        #     "germany50",  #: |E|: 176, |V|: 50
        #     "pioro40",  #: |E|: 178, |V|: 40
        #     "ta2",  #: |E|: 216, |V|: 65
        #     "brain",  #: |E|: 332, |V|: 161
         ]
    }

    # demand provider settings
    mcf_method = "maximal"

    result_filename = os.path.join(OUT_DIR, "results_kwpo_all_topologies.json")
    result_handler = JsonResultWriter(result_filename, overwrite=True)

    test_idx = 0
    # for each source of topology (SNDLib and TopologyZoo)
    for topology_provider in topology_map:
        topologies = topology_map[topology_provider]
        topology_generator = get_topology_generator(topology_provider, topologies)
        for links, n, topology in topology_generator:
            # setup topology specific demand generator and iterate over 10 samples of demands
            demands_generator = get_demands_generator_mcf_maximal(n, links.copy(), ACTIVE_PAIRS_FRACTION, SEED)
            for demands, demands_provider, sample_idx in demands_generator:
                # perform each test instance on each algorithm
                for algorithm in algorithms:
                    setup = get_setup_dict(algorithm, demands, demands_provider, links, ilp_method, n, sample_idx,
                                           test_idx,
                                           topology, topology_provider, ACTIVE_PAIRS_FRACTION, mcf_method, SEED)

                    print(f"submit test: {test_idx} ({topology}, {algorithm}, D_idx = {sample_idx})")
                    success, objective = work(algorithm, links.copy(), n, demands.copy(), ilp_method, setup,
                                              ALGORITHM_TIME_OUT, result_handler,sortStrat=SortSetting.ByCapacity)
                    print(f"Test-ID: {test_idx}, success: {success} [{algorithm}, "
                          f"{topology}, {sample_idx}]: objective: {round(objective, 4)}")
                    test_idx += 1
    return

def geant_all_algorithms():
    """ Sets up tests for topology Geant (snd_lib) and synthetic demands using MCF MAXIMAL.
        Each test instance is executed on all available algorithms """

    # algorithm settings
    algorithms = [  # ("algorithm_name", "ilp_method")
        ("demand_first_waypoints", ""),
        ("heur_ospf_weights", ""),
        ("inverse_capacity", ""),
        ("sequential_combination", ""),
        ("uniform_weights", ""),
        ("kwpo_jointheur_4c", "")
    ]

    # topology provider setup
    topology_provider = "snd_lib"
    topologies =['geant']
    topology_generator = get_topology_generator(topology_provider, topologies)

    # demand provider setup
    mcf_method = "maximal_concurrent"

    # setup result handler
    result_filename = os.path.join(OUT_DIR, "results_kwpo_all_algorithms.json")
    result_handler = JsonResultWriter(result_filename, overwrite=True)

    # fetch data for test instance and perform test
    test_idx = 0
    for links, n, topology in topology_generator:
        # setup topology specific demand generator and iterate over 10 samples of demands
        demands_generator = get_demands_generator_mcf_maximal(n, links.copy(), ACTIVE_PAIRS_FRACTION, SEED)
        for demands, demands_provider, sample_idx in demands_generator:
            # perform each test instance on each algorithm
            for algorithm, ilp_method in algorithms:
                setup = get_setup_dict(algorithm, demands, demands_provider, links, ilp_method, n, sample_idx, test_idx,
                                       topology, topology_provider, ACTIVE_PAIRS_FRACTION, mcf_method, SEED)

                print(f"submit test: {test_idx} ({topology}, {algorithm} {ilp_method}, D_idx = {sample_idx})")
                success, objective = work(algorithm, links.copy(), n, demands.copy(), ilp_method, setup,
                                          ALGORITHM_TIME_OUT, result_handler,sortStrat=SortSetting.ByCapacity)
                print(f"Test-ID: {test_idx}, success: {success} [{algorithm} {ilp_method}, "
                      f"{topology}, {sample_idx}]: objective: {round(objective, 4)}")
                test_idx += 1
    return

def kwpo_real_demands():
    """ Sets up tests using topology and demand data from snd_lib. The demand data is scaled with MCF MAXIMAL CONCURRENT
        Each test instance is executed  on GreedyWPO and all kWPO_JointHeur algorithms """

    # algorithm settings
    base_algorithms = [
        "demand_first_waypoints",
    ]
    ilp_method = ""

    # topology provider setup
    topology_provider = "snd_lib"
    topologies = ['geant']  # ['abilene', 'geant', 'germany50'] # no time for that
    topology_generator = get_topology_generator(topology_provider, topologies)

    # demand provider setup
    mcf_method = "maximal_concurrent"

    # setup result handler
    result_filename = os.path.join(OUT_DIR, "results_kwpo_real_demands.json")
    result_handler = JsonResultWriter(result_filename, overwrite=True)

    test_idx = 0
    for links, n, topology in topology_generator:
        # setup topology specific demand generator and iterate over 10 samples of demands
        demands_generator = get_demands_generator_scaled_snd(n, links.copy(), topology, SEED)
        for demands, demands_provider, sample_idx in demands_generator:
            # perform each test instance on each algorithm
            for algorithm in base_algorithms:
                setup = get_setup_dict(algorithm, demands, demands_provider, links, ilp_method, n, sample_idx, test_idx,
                                       topology, topology_provider, 1, mcf_method, SEED)

                print(f"submit test: {test_idx} ({topology}, {algorithm}, D_idx = {sample_idx})")
                success, objective = work(algorithm, links.copy(), n, demands.copy(), ilp_method, setup,
                                          ALGORITHM_TIME_OUT, result_handler)
                print(f"Test-ID: {test_idx}, success: {success} [{algorithm}, "
                      f"{topology}, {sample_idx}]: objective: {round(objective, 4)}")
                test_idx += 1

            setup = get_setup_dict("", demands, demands_provider, links, ilp_method, n, sample_idx, test_idx,
                                   topology, topology_provider, 1, mcf_method, SEED)
            print(f"submit test: {test_idx} ({topology}, kwpo_jointheur_{K}d, D_idx = {sample_idx})")
            success, objective = work_kwpo_cumulative(links.copy(), n, demands.copy(), ilp_method, setup,
                                                      ALGORITHM_TIME_OUT, result_handler)
            print(f"Test-ID: {test_idx}, success: {success} [kwpo_jointheur_{K}d, "
                  f"{topology}, {sample_idx}]: objective: {round(objective, 4)}")
            test_idx += K+1

            setup["test_idx"]=test_idx
            print(f"submit test: {test_idx} ({topology}, kwpo_jointheur_{K}c, D_idx = {sample_idx})")
            success, objective = work_kwpo_cumulative(links.copy(), n, demands.copy(), ilp_method, setup,
                                                      ALGORITHM_TIME_OUT, result_handler,sortStrat=SortSetting.ByCapacity)
            print(f"Test-ID: {test_idx}, success: {success} [kwpo_jointheur_{K}c, "
                  f"{topology}, {sample_idx}]: objective: {round(objective, 4)}")
            test_idx += K
    return

def main():
    """ For each figure used in the paper we perform a single test-run comprising each multiple test instances """
    # Evaluation Fig. 3
    print(f"Start {HIGHLIGHT}MCF Synthetic Demands - All Topologies{CEND}:")
    kwpo_all_topologies_synthetic_demands()

    # Evaluation Fig. 4
    print(f"Start {HIGHLIGHT}MCF Synthetic Demands - All Algorithms - Geant{CEND}:")
    geant_all_algorithms()

    # Evaluation Fig. 5
    print(f"Start {HIGHLIGHT}Scaled Real Demands - Geant{CEND}:")
    kwpo_real_demands()

if __name__ == '__main__':
    main()
