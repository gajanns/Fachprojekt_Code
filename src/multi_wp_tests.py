import os

from algorithm.segment_routing.multi_waypoints import MultiWayPoints
from demand import dp_factory
from utility import utility
from topology import topology_factory
from utility.json_result_handler import JsonResultWriter
from utility.utility import HIGHLIGHT, CEND, FAIL, error_solution, get_setup_dict, get_fpp

OUT_DIR = os.path.abspath("../out/")
LOG_DIR = os.path.join(OUT_DIR, "log/")

# demands settings
SEED = 318924135
DEMANDS_SAMPLES = 5
ALGORITHM_TIME_OUT = 3600 * 4
ACTIVE_PAIRS_FRACTION = 0.2
K = 4
OC = 3


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

def work(k:int, links, n, demands, setup, res_handler):
    """ Thread worker method: starts a single test instance, i.e.,
        creates algorithm object and solves problem, appends the result to a json file """
    success = False
    result_list=list()

    result_dict = dict()
    result_dict.update(setup)

    try:
        nodes = list(range(n))
        algorithm = None
        solution_list_D = None
        solution_list_C = None
        if OC == 1 or OC == 3:
            algorithm = MultiWayPoints( nodes=nodes, links=links, demands=demands, weights=None,k= k+1, oc=False)
            solution_list_D= algorithm.solve()
        if OC== 2 or OC == 3:
            algorithm = MultiWayPoints(nodes=nodes, links=links, demands=demands, weights=None, k=k+1, oc=True)
            solution_list_C = algorithm.solve()

        test_id = setup["test_idx"]
        for i in range(1,k+1):
            if OC == 1 or OC == 3:
                result_dict=dict()
                result_dict.update(setup)
                result_dict['algorithm'] = str(i)+result_dict['algorithm']
                result_dict['test_idx'] = test_id
                test_id+=1
                result_dict.update(solution_list_D[i])
                result_list.append(result_dict)
                success = True
                print(f"submit test: {result_dict['test_idx']} ({result_dict['topology_name']}, {result_dict['algorithm']}, D_idx = {result_dict['sample_idx']})")

                print(f"Test-ID: {result_dict['test_idx']}, success: {success} [{result_dict['algorithm']}, "
                      f"{result_dict['topology_name']}, {result_dict['sample_idx']}]: objective: {round(result_dict['objective'], 4)}")
            if OC == 2 or OC == 3:
                result_dict = dict()
                result_dict.update(setup)
                result_dict['algorithm'] = str(i) + result_dict['algorithm']+"C"
                result_dict['test_idx'] = test_id
                test_id +=1
                result_dict.update(solution_list_C[i])
                result_list.append(result_dict)
                success = True
                print(
                    f"submit test: {result_dict['test_idx']} ({result_dict['topology_name']}, {result_dict['algorithm']}, D_idx = {result_dict['sample_idx']})")

                print(f"Test-ID: {result_dict['test_idx']}, success: {success} [{result_dict['algorithm']}, "
                      f"{result_dict['topology_name']}, {result_dict['sample_idx']}]: objective: {round(result_dict['objective'], 4)}")

    except Exception as ex:
        err_solution = error_solution()
        result_list.append(err_solution)
        print(f"{HIGHLIGHT}Error on: {setup}\n msg: {str(ex)}{CEND}")

    for result in result_list:
        res_handler.insert_result(result)
    return success, result_list[-1]["objective"]

def k_waypoints_snd():
    """ Sets up tests using topology and demand data from snd_lib. The demand data is scaled with MCF MAXIMAL CONCURRENT
    Each test instance is executed  on 4 heuristic algorithms """


    # topology provider setup
    topology_provider = "snd_lib"
    topologies =['geant']
    topology_generator = get_topology_generator(topology_provider, topologies)

    # demand provider setup
    mcf_method = "maximal_concurrent"

    # setup result handler
    result_filename = os.path.join(OUT_DIR, "results_multi_waypoints.json")
    result_handler = JsonResultWriter(result_filename, overwrite=True)

    test_idx = 0
    for links, n, topology in topology_generator:
        # setup topology specific demand generator and iterate over 10 samples of demands
        demands_generator = get_demands_generator_scaled_snd(n, links.copy(), topology, SEED)
        for demands, demands_provider, sample_idx in demands_generator:
            # perform each test instance on each algorithm

            algorithm = "_waypoints"
            setup = get_setup_dict(algorithm, demands, demands_provider, links, "", n, sample_idx, test_idx,
                                   topology, topology_provider, 1, mcf_method, SEED)
            success, objective = work(K, links.copy(), n, demands.copy(), setup, result_handler)
            if OC == 1 or OC == 2:
                test_idx+=K
            if OC == 3:
                test_idx+=2*K
    return


def main():
    """ For each figure used in the paper we perform a single test-run comprising each multiple test instances """
    # Evaluation Fig. 3
    print(f"Start {HIGHLIGHT}MCF Synthetic Demands - All Topologies{CEND}:")
    #all_topologies_synthetic_demands()

    # Evaluation Fig. 4
    print(f"Start {HIGHLIGHT}MCF Synthetic Demands - All Algorithms - Abilene{CEND}:")
    #abilene_all_algorithms()
    k_waypoints_snd()

    # Evaluation Fig. 5
    print(f"Start {HIGHLIGHT}Scaled Real Demands - Abilene, Geant, Germany50{CEND}:")
    #snd_real_demands()

if __name__ == '__main__':
    main()