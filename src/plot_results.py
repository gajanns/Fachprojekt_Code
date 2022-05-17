import plot_topo_kwp_jointheur_test
import plot_kwpo_jointheur_results
import plot_nodes_kwp_jointheur_test

def main():

    print("Execute Plotter for ALG1: kWPO-JoinHeur\n")
    plot_kwpo_jointheur_results.main()
    print("Finished ALG1-Plots.\n")

    print("Execute Plotter for ALG2: Topo_kWP-JoinHeur\n")
    plot_topo_kwp_jointheur_test.main()
    print("Finished ALG2-Plots.\n")

    print("Execute Plotter for ALG3: Nodes_kWP-JoinHeur\n")
    plot_nodes_kwp_jointheur_test.main()
    print("Finished ALG3-Plots.\n")


if __name__ == '__main__':
    main()