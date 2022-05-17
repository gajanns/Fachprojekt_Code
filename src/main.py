import topo_kwp_jointheur_test
import kwpo_jointheur_tests
import nodes_kwp_jointheur_test



def main():

    print("Execute Testsuite for ALG1: kWPO-JoinHeur\n")
    kwpo_jointheur_tests.main()
    print("Finished ALG1.\n")

    print("Execute Testsuite for ALG2: Topo_kWP-JoinHeur\n")
    topo_kwp_jointheur_test.main()
    print("Finished ALG2.\n")

    print("Execute Testsuite for ALG3: Nodes_kWP-JoinHeur\n")
    nodes_kwp_jointheur_test.main()
    print("Finished ALG3.\n")


if __name__ == '__main__':
    main()
