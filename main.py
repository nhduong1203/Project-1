from read_file import Construct_Graph
from multiple_acs import MultipleAntColonySystem


if __name__ == '__main__':
    file_path = './data/200_customer/C1_2_2.txt'
    ants_num = 10
    alpha = 1
    beta = 2
    q0 = 0.1


    graph = Construct_Graph(file_path)
    macs = MultipleAntColonySystem(graph, ants_num=ants_num, alpha=alpha, beta=beta, q0=q0)
    macs.run_multiple_ant_colony_system()
