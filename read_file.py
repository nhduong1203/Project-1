import numpy as np
import copy

class node:
    """
    ## *Description*: 
    node: Các địa điểm bao gồm địa chỉ khách hàng và kho

    ## *Args*:
    Mỗi địa điểm bao gồm các thuộc tính:
    id: Số thứ tự của node\n
    x, y: Tọa độ của node\n
    is_depot: Có phải là kho hay không: (True:kho); (False:khách hàng)\n
    demand: Dung tích của yêu cầu giao hàng\n
    ready_time: Thời gian bắt đầu của node\n
    due_time: Thời gian kết thúc của node\n
    service_time: thời gian xe cần phục vụ ở node\n
    """
    # khởi tạo thông tin của một node
    def __init__(self, id:  int, x: float, y: float, demand: float, ready_time: float, due_time: float, service_time: float):
        super()
        self.id = id

        if id == 0:
            self.is_depot = True
        else:
            self.is_depot = False

        self.x = x
        self.y = y
        self.demand = demand
        self.ready_time = ready_time
        self.due_time = due_time
        self.service_time = service_time


"Xây dựng đồ thị dựa trên file dữ liệu đầu vào"
class Construct_Graph:
    """
    ## *Description*: 
    Xây dựng đồ thị dựa trên file dữ liệu đầu vào\n
    Xây dựng đồ thị pheromone\n

    ## *Args*: 
    file_path: Đường dẫn của file dữ liệu đầu vào\n
    rho: Hệ số cập nhập mùi trên đồ thị của pheromone\n
    """
    def __init__(self, file_path, rho=0.1):
        super()
        # đọc dữ liệu từ file đầu vào
        self.node_num, self.nodes, self.distance_matrix, self.vehicle_num, self.vehicle_capacity \
            = self.read_file(file_path)
        self.rho = rho

        # khởi tạo giá trị pheromone 
        self.nnh_travel_path, self.init_pheromone_val, _ = self.nearest_neighbor_heuristic()
        self.init_pheromone_val = 1/(self.init_pheromone_val * self.node_num)

        self.pheromone_mat = np.ones((self.node_num, self.node_num)) * self.init_pheromone_val
        self.heuristic_info_mat = 1 / self.distance_matrix

    def read_file(self, file_path):
        """
        ## *Description*:
        Đọc các dữ liệu từ file đầu vào

        ## *Args*:
        file_path: đường dẫn đến file đầu vào

        ## *Return*:
        Khởi tạo các giá trị về số lượng Node, thông tin các Node, số lượng xe, sức chứa các xe.
        """
        # Đọc thông tin đầu vào, khởi tạo các node
        list_nodes = []
        with open(file_path, 'rt') as f:
            count = 1
            for line in f:
                if count == 5:
                    vehicle_num, vehicle_capacity = line.split()
                    vehicle_num = int(vehicle_num)
                    vehicle_capacity = int(vehicle_capacity)
                elif count >= 10:
                    list_nodes.append(line.split())
                count += 1
        node_num = len(list_nodes)
        # Danh sách các node với thông tin đầy đủ của từng node
        nodes = list(node(int(item[0]), float(item[1]), float(item[2]), float(item[3]), float(item[4]), float(item[5]), float(item[6])) for item in list_nodes)

        # Xây dựng ma trận khoảng cách
        distance = np.zeros((node_num, node_num))
        for i in range(node_num):
            node_a = nodes[i]
            distance[i][i] = 1e-8
            for j in range(i+1, node_num):
                node_b = nodes[j]
                distance[i][j] = Construct_Graph.calculate_dist(node_a, node_b)
                distance[j][i] = distance[i][j]

        return node_num, nodes, distance, vehicle_num, vehicle_capacity

    "Công thức tính khoảng cách giữa 2 điểm trên đồ thị theo Khoảnh cách Euclid"
    @staticmethod
    def calculate_dist(node_a, node_b):
        """
        ## *Description*:
        Tính khoảng cách giữa 2 node

        ## *Args*:
        node_a: node thứ nhất\n
        node_b: node thứ hai\n

        ## *Return*:
        Khoảng cách giữa 2 node theo công thức khoảng cách Euclid
        """
        return np.linalg.norm((node_a.x - node_b.x, node_a.y - node_b.y))

    "Công thức local update pheromone matrix"
    def local_update_pheromone(self, start_ind, end_ind):
        """
        ## *Description*:
        Local update pheromone matrix\n


        ## *Args*:
        start_ind: Địa chỉ node thứ nhất\n
        end_ind: Địa chỉ node thứ hai\n
        """
        self.pheromone_mat[start_ind][end_ind] = (1-self.rho) * self.pheromone_mat[start_ind][end_ind] + \
                                                  self.rho * self.init_pheromone_val

    "Công thức global update pheromone matrix"
    def global_update_pheromone(self, best_path, best_path_distance):
        """
        ## *Description*:
        Global update pheromone matrix\n


        ## *Args*:
        best_path: Thông tin đường đi đàn kiến tìm được\n
        best_path_distance: Tổng khoảng cách trên đường đi best_path\n
        """
        self.pheromone_mat = (1-self.rho) * self.pheromone_mat

        current_ind = best_path[0]
        for next_ind in best_path[1:]:
            self.pheromone_mat[current_ind][next_ind] += self.rho/best_path_distance
            current_ind = next_ind

    def copy(self, init_pheromone_val):
        """
        ## *Description*:
        Tạo một bản sao đồ thị mới\n


        ## *Args*:
        init_pheremone_value: Giá trị pheromone khởi tạo cho đồ thị mới\n

        ## *Return*
        Đồ thị mới là bản sao của đồ thị cũ, có giá trị pheromone mới
        """
        new_graph = copy.deepcopy(self)

        new_graph.init_pheromone_val = init_pheromone_val
        new_graph.pheromone_mat = np.ones((new_graph.node_num, new_graph.node_num)) * init_pheromone_val

        return new_graph

    """
    Đáp án của bài toán theo phương pháp nearest neighbor heuristic
    """
    def nearest_neighbor_heuristic(self, max_vehicle_num=None):
        """
        ## *Description*:
        Đáp án của bài toán theo phương pháp nearest neighbor heuristic

        ## *Args*:
        max_vehicle_num: Số lượng xe tối đa

        ## *Return*:
        travel_path: Đường đi theo kết quả của phương pháp nearest neighbor heuristic
        travel_distance: Tổng quãng đường theo travel_path
        vehicle_num: Số lượng xe sử dụng
        """
        index_to_visit = list(range(1, self.node_num))
        current_index = 0
        current_load = 0
        current_time = 0
        travel_distance = 0
        travel_path = [0]

        if max_vehicle_num is None:
            max_vehicle_num = self.node_num

        while len(index_to_visit) > 0 and max_vehicle_num > 0:
            # node gần nhất thỏa mãn ràng buộc
            nearest_next_index = self.nearest_node(index_to_visit, current_index, current_load, current_time)

            if nearest_next_index is None:
                # nếu không có node nào thỏa mãn ràng buộc, trở về kho
                travel_distance += self.distance_matrix[current_index][0]

                current_load = 0
                current_time = 0
                travel_path.append(0)
                current_index = 0

                max_vehicle_num -= 1
            else:
                # nếu có node thỏa mãn: đi tới node gần nhất thỏa mãn
                # cập nhập các thông số về khoảng cách, load, thời gian
                current_load += self.nodes[nearest_next_index].demand

                dist = self.distance_matrix[current_index][nearest_next_index]
                wait_time = max(self.nodes[nearest_next_index].ready_time - current_time - dist, 0)
                service_time = self.nodes[nearest_next_index].service_time

                current_time += dist + wait_time + service_time
                index_to_visit.remove(nearest_next_index)

                travel_distance += self.distance_matrix[current_index][nearest_next_index]
                travel_path.append(nearest_next_index)
                current_index = nearest_next_index
        travel_distance += self.distance_matrix[current_index][0]
        travel_path.append(0)

        vehicle_num = travel_path.count(0)-1
        return travel_path, travel_distance, vehicle_num

    "Tìm node gần nhất so với node hiện tại thỏa mãn các ràng buộc"
    def nearest_node(self, index_to_visit, current_index, current_load, current_time):
        """
        ## *Description*:
        Tìm node gần nhất so với node hiện tại thỏa mãn các ràng buộc

        ## *Args*:
        index_to_visit: Tập các node thỏa mãn các điều kiện ràng buộc
        current_index: node hiện tại
        current_load: Dung tích hiện tại xe đang chở
        current_time: Thời gian hiện tại xe đã di chuyển

        ## *Return*:
        nearest_index: node gần nhất thỏa mãn điều kiện
        """
        nearest_ind = None
        nearest_distance = None
        for next_index in index_to_visit:
            # Kiểm tra điều kiện Capacity
            if current_load + self.nodes[next_index].demand > self.vehicle_capacity:
                continue

            # Kiểm tra điều kiện thời gian
            dist = self.distance_matrix[current_index][next_index]
            wait_time = max(self.nodes[next_index].ready_time - current_time - dist, 0)
            service_time = self.nodes[next_index].service_time
            # Nếu đi tới node này sẽ không thể quay về kho kịp thời gian
            if current_time + dist + wait_time + service_time + self.distance_matrix[next_index][0] > self.nodes[0].due_time:
                continue
            
            # Nếu không thể tới node này kịp thời gian
            if current_time + dist > self.nodes[next_index].due_time:
                continue

            if nearest_distance is None or self.distance_matrix[current_index][next_index] < nearest_distance:
                nearest_distance = self.distance_matrix[current_index][next_index]
                nearest_ind = next_index

        return nearest_ind


class PathMessage:
    """
    ## *Description*:
    Tạo bản sao của một đường đi

    ## *Args*:
    path: đường đi gốc\n
    distance: tổng quãng đường trên path\n

    """
    def __init__(self, path, distance):
        if path is not None:
            self.path = copy.deepcopy(path)
            self.distance = copy.deepcopy(distance)
            self.used_vehicle_num = self.path.count(0) - 1
        else:
            self.path = None
            self.distance = None
            self.used_vehicle_num = None

    def get_path_info(self):
        return self.path, self.distance, self.used_vehicle_num
