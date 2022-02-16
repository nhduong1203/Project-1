import numpy as np
import copy
from read_file import Construct_Graph
from threading import Event


class Ant:
    """
    ## *Description*:
    Kiến có thể di chuyển trên đồ thị như một phương tiện, dựa trên những thông tin về mùi để tìm ra đường đi tốt nhất.

    ## *Args*:
    graph: Đồ thị từ lớp Construct_Graph\n
    start_index: node bắt đầu\n
    current_index: node hiện tại kiến đang ở\n
    vehicle_load: Tải trọng hiện tại của kiến\n
    vehicle_travel_time: Thời gian kiến di chuyển\n
    travel_path: Danh sách địa điểm mà kiến đã di chuyển qua\n
    arrivel_time: Danh sách thời gian kiến ghé thăm các địa điểm trên path\n
    index_to_visit: Danh sách node chưa thăm\n
    total_travel_distance: Tổng quãng đường di chuyển của kiến\n


    """
    def __init__(self, graph: Construct_Graph, start_index=0):
        super()
        self.graph = graph
        self.current_index = start_index
        self.vehicle_load = 0
        self.vehicle_travel_time = 0
        self.travel_path = [start_index]
        self.arrival_time = [0]

        self.index_to_visit = list(range(graph.node_num))
        self.index_to_visit.remove(start_index)

        self.total_travel_distance = 0

    "clear path"
    def clear(self):
        """
        ## *Description*:
        Xóa quãng đường di chuyển của kiến
        """
        self.travel_path.clear()
        self.index_to_visit.clear()

    "Kiến di chuyển tới node tiếp theo"
    def move_to_next_index(self, next_index):
        """
        ## *Description*:
        Di chuyển tới node tiếp theo

        ## *Args*:
        next_index: node tiếp theo kiến cần di chuyển tới
        """
        # Cập nhập node tiếp theo và tổng quãng đường di chuyển
        self.travel_path.append(next_index)
        self.total_travel_distance += self.graph.distance_matrix[self.current_index][next_index]

        # Cập nhập thời gian di chuyển
        dist = self.graph.distance_matrix[self.current_index][next_index]
        self.arrival_time.append(self.vehicle_travel_time + dist)

        if self.graph.nodes[next_index].is_depot: # Nếu node đi tới là node Kho
            # Reset vehicle load và travel time về 0
            self.vehicle_load = 0
            self.vehicle_travel_time = 0
        else: # Nếu node đi tới là node khách hàng
            # Tăng thêm capacity dựa trên nhu cầu khách hàng
            self.vehicle_load += self.graph.nodes[next_index].demand
            # Tăng thêm thời gian di chuyển
            self.vehicle_travel_time += dist + max(self.graph.nodes[next_index].ready_time - self.vehicle_travel_time - dist, 0) + self.graph.nodes[next_index].service_time
            # Loại node vừa đi tới ra khỏi danh sách node chưa thăm
            self.index_to_visit.remove(next_index)

        # Cập nhập node hiện tại
        self.current_index = next_index

    def index_to_visit_empty(self):
        """
        ## *Description*:
        Kiểm tra danh sách các node thỏa mãn ràng buộc

        ## *Return*:
        False: Nếu danh sách trống
        True: Nếu danh sách có phần tử
        """
        return len(self.index_to_visit) == 0

    def get_active_vehicles_num(self):
        """
        ## *Description*:
        Kiểm tra số lượng xe chưa được sử dụng

        ## *Return*:
        Số lượng xe chưa được sử dụng
        """
        return self.travel_path.count(0)-1

    # Kiểm tra điều kiện của node tiếp theo
    def check_condition(self, next_index) -> bool:
        """
        ## *Description*:
        Kiểm tra node tiếp theo có thỏa mãn các điều kiện ràng buộc không

        ## *Args*:
        next_index: node cần kiểm tra

        ## *Return*:
        False: Nếu node không thỏa mãn các ràng buộc
        True: Nếu node thỏa mãn các ràng buộc
        """
        # Kiểm tra điều kiện về capacity 
        if self.vehicle_load + self.graph.nodes[next_index].demand > self.graph.vehicle_capacity:
            return False

        # Kiểm tra điều kiện về thời gian
        dist = self.graph.distance_matrix[self.current_index][next_index]
        wait_time = max(self.graph.nodes[next_index].ready_time - self.vehicle_travel_time - dist, 0)
        service_time = self.graph.nodes[next_index].service_time

        # Nếu node đi tới dẫn đến việc không về kịp kho
        if self.vehicle_travel_time + dist + wait_time + service_time + self.graph.distance_matrix[next_index][0] > self.graph.nodes[0].due_time:
            return False

        # Nếu không kịp thời gian đi tới node này
        if self.vehicle_travel_time + dist > self.graph.nodes[next_index].due_time:
            return False

        return True

    "Danh sách các node thỏa mãn ràng buộc"
    def cal_next_index_meet_constrains(self):
        """
        ## *Description*:
        Lấy danh sách các node thỏa mãn ràng buộc

        ## *Return*:
        Danh sách các node thỏa mãn ràng buộc
        """
        
        next_index_meet_constrains = []
        for next_ind in self.index_to_visit:
            if self.check_condition(next_ind):
                next_index_meet_constrains.append(next_ind)
        return next_index_meet_constrains

    "Tìm node gần nhất với node hiện tại" 
    def cal_nearest_next_index(self, next_index_list):
        """
        ## *Description*:
        Tìm node gần nhất so với node hiện tại thỏa mãn các ràng buộc

        ## *Args*:
        next_index_list: Tập các node thỏa mãn các điều kiện ràng buộc

        ## *Return*:
        nearest_ind: node gần nhất thỏa mãn điều kiện
        """

        current_ind = self.current_index

        nearest_ind = next_index_list[0]
        min_dist = self.graph.distance_matrix[current_ind][next_index_list[0]]

        for next_ind in next_index_list[1:]:
            dist = self.graph.distance_matrix[current_ind][next_ind]
            if dist < min_dist:
                min_dist = dist
                nearest_ind = next_ind

        return nearest_ind

    "Tính tổng quãng đường di chuyển"
    @staticmethod
    def cal_total_travel_distance(graph: Construct_Graph, travel_path):
        """
        ## *Description*:
        Tính tổng quãng đường di chuyển của kiến

        ## *Args*:
        graph: Đồ thị thuộc lớp đồ thị Construct_Graph\n
        travel_path: Đường đi của kiến

        ## *Return*:
        distance: Tổng quãng đường kiến đã di chuyển
        """
        distance = 0
        current_ind = travel_path[0]
        for next_ind in travel_path[1:]:
            distance += graph.distance_matrix[current_ind][next_ind]
            current_ind = next_ind
        return distance

    
