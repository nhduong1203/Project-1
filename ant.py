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
    index_to_visit: Danh sách node thỏa mãn các điều kiện ràng buộc\n
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

    "Thực hiện local search"
    @staticmethod
    def local_search_once(graph: Construct_Graph, travel_path: list, travel_distance: float, i_start, stop_event: Event):

        depot_ind = []
        for ind in range(len(travel_path)):
            if graph.nodes[travel_path[ind]].is_depot:
                depot_ind.append(ind)

        for i in range(i_start, len(depot_ind)):
            for j in range(i + 1, len(depot_ind)):

                if stop_event.is_set():
                    return None, None, None

                for start_a in range(depot_ind[i - 1] + 1, depot_ind[i]):
                    for end_a in range(start_a, min(depot_ind[i], start_a + 6)):
                        for start_b in range(depot_ind[j - 1] + 1, depot_ind[j]):
                            for end_b in range(start_b, min(depot_ind[j], start_b + 6)):
                                if start_a == end_a and start_b == end_b:
                                    continue
                                new_path = []
                                new_path.extend(travel_path[:start_a])
                                new_path.extend(travel_path[start_b:end_b + 1])
                                new_path.extend(travel_path[end_a:start_b])
                                new_path.extend(travel_path[start_a:end_a])
                                new_path.extend(travel_path[end_b + 1:])

                                depot_before_start_a = depot_ind[i - 1]

                                depot_before_start_b = depot_ind[j - 1] + (end_b - start_b) - (end_a - start_a) + 1
                                if not graph.nodes[new_path[depot_before_start_b]].is_depot:
                                    raise RuntimeError('error')

                                success_route_a = False
                                check_ant = Ant(graph, new_path[depot_before_start_a])
                                for ind in new_path[depot_before_start_a + 1:]:
                                    if check_ant.check_condition(ind):
                                        check_ant.move_to_next_index(ind)
                                        if graph.nodes[ind].is_depot:
                                            success_route_a = True
                                            break
                                    else:
                                        break

                                check_ant.clear()
                                del check_ant

                                success_route_b = False
                                check_ant = Ant(graph, new_path[depot_before_start_b])
                                for ind in new_path[depot_before_start_b + 1:]:
                                    if check_ant.check_condition(ind):
                                        check_ant.move_to_next_index(ind)
                                        if graph.nodes[ind].is_depot:
                                            success_route_b = True
                                            break
                                    else:
                                        break
                                check_ant.clear()
                                del check_ant

                                if success_route_a and success_route_b:
                                    new_path_distance = Ant.cal_total_travel_distance(graph, new_path)
                                    if new_path_distance < travel_distance:

                                        for temp_ind in range(1, len(new_path)):
                                            if graph.nodes[new_path[temp_ind]].is_depot and graph.nodes[new_path[temp_ind - 1]].is_depot:
                                                new_path.pop(temp_ind)
                                                break
                                        return new_path, new_path_distance, i
                                else:
                                    new_path.clear()

        return None, None, None

    def local_search_procedure(self, stop_event: Event):

        new_path = copy.deepcopy(self.travel_path)
        new_path_distance = self.total_travel_distance
        times = 100
        count = 0
        i_start = 1
        while count < times:
            temp_path, temp_distance, temp_i = Ant.local_search_once(self.graph, new_path, new_path_distance, i_start, stop_event)
            if temp_path is not None:
                count += 1

                del new_path, new_path_distance
                new_path = temp_path
                new_path_distance = temp_distance

                i_start = (i_start + 1) % (new_path.count(0)-1)
                i_start = max(i_start, 1)
            else:
                break

        self.travel_path = new_path
        self.total_travel_distance = new_path_distance
