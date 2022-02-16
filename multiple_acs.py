from operator import index
import numpy as np
import random
from read_file import Construct_Graph, PathMessage
from ant import Ant
from threading import Thread, Event
from queue import Queue
from concurrent.futures import ThreadPoolExecutor
import copy
import time
from multiprocessing import Process
from multiprocessing import Queue as MPQueue


class MultipleAntColonySystem:
    def __init__(self, graph: Construct_Graph, ants_num=10, alpha = 1, beta=1, q0=0.1):
        super()
        # graph 
        self.graph = graph
        # ants_num
        self.ants_num = ants_num
        # vehicle_capacity 
        self.max_load = graph.vehicle_capacity
        # beta
        self.alpha = alpha
        self.beta = beta
        # q0 
        self.q0 = q0
        # quãng đường tốt nhất
        self.best_path_distance = None
        self.best_path = None
        self.best_vehicle_num = None


    @staticmethod
    def stochastic_accept(index_to_visit, transition_prob):
        """
        ## *Description*:
        Lựa chọn Node tiếp theo để di chuyển theo xác suất

        ## *Args*:
        index_to_visit: Danh sách các node thỏa mãn các ràng buộc. Có thể di chuyển tới các node này.
        transition_prob: Xác suất ban đầu để di chuyển đến từng node

        ## *Return*:
        nearest_ind: node gần nhất thỏa mãn điều kiện
        """
        # Số lượng node thỏa mãn các ràng buộc
        N = len(index_to_visit)

        # chuẩn hóa xác suất
        sum_tran_prob = np.sum(transition_prob)
        norm_transition_prob = transition_prob/sum_tran_prob

        for i in range(1, len(norm_transition_prob)):
            norm_transition_prob[i] += norm_transition_prob[i-1]

        # lựa chọn theo xác xuất node tiếp theo
        r = random.random()
        for i in range(len(norm_transition_prob)):
            if(r<norm_transition_prob[i]):
                return index_to_visit[i]
                

    @staticmethod
    def new_active_ant(ant: Ant, vehicle_num: int, q0: float, alpha:float, beta: float, stop_event: Event):
        """
        ## *Description*:
        Thực hiện quá trình di chuyển của một con kiến

        ## *Args*:
        ant: Kiến thực hiện tìm kiếm
        vehicle_num: Số lượng xe
        q0: Tham số trong lựa chọn xác suất của node tiếp theo
        alpha: Tham số trong công thức lựa chọn xác suất
        beta: Tham số trong công thức lựa chọn xác suất
        stop_event: Sử dụng để dừng quá trình tìm kiếm

        ## *Return*:
        Local update pheromone 
        """

        # Số lượng xe chưa được sử dụng
        unused_depot_count = vehicle_num

        while not ant.index_to_visit_empty() and unused_depot_count > 0:
            if stop_event.is_set():
                return

            # Danh sách các node thỏa mãn ràng buộc
            next_index_meet_constrains = ant.cal_next_index_meet_constrains()

            # Nếu không còn node nào thỏa mãn ràng buộc, quay trở lại kho
            if len(next_index_meet_constrains) == 0:
                ant.move_to_next_index(0)
                unused_depot_count -= 1
                continue

            # Dữ liệu thời gian của từng node thỏa mãn ràng buộc
            length = len(next_index_meet_constrains)
            ready_time = np.zeros(length)
            due_time = np.zeros(length)

            for i in range(length):
                ready_time[i] = ant.graph.nodes[next_index_meet_constrains[i]].ready_time
                due_time[i] = ant.graph.nodes[next_index_meet_constrains[i]].due_time

            # Thời gian xe bắt đầu phục vụ ở node
            delivery_time = np.maximum(ant.vehicle_travel_time + ant.graph.distance_matrix[ant.current_index][next_index_meet_constrains], ready_time)
            # Quãng thời gian kết thúc phục vụ ở node trước cho tới bắt đầu phục vụ ở node sau
            delta_time = delivery_time - ant.vehicle_travel_time
            distance = delta_time * (due_time - ant.vehicle_travel_time)
            

            closeness = 1/distance

            # Công thức tính xác suất 
            transition_prob = np.power(ant.graph.pheromone_mat[ant.current_index][next_index_meet_constrains], alpha) * \
                              np.power(closeness, beta)
            # Chuẩn hóa
            #transition_prob = transition_prob / np.sum(transition_prob)

            if np.random.rand() < q0:
                # Nếu random < q0: chọn node có xác xuất lớn nhất
                max_prob_index = np.argmax(transition_prob)
                next_index = next_index_meet_constrains[max_prob_index]
            else:
                # Nếu không: chọn ngẫu nhiên
                next_index = MultipleAntColonySystem.stochastic_accept(next_index_meet_constrains, transition_prob)
            ant.graph.local_update_pheromone(ant.current_index, next_index)
            ant.move_to_next_index(next_index)

        # Nếu không còn node nào thỏa mãn, quay trở về kho
        if ant.index_to_visit_empty():
            # Local update pheromone
            ant.graph.local_update_pheromone(ant.current_index, 0)
            ant.move_to_next_index(0)

    @staticmethod
    def acs_time(new_graph: Construct_Graph, vehicle_num: int, ants_num: int, q0: float,alpha: float, beta: float,
                 global_path_queue: Queue, path_found_queue: Queue, stop_event: Event):
        """
        ## *Description*:
        Thực hiện quá trình di chuyển của một con kiến

        ## *Args*:
        new_graph: Đồ thị sinh ra từ file đầu vào của bài toán
        vehicle_num: Số lượng xe  
        ants_num: Số lượng kiến
        q0: Tham số trong lựa chọn xác suất của node tiếp theo
        alpha: Tham số trong công thức lựa chọn xác suất
        beta: Tham số trong công thức lựa chọn xác suất
        global_path_queue: Hàng đợi chưa các quãng đường tốt nhất đã tìm được của bài toán
        path_found_queue: Hàng đợi chứa các quãng đường tốt nhất kiến tìm đường
        stop_event: Sử dụng để dừng quá trình tìm kiếm

        ## *Return*:
        Tìm các tuyến đường tối ưu
        Local và global update pheromone 
        """

        global_best_path = None
        global_best_distance = None
        ants_pool = ThreadPoolExecutor(ants_num)
        ants_thread = []
        ants = []
        while True:

            # Nếu nhận được tín hiệu dừng (hết thời gian chạy) thì dừng vòng lặp
            if stop_event.is_set():
                return

            # Tạo hành trình của đàn kiến. Mỗi hành trình thực hiện trên 1 thread.
            for k in range(ants_num):
                ant = Ant(new_graph, 0)
                thread = ants_pool.submit(MultipleAntColonySystem.new_active_ant, ant, vehicle_num, q0, alpha, beta, stop_event)
                ants_thread.append(thread)
                ants.append(ant)

            for thread in ants_thread:
                thread.result()

            # Đường đi ngắn nhất mà kiến tìm được
            ant_best_path = None
            # Khoảng cách của quãng đường ngắn nhất mà kiến tìm được
            ant_best_travel_distance = None

            for ant in ants:
                # Nếu nhận được tín hiệu dừng (hết thời gian chạy) thì dừng vòng lặp
                if stop_event.is_set():
                    return

                # Lấy thông tin tối ưu toàn cục nhất hiện tại 
                if not global_path_queue.empty():
                    info = global_path_queue.get()
                    while not global_path_queue.empty():
                        info = global_path_queue.get()
                    global_best_path, global_best_distance, global_used_vehicle_num = info.get_path_info()

                # Nếu quãng đường kiến tìm được có khoảng cách tốt hơn quãng đường tối ưu hiện tại kiến tìm được
                # Cập nhập quãng đường tối ưu mới
                if ant.index_to_visit_empty() and (ant_best_travel_distance is None or ant.total_travel_distance < ant_best_travel_distance):
                    ant_best_travel_distance = ant.total_travel_distance
                    ant_best_path = ant.travel_path

            #globle update pheromone 
            new_graph.global_update_pheromone(global_best_path, global_best_distance)

            # Nếu quãng đường kiến tìm được nhỏ hơn quãng đường tối ưu hiện tại
            if ant_best_travel_distance is not None and ant_best_travel_distance < global_best_distance:
                path_found_queue.put(PathMessage(ant_best_path, ant_best_travel_distance))

            # clear ant và các luồng
            ants_thread.clear()
            for ant in ants:
                ant.clear()
                del ant
            ants.clear()

    @staticmethod
    def acs_vehicle(new_graph: Construct_Graph, vehicle_num: int, ants_num: int, q0: float, alpha: float, beta: float,
                    global_path_queue: Queue, path_found_queue: Queue, stop_event: Event):
        """
        ## *Description*:
        Thực hiện quá trình di chuyển của một con kiến

        ## *Args*:
        new_graph: Đồ thị sinh ra từ file đầu vào của bài toán
        vehicle_num: Số lượng xe  
        ants_num: Số lượng kiến
        q0: Tham số trong lựa chọn xác suất của node tiếp theo
        beta: Tham số trong công thức cập nhập mùi
        global_path_queue: Hàng đợi chưa các quãng đường tốt nhất đã tìm được của bài toán
        path_found_queue: Hàng đợi chứa các quãng đường tốt nhất kiến tìm đường
        stop_event: Sử dụng để dừng quá trình tìm kiếm

        ## *Return*:
        Tìm các tuyến đường tối ưu
        Local và global update pheromone 
        """
        global_best_path = None
        global_best_distance = None

        # Khởi tạo đường đi hiện tại từ kết quả của phương pháp nearest neighbor heuristic
        current_path, current_path_distance, _ = new_graph.nearest_neighbor_heuristic(max_vehicle_num=vehicle_num)

        current_index_to_visit = list(range(new_graph.node_num))
        for ind in set(current_path):
            current_index_to_visit.remove(ind)

        ants_pool = ThreadPoolExecutor(ants_num)
        ants_thread = []
        ants = []
        while True:

            # Nếu nhận được tín hiệu dừng (hết thời gian chạy) thì dừng vòng lặp
            if stop_event.is_set():
                return

            # Tạo hành trình của đàn kiến. Mỗi hành trình thực hiện trên 1 thread.
            for k in range(ants_num):
                ant = Ant(new_graph, 0)
                thread = ants_pool.submit(MultipleAntColonySystem.new_active_ant, ant, vehicle_num, q0, alpha,
                                          beta, stop_event)

                ants_thread.append(thread)
                ants.append(ant)

            for thread in ants_thread:
                thread.result()

            for ant in ants:

                # Nếu nhận được tín hiệu dừng (hết thời gian chạy) thì dừng vòng lặp
                if stop_event.is_set():
                    return

                if len(ant.index_to_visit) < len(current_index_to_visit):
                    current_path = copy.deepcopy(ant.travel_path)
                    current_index_to_visit = copy.deepcopy(ant.index_to_visit)
                    current_path_distance = ant.total_travel_distance

                    if ant.index_to_visit_empty():
                        path_found_queue.put(PathMessage(ant.travel_path, ant.total_travel_distance))

            new_graph.global_update_pheromone(current_path, current_path_distance)

            # Lấy thông tin tối ưu toàn cục nhất hiện tại
            if not global_path_queue.empty():
                info = global_path_queue.get()
                while not global_path_queue.empty():
                    info = global_path_queue.get()
                global_best_path, global_best_distance, global_used_vehicle_num = info.get_path_info()

            # Global update pheromone 
            new_graph.global_update_pheromone(global_best_path, global_best_distance)

            # clear ant và các luồng
            ants_thread.clear()
            for ant in ants:
                ant.clear()
                del ant
            ants.clear()

    def run_multiple_ant_colony_system(self, file_to_write_path="./result.txt"):
        """
        ## *Description*:
        Chạy thuật toán Ant Colony Optimization

        ## *Args*:
        file_to_write_path: Đường dẫn đến file kết quả

        ## *Return*:
        Viết kết quả của thuật toán ra file
        """
        multiple_ant_colony_system_thread = Process(target=self._multiple_ant_colony_system, args=(file_to_write_path, ))
        multiple_ant_colony_system_thread.start()

        multiple_ant_colony_system_thread.join()

    def _multiple_ant_colony_system(self, file_to_write_path=None):
        """
        ## *Description*:
        Chạy thuật toán Ant Colony Optimization

        ## *Args*:
        file_to_write_path: Đường dẫn đến file kết quả

        ## *Return*:
        Viết kết quả của thuật toán ra file
        """

        # file ghi kết quả
        if file_to_write_path is not None:
            file_to_write = open(file_to_write_path, 'w')
        else:
            file_to_write = None

        start_time_total = time.time()

        
        global_path_to_acs_time = Queue()
        global_path_to_acs_vehicle = Queue()

        
        path_found_queue = Queue()

        
        self.best_path, self.best_path_distance, self.best_vehicle_num = self.graph.nearest_neighbor_heuristic()

        while True:

            global_path_to_acs_vehicle.put(PathMessage(self.best_path, self.best_path_distance))
            global_path_to_acs_time.put(PathMessage(self.best_path, self.best_path_distance))

            stop_event = Event()

            # Chạy acs_vehicle
            graph_for_acs_vehicle = self.graph.copy(self.graph.init_pheromone_val)
            acs_vehicle_thread = Thread(target=MultipleAntColonySystem.acs_vehicle,
                                        args=(graph_for_acs_vehicle, self.best_vehicle_num-1, self.ants_num, self.q0, self.alpha,
                                              self.beta, global_path_to_acs_vehicle, path_found_queue, stop_event))

            # Chạy acs_time
            graph_for_acs_time = self.graph.copy(self.graph.init_pheromone_val)
            acs_time_thread = Thread(target=MultipleAntColonySystem.acs_time,
                                     args=(graph_for_acs_time, self.best_vehicle_num, self.ants_num, self.q0, self.alpha, self.beta,
                                           global_path_to_acs_time, path_found_queue, stop_event))

            acs_vehicle_thread.start()
            acs_time_thread.start()

            best_vehicle_num = self.best_vehicle_num

            while acs_vehicle_thread.is_alive() and acs_time_thread.is_alive():

                # Kết quả cuối cùng
                given_time = 1
                if time.time() - start_time_total > 60 * given_time:
                    stop_event.set()
                    self.print_and_write_in_file(file_to_write, '*' * 50)
                    self.print_and_write_in_file(file_to_write, 'Time is up: solution in given time(%d minutes)' % given_time)
                    self.print_and_write_in_file(file_to_write, 'It takes %0.3f second from multiple_ant_colony_system running' % (time.time()-start_time_total))
                    self.print_and_write_in_file(file_to_write, 'The best path have found is:')
                    
                    # Viết các tuyến đường trong lời giải tìm được vào file result
                    count_vehice = 0
                    for i in range(len(self.best_path)):
                        if self.best_path[i] == 0:
                            if i>0:
                                self.print_and_write_in_file(file_to_write, msg + '0\n')
                            count_vehice += 1
                            msg = ''
                            msg += 'Vehicle ' + str(count_vehice)  + ': 0 '
                        else:
                            msg += str(self.best_path[i]) + ' '
                    

                    self.print_and_write_in_file(file_to_write, 'Best path distance is %f, best vehicle_num is %d' % (self.best_path_distance, self.best_vehicle_num))
                    self.print_and_write_in_file(file_to_write, '*' * 50)

                    if file_to_write is not None:
                        file_to_write.flush()
                        file_to_write.close()
                    return

                if path_found_queue.empty():
                    continue

                path_info = path_found_queue.get()
                found_path, found_path_distance, found_path_used_vehicle_num = path_info.get_path_info()
                while not path_found_queue.empty():
                    path, distance, vehicle_num = path_found_queue.get().get_path_info()

                    if distance < found_path_distance:
                        found_path, found_path_distance, found_path_used_vehicle_num = path, distance, vehicle_num

                    if vehicle_num < found_path_used_vehicle_num:
                        found_path, found_path_distance, found_path_used_vehicle_num = path, distance, vehicle_num

                # Nếu tìm được lời giải có quãng đường di chuyển ngắn hơn lời giải tối ưu hiện tịa thì cập nhập lời giải
                if found_path_distance < self.best_path_distance:
                    if file_to_write is not None:
                        file_to_write.flush()

                    self.best_path = found_path
                    self.best_vehicle_num = found_path_used_vehicle_num
                    self.best_path_distance = found_path_distance

                    global_path_to_acs_vehicle.put(PathMessage(self.best_path, self.best_path_distance))
                    global_path_to_acs_time.put(PathMessage(self.best_path, self.best_path_distance))

                # Nếu tìm được lời giải sử dụng ít phương tiện hơn lời giải tối ưu hiện tại thì cập nhập lời giải.
                if found_path_used_vehicle_num < best_vehicle_num:
                    if file_to_write is not None:
                        file_to_write.flush()

                    self.best_path = found_path
                    self.best_vehicle_num = found_path_used_vehicle_num
                    self.best_path_distance = found_path_distance

                    stop_event.set()

    @staticmethod
    def print_and_write_in_file(file_to_write=None, message='default message'):
        if file_to_write is None:
            print(message)
        else:
            print(message)
            file_to_write.write(str(message)+'\n')
