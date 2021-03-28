from collections import defaultdict, deque
from copy import deepcopy
from queue import PriorityQueue
## Taking the graph as input in the form of adjacency matrix
V = int(input("Enter the number of vertices"))
adj = [[0 for i in range(V)] for j in range(V)]
print("Now start entering the edge distances")
for i in range(V):
    for j in range(V):
        inp = input(f"e{i}{j}->")
        if inp == "INF" or inp == "inf":
            adj[i][j] = float("inf")
        else:
            adj[i][j] = float(inp)

## Taking the vehicle information input

start_node = {}
destination_node = {}
battery_status = {}
charging_rate = {}
discharging_rate = {}
Max_battery = {}
avg_speed = {}
K = int(input("Number of Cars"))
for i in range(K):
    start_node[i] = int(input(f"source node of {i + 1}th car")) - 1
    destination_node[i] = int(input(f"destination node of {i + 1}th car")) - 1
    battery_status[i] = float(input(f"initial battery status of {i + 1}th car"))
    charging_rate[i] = float(input(f"charging rate of {i + 1}th car"))
    discharging_rate[i] = float(input(f"discharging rate of {i + 1}th car"))
    Max_battery[i] = float(input(f"Max battery capacity of {i + 1}th car"))
    avg_speed[i] = float(input(f"average speed of {i + 1}th car"))

arrival_time = [[float("inf") for i in range(K)] for i in
                range(V)]  ## a 2D matrix with nodes as row indices and columns as car


## indices representing the arrival time of each car at each node initially initialized to all values as infinity

class car:
    def __init__(self, index, start_node, destination_node, battery_status, charging_rate, discharging_rate,
                 Max_battery, avg_speed, adj):
        self.index = index
        self.start_node = start_node
        self.destination_node = destination_node
        self.battery_status = battery_status
        self.charging_rate = charging_rate
        self.discharging_rate = discharging_rate
        self.Max_battery = Max_battery
        self.avg_speed = avg_speed
        self.path = []
        self.graph = deepcopy(adj)  ## any changes to the modified graph of the concerned car is not reflected in the original adjacency marix
        for i in range(len(adj)):
            for j in range(len(adj)):
                if adj[i][j] != float("inf"):
                    if (adj[i][j] / self.discharging_rate) > self.Max_battery:
                        self.graph[i][j] = float("inf")

        self.edges = []  ## creating a adjacency list
        for i in range(adj.shape[0]):
            temp = []
            for j in range(adj.shape[0]):
                if self.graph[i][j] != float('inf'):
                    temp.append(j)
            self.edges.append(temp)

    def shortest_path(self):  ## finding the shortest path for the current car from source node to destination node
        initial = self.start_node
        shortest_paths = defaultdict(lambda: (None, float("inf")))
        shortest_paths[initial] = (None, 0)
        visited = defaultdict(bool)
        q = PriorityQueue()
        q.put((0, initial))
        while (not q.empty()):
            curr_node = q.get()[1]
            if (visited[curr_node]):
                continue
            visited[curr_node] = True
            for next_node in self.edges[curr_node]:
                if (shortest_paths[curr_node][1] + self.graph[curr_node][next_node] < shortest_paths[next_node][1]):
                    shortest_paths[next_node] = (curr_node, shortest_paths[curr_node][1] + graph[curr_node][next_node])
                    q.put((shortest_paths[next_node][1], next_node))

        # Work back through destinations in shortest path
        current_node = self.destination_node
        while current_node is not None:
            self.path.append(current_node)
            next_node = shortest_paths[current_node][0]
            current_node = next_node
        # Reverse path
        self.path = self.path[::-1]
        self.distance = defaultdict(int)
        for i in range(len(self.path) - 2, -1, -1):
            self.distance[self.path[i]] = self.graph[self.path[i]][self.path[i + 1]] + self.distance[self.path[i + 1]]

        self.min_time = defaultdict(
            int)  ## minimum time required for the concerned car to move from current node to destination node
        temp_battery_status = self.battery_status
        self.charge_time_required = defaultdict(int)
        for i in range(len(self.path)):
            if (i != 0):
                temp_battery_status -= self.graph[self.path[i - 1]][self.path[i]] / self.discharging_rate
            self.min_time[self.path[i]] = self.distance[self.path[i]] / self.avg_speed
            min_charge_required = self.distance[self.path[i]] / self.discharging_rate
            self.charge_time_required[self.path[i]] = 0
            if (temp_battery_status <= min(self.Max_battery, min_charge_required)):
                self.charge_time_required[self.path[i]] = (min(self.Max_battery,
                                                               min_charge_required) - temp_battery_status) / self.charging_rate
                self.min_time[self.path[i]] += self.charge_time_required[self.path[i]]

            arrival_time[self.path[i]][self.index] = 0
            if (i != 0):
                arrival_time[self.path[i]][self.index] = arrival_time[self.path[i - 1]][self.index] + self.min_time[
                    self.path[i - 1]] - self.min_time[self.path[i]]

    def update_parameters(self, node,
                          waiting_time):  ## updates the paarmeters *min_time and the global matrix *arrival_time if there is waiting due to collision
        self.min_time[node] += waiting_time
        update = False
        for i in range(len(self.path)):
            if (update):
                self.min_time[self.path[i]] += waiting_time
                arrival_time[self.path[i]][self.index] += waiting_time
            if (self.path[i] == node):
                update = True


all_cars = {}  ## Creating dictionary of the car index to car objects
for i in range(K):
    obj = car(i, start_node[i], destination_node[i], battery_status[i], charging_rate[i], discharging_rate[i],
              Max_battery[i], avg_speed[i], adj)
    obj.shortest_path()
    all_cars[i] = obj

# make global time list of events for synchronization
global_time_list = []
for curr_node in range(V):
    cars_present = [all_cars[i] for i in range(K) if arrival_time[curr_node][i] != float("inf")]
    charge_time_left_list = [0] * K
    for i in cars_present:
        arr_time = arrival_time[curr_node][i.index]
        dep_time = arr_time + i.charge_time_required[curr_node]
        charge_time_left_list[i.index] = int(i.charge_time_required[curr_node])
        global_time_list.append([i, "arrival", curr_node, arr_time])
        global_time_list.append([i, "departure", curr_node, dep_time])
global_time_list.sort(key=lambda x: x[3])


def schedule(cars_list, prev_charging_car, charge_time_left_list,
             curr_node):  ## Rescheduling the charging queue upon addition of a new car to the charging list
    car_obj_list = [w[0] for w in cars_list]
    if len(car_obj_list) == 1:
        return car_obj_list[0].index
    temp = sorted([sorted(
        [(i, w.min_time[curr_node] + charge_time_left_list[x.index]) for j, w in enumerate(car_obj_list) if j != i],
        key=lambda y: -y[1]) for i, x in enumerate(car_obj_list)], key=lambda y: y[0][1])
    ## Parameter of prioritizing is w.min_time[curr_node]+charge_time_left[x.index]
    return temp[0][0][0]


cars_list_list = [[]] * V  # num nodes X num cars at node
prev_event_time_list = [0] * V  # num nodes
prev_charging_cars_list = [-1] * V  # num nodes
while (len(global_time_list) != 0):

    event = global_time_list.pop(0)

    curr_node = event[2]

    if (event[1] == "departure"):

        popped = False
        index_event_index = -1
        for i in range(len(cars_list_list[curr_node])):
            if (cars_list_list[curr_node][i][0].index == event[0].index):
                index_event_index = i
        waiting_time = event[3] - prev_event_time_list[curr_node]
        prev_event_time_list[curr_node] = int(event[3])

        charge_time_left_list[prev_charging_cars_list[curr_node]] -= waiting_time
        if (prev_charging_cars_list[curr_node] == event[0].index):
            cars_list_list[curr_node].pop(index_event_index)
            popped = True

        if (len(cars_list_list[curr_node]) > 0):
            for i in range(len(cars_list_list[curr_node])):
                if (cars_list_list[curr_node][i][0].index != prev_charging_cars_list[curr_node]):
                    index_list_in_global_time_list = []
                    for idx, listn in enumerate(global_time_list):
                        if listn[0].index == cars_list_list[curr_node][i][0].index and listn[2] == curr_node:
                            index_list_in_global_time_list.append(idx)

                    cars_list_list[curr_node][i][0].update_parameters(curr_node, waiting_time)
                    if index_list_in_global_time_list:
                        for index_in_global_time_list in index_list_in_global_time_list:
                            old_list = global_time_list[index_in_global_time_list]
                            arr_time = arrival_time[curr_node][cars_list_list[curr_node][i][0].index]
                            dep_time = arr_time + cars_list_list[curr_node][i][0].charge_time_required[curr_node]
                            new_list = [cars_list_list[curr_node][i][0], "arrival", old_list[2], arr_time] if old_list[
                                                                                                                  1] == "arrival" \
                                else [cars_list_list[curr_node][i][0], "departure", old_list[2], dep_time]
                            global_time_list[index_in_global_time_list] = new_list
                    global_time_list.sort(key=lambda x: x[3])

        if popped and len(cars_list_list[curr_node]) > 0:
            prev_charging_cars_list[curr_node] = schedule(cars_list_list[curr_node], prev_charging_cars_list[curr_node],
                                                          charge_time_left_list, curr_node)

    else:

        index_event_index = -1
        for i in range(len(cars_list_list[curr_node])):
            if (cars_list_list[curr_node][i][0].index == event[0].index):
                index_event_index = i
        waiting_time = event[3] - prev_event_time_list[curr_node]
        prev_event_time_list[curr_node] = event[3]

        cars_list_list[curr_node].append(event)

        charge_time_left_list[prev_charging_cars_list[curr_node]] -= waiting_time
        if (len(cars_list_list[curr_node]) > 0):  # changing departure time
            for i in range(len(cars_list_list[curr_node])):
                if (cars_list_list[curr_node][i][0].index != prev_charging_cars_list[curr_node]):
                    index_list_in_global_time_list = []
                    for idx, listn in enumerate(global_time_list):
                        if listn[0].index == cars_list_list[curr_node][i][0].index and listn[2] == curr_node:
                            index_list_in_global_time_list.append(idx)

                    cars_list_list[curr_node][i][0].update_parameters(curr_node, waiting_time)
                    if index_list_in_global_time_list:
                        for index_in_global_time_list in index_list_in_global_time_list:
                            old_list = global_time_list[index_in_global_time_list]
                            arr_time = arrival_time[curr_node][cars_list_list[curr_node][i][0].index]
                            dep_time = arr_time + cars_list_list[curr_node][i][0].charge_time_required[curr_node]
                            new_list = [cars_list_list[curr_node][i][0], "arrival", old_list[2], arr_time] if old_list[
                                                                                                                  1] == "arrival" \
                                else [cars_list_list[curr_node][i][0], "departure", old_list[2], dep_time]
                            global_time_list[index_in_global_time_list] = new_list
                    global_time_list.sort(key=lambda x: x[3])

        if len(cars_list_list[curr_node]) > 0:
            prev_charging_cars_list[curr_node] = schedule(cars_list_list[curr_node], prev_charging_cars_list[curr_node],
                                                          charge_time_left_list, curr_node)  # scheduling

## print the path of each car
for i in range(K):
    print(*all_cars[i].path)

## print the arrival time matrix
print(*arrival_time)