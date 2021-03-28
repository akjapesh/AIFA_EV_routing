# Floyd-Warshall algorithm to calculate min. distance of every node from other
#  this calculated distance will be used as h(n) i.e heurestic value at each node
def floydwarshall(W):
    V = W.shape[0]
    for k in range(V):

        # pick all vertices as source one by one
        for i in range(V):

            # Pick all vertices as destination for the
            # above picked source
            for j in range(V):
                # If vertex k is on the shortest path from
                # i to j, then update the value of dist[i][j]
                W[i][j] = min(W[i][j],
                              W[i][k] + W[k][j])
    return W

#for inputting the graph data
V = int(input("Enter the number of vertices"))                                  #input the no of nodes
adj = [[0 for i in range(V)] for j in range(V)]                                 #adj is the adjancy matrix of the graph
print("Now start entering the edge distances")
for i in range(V):
  for j in range(V):
    inp = input(f"e{i}{j}->")
    if inp=="INF" or inp=="inf":
      adj[i][j]=float("inf")                                                    #if edge present then weight is set as the distance b/w them else INF
    else:
      adj[i][j] = float(inp)

W = floydwarshall(adj)                                                          # W will store the matrix having min distance of each node from another

#defining Car class for different EV's to have unique information
from collections import defaultdict
class car:
  def __init__(self,src, dest, battery_status, charging_rate, discharging_rate, Max_battery, avg_speed, W):
    self.src = src                                                              #starting node of the car
    self.dest = dest                                                            #ending node of the car
    self.battery_status = battery_status                                        #current battery charge of EV
    self.charging_rate = charging_rate                                          #charginf rate of battery
    self.discharging_rate=discharging_rate                                      #discharging rate of battery
    self.Max_battery=Max_battery                                                #max battery capacity
    self.avg_speed=avg_speed                                                    #avg speed of EV on a road
    self.hn=W[self.src][self.dest]/self.avg_speed                               #heurestic value on EV when on particular node...initial it is time for shortest dist b/w src->dest, and update as the min time from current_node->dest i.e calculates from floyd-warshal algo
    self.gn=0                                                                   #g(n) stores the spent time while reaching the present node...initially it is 0 and will be updated on each node as time to reach that node
    self.fn=self.hn+self.gn                                                     #f(n)=h(n)+g(n)
    self.cur = self.src                                                         #current node of the EV
    self.visited = defaultdict(bool)                                            # a is_visited array to store if the node has been visited or not to generate new states
    self.parent = []                                                            #a list to store the parent nodes for retracing the path

  def calculate_fn(self, node):                                                  # func to calculate the f(n) as it will be updated on each state
      self.hn = W[node][self.dest]/self.avg_speed                               # new h(n) will be now min time from child node to destination
      battery_consmp = W[self.cur][self.dest]/self.discharging_rate             # min future amount of battery that will be consumed in going from present node to destination
      if(self.battery_status<battery_consmp):                                   # if our present battery status is less than that will be consumed later so we charge at this node, as it is best to charge at this node as no collision on this node
          self.gn += (min(battery_consmp, self.Max_battery)-self.battery_status)/self.charging_rate   #hence updating the g(n) as time spent to charge here on this node
          self.battery_status = min(battery_consmp, self.Max_battery)           #battery status updated
      self.gn += adj[self.cur][node]/self.avg_speed                             #g(n) will be updated normally as time taken from present node to next node
      self.fn = self.hn + self.gn                                               #finally f(n) updated


#cell to input all types of input i.e. src,dest,...,avg_speed of each EV's individaully  src[i] represents src of ith EV and so on
src={}
dest={}
battery_status={}
charging_rate={}
discharging_rate={}
Max_battery={}
avg_speed={}
for i in range(V):
  src[i]=int(input(f"source node of {i+1}th car"))
  dest[i]=int(input(f"destination node of {i+1}th car"))
  battery_status[i]=float(input(f"initial battery status of {i+1}th car"))
  charging_rate[i]=float(input(f"charging rate of {i+1}th car"))
  discharging_rate[i]=float(input(f"discharging rate of {i+1}th car"))
  Max_battery[i]=float(input(f"Max battery capacity of {i+1}th car"))
  avg_speed[i]=float(input(f"average speed of {i+1}th car"))


#function to return maximum f(n) among all cars in a state i.e. max time for completion of the last car to destination
#so f(n) of the state is represented by this and will use this value in A* algorithm to chose minimum from
def calculate_max_fn(state):
    max_fn = float("-inf")
    for key, value in state.items():
        obj = value
        max_fn = min(max_fn, obj.fn)
    return max_fn

# Initialisation of starting state
curr_state = {}                                                                 #starting state will be a list of all cars at its starting position
for i in range(V):
  obj = car(src[i],dest[i],battery_status[i],charging_rate[i],discharging_rate[i],Max_battery[i],avg_speed[i],W)#creating an object of ith car with req info
  obj.calculate_fn(src[i])                                                      #calculating its f(n) at source node
  curr_state[i+1]=obj                                                           #adding this object car to state dict

# A* algorithm implemented to find the best possible time for all to reach dest node
from queue import PriorityQueue

q = PriorityQueue()  # priority queue to store the {f(state),state} and return the state with minimum f(state)
q.put((calculate_max_fn(curr_state),
       curr_state))  # for each state its f(state) is calculated by this function representing the max f(n) of all cars in its present location
while (not q.empty()):
    curr_fn, curr_state = q.get()
    if (check(curr_state) == True):  # check if the present state is the final state as all cars have reached its destination
        break;
    new_states = []  # list to store all new states
    generate_states(new_states,
                    curr_states)  # func that will generate all permutations of possible state i.e all possible combination of each car going to its next node
    for st in new_states():  # for each state in total generated state
        flag = True  # flag to check if given state is valid or not
        for i in range(len(st)):  # for each car in present state
            if st[i].battery_status < adj[curr_state[i].cur][
                st[i].cur]:  # if with present battery it can't go to next node then current state is invalid
                flag = False
                break
            st[i].calculate_fn(curr_state[
                                         i].cur)  # calculate f(n) of this curr node of car in this state with parent as previous node from last state

            for i in range(len(st)):  # condition of collision of 2 cars
                for j in range(i + 1, len(st)):
                    if (st[j].cur == st[i].cur & & st[j].gn == st[
                        i].gn):  # if current node of both car is same as well as g(n) i.e. time taken by both car to reach this node is sam
                        if (st[i].fn < st[
                            j].fn):  # if ith car has less time of reaching till end then it can wait a litte longer and jth car will be charged and vice versa
                            st[i].gn += (st[j].Max_battery - st[j].battery_status) / st[
                                j].charging_rate  # so ith car is waiting and jth charginf time added to its g(n) i.e. time cost till this node
                        else:
                            st[j].gn += (st[i].Max_battery - st[i].battery_status) / st[i].charging_rate
        if (flag == False):  # if flag==false i.e. state is not valid then continue
            continue
        q.put((calculate_max_fn(st), st))  # push this state in priority queue
