import queue as Q
pq = Q.PriorityQueue() 

from cProfile import label
from multiprocessing.sharedctypes import Value    #pip install pyvis
from pyvis.network import Network

#defining a graph of campus
graph={'shankar':[('old_football_field',0.0928),('f_block',0.4936),('g_block',0.414),('main_gate',0.4899),('kabadi_field',0.2602)]
    ,'old_football_field':[('mess_1',0.191), ('shankar',0.0928)]
    ,'mess_1':[('workshop',0.593),('CP',0.3042),('old_football_field',0.191)],
    'CP':[('mess_1',0.3042),('SAC',0.2926),('kabadi_field',0.1884),('cricket_ground',0.3756)],
    'SAC':[('CP',0.2926),('workshop',0.8362)],
    'kabadi_field':[('shankar',0.2602),('CP',0.1884)],
    'workshop':[('mess_1',0.593),('f_block',0.113),('SAC',0.8362)],
    'f_block':[('workshop',0.113),('shankar',0.4936),('g_block',0.1263)],
    'g_block':[('f_block',0.1236),('main_gate',0.5344),('shankar',0.4104)],
    'main_gate':[('g_block',0.5344),('shankar',0.4899)],
    'cricket_ground':[('CP',0.3756)]}


def plotter():      #function to plot the graph(open the html file generated after running code).....Visualisation using pyvis library
    net = Network()

    for keys in graph:
        net.add_node(keys, label = keys)

    for keys in graph:

        for it in graph[keys]:
            node,val = it
            net.add_edge(keys,node, label = str(val))

    net.repulsion(node_distance=100, spring_length=300)
    net.show('visualise.html')

def reconstruct_path(came_from, current):  #function to trace back and print the shortest path in A star and dijkstra algorithm
    final_path = []

    final_path.append(current)

    while current in came_from:
        current = came_from[current]
        final_path.append(current)
    
    n = len(final_path)
    n = n-1

    print("Path: ", end="")
    while n >= 0:
        print(final_path[n], end="->")
        n = n-1

def another_dijkstra(source, destination): #function for dijkstra ..... given a source and destination node it returns the shortest distance between them
    #in dijkstra the priority queue is sorted according to distance of node from source node
    infinity = 999999

    distTo = {'shankar':infinity
    ,'old_football_field':infinity
    ,'mess_1':infinity,
    'CP':infinity,
    'SAC':infinity,
    'kabadi_field':infinity,
    'workshop':infinity,
    'f_block':infinity,
    'g_block':infinity,
    'main_gate':infinity,
    'cricket_ground':infinity}

    came_fromm = {}
    distTo[source] = 0
    pq.put((0, source))
    

    while not pq.empty():

        dist,prev =  pq.get()

        for it in graph[prev]:

            nexte,nextDist = it

            if(distTo[nexte] > dist + nextDist):
                distTo[nexte] = dist + nextDist
                came_fromm[nexte] = prev
                pq.put((distTo[nexte], nexte))
    
    print(source, " to ", destination)
    print("Shortest distance using Dijkstra: ", distTo[destination])
    reconstruct_path(came_fromm, destination)

def dijkstra(source, destination): #function for dijkstra ..... given a source and destination node it returns the shortest distance between them
    #in dijkstra the priority queue is sorted according to distance of node from source node
    infinity = 999999

    distTo = {'shankar':infinity
    ,'old_football_field':infinity
    ,'mess_1':infinity,
    'CP':infinity,
    'SAC':infinity,
    'kabadi_field':infinity,
    'workshop':infinity,
    'f_block':infinity,
    'g_block':infinity,
    'main_gate':infinity,
    'cricket_ground':infinity}

    came_fromm = {}
    distTo[source] = 0
    pq.put((0, source))
    

    while not pq.empty():

        dist,prev =  pq.get()

        for it in graph[prev]:

            nexte,nextDist = it

            if(distTo[nexte] > dist + nextDist):
                distTo[nexte] = dist + nextDist
                came_fromm[nexte] = prev
                pq.put((distTo[nexte], nexte))
    

    return distTo[destination]
    

def A_star(source, destination): #function for A star... prints the shortest path from source to destination
    #in A-star the priority queue is sorted according to heutistic + distance of node from source node
    infinity = 999999

    distTo = {'shankar':infinity
    ,'old_football_field':infinity
    ,'mess_1':infinity,
    'CP':infinity,
    'SAC':infinity,
    'kabadi_field':infinity,
    'workshop':infinity,
    'f_block':infinity,
    'g_block':infinity,
    'main_gate':infinity,
    'cricket_ground':infinity}

    came_from = {} #this dictionary stores which node the particular node came from .... used to trace back the final shortest path
    heuristic = {} #makes sure the search is going in the right direction(towards destination)


    # calculating heuristics(shortest distance to destination from each node) using dijkstra's algorithm
    for keys in graph:
        heuristic[keys] = dijkstra(keys, destination)
    

    distTo[source] = 0
    pq.put((heuristic[source],0, source))
    

    while not pq.empty():

        hx,dist,prev =  pq.get()
        if(prev == destination):
            print(source, " to ", destination)
            print("Shortest distance using A-star: ",distTo[destination])
            reconstruct_path(came_from, prev) #if we found the destination we wanted to reach to ... backtrack the path where we came from and print answer
            return

        for it in graph[prev]:

            nexte,nextDist = it

            if(distTo[nexte] > dist + nextDist):
                distTo[nexte] = dist + nextDist
                came_from[nexte] = prev
                pq.put((distTo[nexte]+heuristic[nexte], distTo[nexte], nexte))
    

    #if no path exists print this
    print("No path found")



####################################################################### FINAL RESULTS


plotter() # this functions generates a HTML file in working directory(visualise.html) which has the visualisation of the graph

#functions prints shortest distance ....source to destination along with the path(using dijkstra)
another_dijkstra('shankar', 'SAC')

print("\n")

#functions prints shortest distance ....source to destination along with the path(using A-star)
A_star('shankar', 'SAC') 




