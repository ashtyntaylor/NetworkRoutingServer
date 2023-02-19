#!/usr/bin/python3


from CS312Graph import *
import time
import math


class NetworkRoutingSolver:
    def __init__(self):
        pass

    def initializeNetwork(self, network):
        assert (type(network) == CS312Graph)
        self.network = network

    def getShortestPath(self, destIndex):
        self.dest = destIndex
        # TODO: RETURN THE SHORTEST PATH FOR destIndex
        #       INSTEAD OF THE DUMMY SET OF EDGES BELOW
        #       IT'S JUST AN EXAMPLE OF THE FORMAT YOU'LL 
        #       NEED TO USE
        path_edges = []
        total_length = 0
        node = self.network.nodes[self.dest]

        while node.node_id != self.source:
            if self.results[node.node_id]['prev'] == -1:
                total_length = math.inf
                path_edges = []  # TODO: Do i need this?
                break
            prev = self.network.nodes[self.results[node.node_id]['prev']]

            # Get the neighbors of the previous node
            neighbors = prev.neighbors
            for neighbor in neighbors:
                if neighbor.dest.node_id == node.node_id:
                    edge = neighbor
                    break
            path_edges.append((edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)))
            total_length += edge.length
            node = prev

        '''edges_left = 3
        while edges_left > 0:
            edge = node.neighbors[2]
            path_edges.append( (edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)) )
            total_length += edge.length
            node = edge.dest
            edges_left -= 1'''

        return {'cost': total_length, 'path': path_edges}

    def computeShortestPaths(self, srcIndex, use_heap=False):
        self.source = srcIndex
        t1 = time.time()

        # TODO: RUN DIJKSTRA'S TO DETERMINE SHORTEST PATHS.
        #       ALSO, STORE THE RESULTS FOR THE SUBSEQUENT
        #       CALL TO getShortestPath(dest_index)

        self.dijkstra(use_heap)

        t2 = time.time()
        return t2 - t1

    # def dijkstra(self, G, l, s):
    def dijkstra(self, use_heap):
        # For all u in V set distance to infinity and prev to null
        H = PriorityQueue(self.network, use_heap)

        # Set source node distance to 0
        H.decrease_key(self.source, 0)

        # TODO: do we really need another queue? what is this for?
        self.results = {}
        # Initialize the results dictionary
        for i in range(len(self.network.nodes)):
            # Add the node to the array
            self.results[self.network.nodes[i].node_id] = {'dist': math.inf, 'prev': -1}
        # Set the results source node distance to 0
        self.results[self.source]['dist'] = 0

        # while H is not empty do
        while not H.isEmpty():
            u = H.delete_min()
            E = self.network.nodes[u['id']].neighbors

            # for all edges (u,v) in E do:
            for edge in E:
                v = self.results[edge.dest.node_id]

                # if dist(v) > dist(u) + l(u, v) then
                if v['dist'] > u['dist'] + edge.length:
                    # dist(v) = dist(u) + l(u, v)
                    v['dist'] = u['dist'] + edge.length

                    # prev(v) = u
                    v['prev'] = u['id']

                    # H.decreasekey(v)
                    H.decrease_key(edge.dest.node_id, v['dist'])


# Implementations of a priority queue using an array and using a binary heap
class PriorityQueue:
    def __init__(self, network, use_heap=False):
        self.use_heap = use_heap
        print("Use heap: " + str(self.use_heap))
        self.node_count = len(network.nodes)

        # Priority queue using an array
        if not use_heap:
            self.arrayPQ = {}

            # Add nodes from network into the array with distances set to infinity
            for i in range(self.node_count):
                #self.arrayPQ[network.nodes[i].node_id] = {'dist': math.inf}
                self.insert(network.nodes[i].node_id, math.inf)

        # Priority queue using a binary heap
        else:
            self.heapPQ = [-1]
            for i in range(self.node_count):
                self.insert(network.nodes[i].node_id, math.inf)

    def insert(self, node_id, dist):
        if not self.use_heap:
            self.arrayPQ[node_id] = {'dist': dist}
        else:
            x = {'id': node_id, 'dist': dist}
            self.heapPQ.append(x)

            self.percolate_up(x, len(self.heapPQ) - 1)

    def percolate_up(self, x, pos):
        # Check if the parent is bigger
        while pos != 1 and self.heapPQ[int(pos / 2)]['dist'] > x['dist']:
            # Get the element's parent
            parent = self.heapPQ[int(pos / 2)]
            # Swap the parent and the child
            self.heapPQ[int(pos / 2)] = x
            self.heapPQ[pos] = parent
            # Update the position of the new element
            pos = int(pos / 2)

    # Log(V) operation because we multiply the position by 2 every step
    def percolate_down(self, x, pos):
        # Check if we are at the bottom of the tree
        # or if the children are not less than x
        while True:
            children = []
            positions = []
            # Check left child
            if (2 * pos) <= (len(self.heapPQ) - 1) and self.heapPQ[pos * 2]['dist'] < x['dist']:
                # Get the child
                children.append(self.heapPQ[pos * 2])
                positions.append(pos * 2)
            # Otherwise check if there is another child
            if (2 * pos + 1) <= (len(self.heapPQ) - 1) and self.heapPQ[pos * 2 + 1]['dist'] < x['dist']:
                children.append(self.heapPQ[pos * 2 + 1])
                positions.append(pos * 2 + 1)
            if len(children) == 0:
                break
            # Swap the parent and the child
            if len(children) == 1:
                child = children[0]
                child_pos = positions[0]
            elif children[0]['dist'] < children[1]['dist']:
                child = children[0]
                child_pos = positions[0]
            else:
                child = children[1]
                child_pos = positions[1]

            self.heapPQ[child_pos] = self.heapPQ[pos]
            self.heapPQ[pos] = child
            pos = child_pos

    def decrease_key(self, node_id, dist):
        if not self.use_heap:
            self.arrayPQ[node_id]['dist'] = dist
        else:
            # self.heapPQ.update(node_id, dist)
            for i in range(1, len(self.heapPQ)):
                if self.heapPQ[i]['id'] == node_id:
                    self.heapPQ[i]['dist'] = dist
                    self.percolate_up(self.heapPQ[i], i)
                    break

    def isEmpty(self):
        if not self.use_heap:
            if len(self.arrayPQ) == 0:
                return True
            else:
                return False
        else:
            if len(self.heapPQ) - 1 == 0:
                return True
            else:
                return False

    # Return the element with the smallest key and remove it from the set
    # O(n)
    def delete_min(self):
        if not self.use_heap:
            min_dist = math.inf
            smallest_key = -1

            # Find smallest key
            for key, value in self.arrayPQ.items():
                if self.arrayPQ[key]['dist'] < min_dist:
                    min_dist = self.arrayPQ[key]['dist']
                    smallest_key = key

            smallest_element = {'id': smallest_key, 'dist': min_dist}

            # Remove smallest element from the queue
            if smallest_key == -1:
                remove_element = self.arrayPQ.popitem()
                return {'id': remove_element[0], 'dist': remove_element[1]['dist']}
            del self.arrayPQ[smallest_key]
            return smallest_element

        else:
            first = self.heapPQ[1]
            x = self.heapPQ[len(self.heapPQ) - 1]
            self.heapPQ[1] = x
            self.heapPQ.pop()

            self.percolate_down(x, 1)

            return first
