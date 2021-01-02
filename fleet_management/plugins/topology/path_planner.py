import logging
import networkx as nx
from topology.map_loader import load_graph_from_file
from topology.plot_map import plot





class _TopologyPathPlanner():


    def __init__(self):
        """

        """
        self.mapping_dict = dict()
        self.occ_grid = None
        self.start = ''
        self.destination=''
        self.Graph = None
        self.Astar_path = None
        self.sub

        self.logger= logging.getLogger('fms.plugins.path_planner')

        self.logger.info("Path planner service ready ...")

    def get_path_plan(self,map_name='brsu-full', start='',destination=''):

        self.map_name= map_name
        self.occ_grid = plt.imread("/maps/%s/map.pgm" % self.map_name, True)
        self.meta_data = nx.read_yaml("/maps/%s/map.yaml" % self.map_name)

        self.Graph = load_graph_from_file(self.map_name)
        #plot(self.Graph, self.occ_grid, self.meta_data, name=self.map_name)

        self.mapping_dict = dict{}
        for id, label in self.Graph.nodes("label"):
            self.mapping_dict[id] =  label

        self.start = list(self.mapping_dict.keys())[list(self.mapping_dict.values()).index(start)]
        self.destination = list(self.mapping_dict.keys())[list(self.mapping_dict.values()).index(destination)]

        self.Astar_path = nx.algorithms.shortest_paths.astar.astar_path(self.Graph,self.start,self.destination)
        self.shortest_path_subgraph = self.Graph.subgraph(self.Astar_path)

        plot(self.shortest_path_subgraph,self.occ_grid,self.meta_data,name=self.map_name)

        return self.shortest_path_subgraph

    def get_estimated_distance(self):

        return nx.algorithms.shortest_paths.astar.astar_path_length(self.Graph,self.start,self.destination)