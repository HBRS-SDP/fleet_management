import networkx as nx
import matplotlib.pyplot as plt
import yaml
from importlib_resources import open_text

from fleet_management.plugins.topology.planner_area import PlannerArea

# from fleet_management.plugins.topology.plot_map import plot


class TopologyPlannerMap:
    def __init__(self, map_name="brsu-full"):

        self.map_name = map_name
        self.occ_grid = plt.imread(
            "/home/hwalli92/Documents/MAS/SDP/fleet-management/fleet_management/plugins/topology/maps/%s/map.pgm"
            % self.map_name,
            True,
        )
        self.meta_data = nx.read_yaml(
            "/home/hwalli92/Documents/MAS/SDP/fleet-management/fleet_management/plugins/topology/maps/%s/map.yaml"
            % self.map_name
        )
        self.map_graph = self.load_graph_from_file(self.map_name)
        self.map_dict = self.convert_map_to_dict()

    def load_yaml_file(self, module, filename):
        """Load a yaml file from ``module``"""
        with open_text(module, filename) as map_file:
            map_data = yaml.safe_load(map_file)

        return map_data

    def load_graph_from_file(self, map_name, file_name="topology.yaml"):
        data = self.load_yaml_file(
            "fleet_management.plugins.topology.maps." + map_name, file_name
        )
        return nx.node_link_graph(data)

    def save_to_file(self, map_name, graph):
        yaml.Dumper.ignore_aliases = lambda *args: True
        with open(map_name + "/topology.yaml", "w") as yaml_file:
            yaml.dump(nx.node_link_data(graph), yaml_file, default_flow_style=False)

    def convert_map_to_dict(self):
        mapping_dict = dict()
        for id, label in self.map_graph.nodes("label"):
            mapping_dict[id] = label

        return mapping_dict

    def get_all_nodes_of_behaviour_type(self, behaviour_type):
        node_with_behaviour = []
        for node in self.map_graph.nodes(data=True):
            if node[1][behaviour_type] == True:
                node_with_behaviour.append(node[1]["label"])

        return node_with_behaviour

    def get_astar_path(self, start, destination):
        start_node = list(self.map_dict.keys())[
            list(self.map_dict.values()).index(start)
        ]

        destination_node = list(self.map_dict.keys())[
            list(self.map_dict.values()).index(destination)
        ]

        astar_path = nx.algorithms.shortest_paths.astar.astar_path(
            self.map_graph, start_node, destination_node
        )

        shortest_path_subgraph = self.map_graph.subgraph(astar_path)

        # plot(shortest_path_subgraph, self.occ_grid, self.meta_data)

        path_plan = []
        for node in shortest_path_subgraph.nodes(data=True):
            plannerNode = PlannerArea(node)
            path_plan.append(plannerNode)

        return path_plan

    def get_estimated_distance(self, start, destination):

        return nx.algorithms.shortest_paths.astar.astar_path_length(
            self.map_graph, start, destination
        )
