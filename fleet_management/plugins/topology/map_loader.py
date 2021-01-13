import networkx as nx
import matplotlib.pyplot as plt
import yaml
from importlib_resources import open_text, path

from fleet_management.plugins.topology.planner_area import PlannerArea

from fleet_management.plugins.topology.plot_map import plot


class TopologyPlannerMap:
    def __init__(self, map_name="brsu-full"):

        self.map_name = map_name
        self.occ_grid = self.load_occ_grid_from_file(self.map_name)
        self.meta_data = self.load_meta_data_from_file(self.map_name)
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

    def load_occ_grid_from_file(self, map_name, file_name="map.pgm"):
        with path("fleet_management.plugins.topology.maps." + map_name, file_name) as p:
            occ_grid_file = p
        return plt.imread(occ_grid_file, True)

    def load_meta_data_from_file(self, map_name, file_name="map.yaml"):
        with path("fleet_management.plugins.topology.maps." + map_name, file_name) as p:
            meta_data_file = p
        return nx.read_yaml(meta_data_file)

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
            list(self.map_dict.values()).index(start.split("_")[-1])
        ]

        destination_node = list(self.map_dict.keys())[
            list(self.map_dict.values()).index(destination.split("_")[-1])
        ]

        astar_path = nx.algorithms.shortest_paths.astar.astar_path(
            self.map_graph, start_node, destination_node
        )

        # shortest_path_subgraph = self.map_graph.subgraph(astar_path)
        # plot(shortest_path_subgraph, self.occ_grid, self.meta_data, self.map_name)

        areas = []
        for node in astar_path:
            plannerNode = self.nx_to_planner_area(self.map_graph.nodes(data=True)[node])
            areas.append(plannerNode)

        mean, variance = self.get_travel_duration(astar_path)

        return [areas, mean, variance]

    def nx_to_planner_area(self, node):

        plannerNode = PlannerArea(node)

        return plannerNode

    def get_travel_duration(self, path):

        mean = 0
        variance = 0

        for edge in nx.utils.pairwise(path):
            mean = mean + self.map_graph.edges.get(edge, {}).get("mean", 0)
            variance = variance + self.map_graph.edges.get(edge, {}).get("variance", 0)

        return mean, variance

    def local_area_finder(self, *args, **kwargs):
        """gets the LocalArea object containing a point (x, y) or with a behaviour tag
        Args:
            :x: int/float
            :y: int/float
            :area_name: string
            :floor_name: string
            :behaviour: string
            :isLatlong: boolean (default False)
        returns: 
            PlannerArea object
        
        Either area_name or floor_name is required to get non None return
        Either behaviour or (x and y) is required to get non None return 
        """
        area_name = kwargs.get("area_name")
        floor_name = kwargs.get("floor_name")
        pointX = kwargs.get("x")
        pointY = kwargs.get("y")
        behaviour = kwargs.get("behaviour")
        # self._isLatlong = kwargs.get("isLatlong", self._isLatlong)
        if floor_name is None and area_name is None:
            return None
        if behaviour is None and (pointX is None or pointY is None):
            return None

        if behaviour is None:
            return self._get_local_area_from_position(
                pointX, pointY, area_name, floor_name
            )
        else:
            return self._get_local_area_from_behaviour(area_name, floor_name, behaviour)

    def _get_local_area_from_behaviour(self, area_name, floor_name, behaviour):

        node_idx = list(self.map_dict.keys())[
            list(self.map_dict.values()).index(area_name.split("_")[-1])
        ]

        if self.map_graph.nodes(data=True)[node_idx][behaviour] is True:
            return self.map_graph.nodes(data=True)[node_idx]
        else:
            return None

    def _get_local_area_from_position(self, pointX, pointY, area_name, floor_name):
        return None
