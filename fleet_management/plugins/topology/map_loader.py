import networkx as nx
import matplotlib.pyplot as plt
import yaml
from importlib_resources import open_text, path

from fleet_management.plugins.topology.planner_area import PlannerArea
from fleet_management.plugins.topology.plot_map import plot

MAP_BASE_PATH = "fleet_management.plugins.topology.maps."


class TopologyPlannerMap:
    def __init__(self, map_name="brsu-full"):

        self.map_name = map_name
        self.occ_grid = self.load_occ_grid_from_file(self.map_name)
        self.meta_data = self.load_meta_data_from_file(self.map_name)
        self.map_graph = self.load_graph_from_file(self.map_name)
        self.map_dict = self.convert_map_to_dict()

    def load_yaml_file(self, module, filename):
        """
        Load a yaml file from ``module`` 
        function taken from https://github.com/HBRS-SDP/topological_map/blob/refactoring/layered-maps/common/map_utils/map_loader.py
        """
        with open_text(module, filename) as map_file:
            map_data = yaml.safe_load(map_file)

        return map_data

    def load_graph_from_file(self, map_name, file_name="topology.yaml"):
        """
        Load a topological map from yaml file 
        function taken from https://github.com/HBRS-SDP/topological_map/blob/refactoring/layered-maps/common/map_utils/map_loader.py
        """
        data = self.load_yaml_file(MAP_BASE_PATH + map_name, file_name)
        return nx.node_link_graph(data)

    def load_occ_grid_from_file(self, map_name, file_name="map.pgm"):
        """
        Load a occupancy gride from yaml file
        function taken from https://github.com/HBRS-SDP/topological_map/blob/refactoring/layered-maps/common/map_utils/map_loader.py
        """
        with path(MAP_BASE_PATH + map_name, file_name) as p:
            occ_grid_file = p
        return plt.imread(occ_grid_file, True)

    def load_meta_data_from_file(self, map_name, file_name="map.yaml"):
        """
        Load a topological map meta data from yaml file 
        function taken from https://github.com/HBRS-SDP/topological_map/blob/refactoring/layered-maps/common/map_utils/map_loader.py
        """
        with path(MAP_BASE_PATH + map_name, file_name) as p:
            meta_data_file = p
        return nx.read_yaml(meta_data_file)

    def save_to_file(self, map_name, graph):
        """
        Save a topological map to yaml file
        function taken from https://github.com/HBRS-SDP/topological_map/blob/refactoring/layered-maps/common/map_utils/map_loader.py
        """
        yaml.Dumper.ignore_aliases = lambda *args: True
        with open(map_name + "/topology.yaml", "w") as yaml_file:
            yaml.dump(nx.node_link_data(graph), yaml_file, default_flow_style=False)

    def convert_map_to_dict(self):
        """
        Covert the topological map into a dictionary of format:
        {node_id: "node_label"}
        
        Return:
        mapping_dict: dictionary
        """
        mapping_dict = dict()
        for id, label in self.map_graph.nodes("label"):
            mapping_dict[id] = label

        return mapping_dict

    def get_all_nodes_of_behaviour_type(self, behaviour_type):
        """
        Retrieve all nodes of a certain behaviour type
        
        Arg:
        behaviour_type: "docking" or "undocking"

        Return:
        List of nodes with the desired behaviour
        """
        node_with_behaviour = []
        for node in self.map_graph.nodes(data=True):
            if node[1][behaviour_type] == True:
                node_with_behaviour.append(node[1]["label"])

        return node_with_behaviour

    def get_astar_path(self, start, destination):
        """Plans path using A* in the topological map

        Args:
            start (str): start node label
            destination (str): destination node label
            
        Returns:
            TYPE: [PlannerArea], mean and variance of path
        """
        start_node = list(self.map_dict.keys())[
            list(self.map_dict.values()).index(start.split("_")[-1])
        ]

        destination_node = list(self.map_dict.keys())[
            list(self.map_dict.values()).index(destination.split("_")[-1])
        ]

        astar_path = nx.algorithms.shortest_paths.astar.astar_path(
            self.map_graph, start_node, destination_node
        )

        areas = []
        for node in astar_path:
            planner_node = self.nx_to_planner_area(
                self.map_graph.nodes(data=True)[node]
            )
            areas.append(planner_node)

        mean, variance = self.get_travel_duration(astar_path)

        return [areas, mean, variance]

    def nx_to_planner_area(self, node):
        """
        Covert the NetworkX Node into PlannerArea format
        
        Return:
        planner_node: PlannerArea
        """
        planner_node = PlannerArea(node)

        return planner_node

    def get_travel_duration(self, path):
        """
        Get the mean and variance values of a specific path
        
        Return:
        mean: Double
        variance: Double
        """
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
        xpoint = kwargs.get("x")
        ypoint = kwargs.get("y")
        behaviour = kwargs.get("behaviour")
        if floor_name is None and area_name is None:
            return None
        if behaviour is None and (xpoint is None or ypoint is None):
            return None

        if behaviour is None:
            return self._get_local_area_from_position(
                xpoint, ypoint, area_name, floor_name
            )
        else:
            return self._get_local_area_from_behaviour(area_name, floor_name, behaviour)

    def _get_local_area_from_behaviour(self, area_name, floor_name, behaviour):
        """returns a LocalArea object inside area_name with given behaviour tag
        :area_name: string
        :floor_name: string
        :query_behaviour: string
        :returns: LocalArea object
        """
        node_idx = list(self.map_dict.keys())[
            list(self.map_dict.values()).index(area_name.split("_")[-1])
        ]

        if self.map_graph.nodes(data=True)[node_idx][behaviour] is True:
            return self.map_graph.nodes(data=True)[node_idx]
        else:
            return None

    def _get_local_area_from_position(self, xpoint, ypoint, area_name, floor_name):
        """get LocalArea object which contains point (pointX, pointY)
        :pointX: int/float
        :pointY: int/float
        :area_name: string
        :floor_name: string
        :returns: LocalArea object
        """
        return None
