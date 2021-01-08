import logging
import networkx as nx
from topology.map_loader import load_graph_from_file
from ropod.structs.area import Area, SubArea
from topology.plot_map import plot
from fleet_management.plugins.topology.planner_area import PlannerArea



class _TopologyPathPlanner(object):


    def __init__(self):
        """

        """
        self.mapping_dict = dict()
        self.occ_grid = None
        self.start = ''
        self.destination=''
        self.Graph = None
        self.Astar_path = None
        self.path_plan_fms = []

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

        path_plan = []
        for node in self.shortest_path_subgraph.nodes(data=True):
            plannerNode = PlannerArea(a)
            path_plan.append(plannerNode)

        for node in path_plan:
            self.path_plan_fms.append(self.decode_planner_area(node))

        #plot(self.shortest_path_subgraph,self.occ_grid,self.meta_data,name=self.map_name)

        return self.path_plan_fms

    def get_estimated_distance(self):

        return nx.algorithms.shortest_paths.astar.astar_path_length(self.Graph,self.start,self.destination)

    def obl_to_fms_area(self, topology_area):
        """Converts OBL area to FMS area

        Args:
            topology_area (Topology Area): eg. rooms, corridor, elevator etc.

        Returns:
            TYPE: FMS area

        """
        area = Area()
        area.id = topology_area.id
        area.name = topology_area.ref
        area.type = topology_area.type
        if topology_area.level:
            area.floor_number = int(topology_area.level)
        area.sub_areas = []
        if topology_area.navigation_areas is not None:
            for nav_area in topology_area.navigation_areas:
                area.sub_areas.append(self.obl_to_fms_subarea(nav_area))
        return area

    def obl_to_fms_subarea(self, topology_area):
        """Converts Topology to FMS subarea

        Args:
            topology_area (Topology LocalArea): eg. charging, docking,
                                                   undocking areas
        Returns:
            TYPE: FMS SubArea

        """
        sa = SubArea()
        sa.id = topology_area.id
        sa.name = topology_area.ref
        return sa

    def decode_planner_area(self, planner_area):
        """Topology Path planner path consist of PlannerAreas which has local areas and exit doors. In FMS we consider door at same level as area.
        This function is used to extract door from OBL PlannerArea and return it as separate area along with door

        Args:
            planner_area (Topology PlannerArea):

        Returns:
            TYPE: [FMS Area]

        """
        area = self.obl_to_fms_area(planner_area)

        
        return [area]

    def task_to_behaviour(self, task):
        """Convert FMS task to behaviours modelled in OSM world model

        Args:
            task (string):

        Returns:
            TYPE: Maybe string

        """
        if task == 'DOCK':
            return 'docking'
        elif task == 'UNDOCK':
            return 'undocking'
        elif task == 'CHARGE':
            return 'charging'
        elif task == 'REQUEST_ELEVATOR' or task == 'EXIT_ELEVATOR':
            return 'waiting'
        return None




class TopologyPathPlannerBuilder:
    def __init__(self):
        self._instance = None

    def __call__(self, **kwargs):
        if not self._instance:
            self._instance = _TopologyPathPlanner(**kwargs)
        return self._instance


configure = TopologyPathPlannerBuilder()