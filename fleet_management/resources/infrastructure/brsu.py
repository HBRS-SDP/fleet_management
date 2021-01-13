import networkx as nx

from fmlib.utils.utils import load_yaml, load_file_from_module


class DurationGraph(nx.Graph):
    def get_duration(self, plan):
        """Computes the duration of the task plan

        The estimated duration only includes GOTO, DOCK and UNDOCK actions.
        This is based on the data collected for the navigation tasks in the ROPOD project
        from September 2019 to February 2020.

        See https://github.com/anenriquez/ropod_rosbag_processing for details.

        Args:
            plan(TaskPlan):

        Returns:
            mean:
            variance:

        """
        mean = 0
        variance = 0

        subarea_plan = list()
        for action in plan.actions:
            if action.type == "GOTO":
                subarea_plan.extend(self.get_subarea_plan(action.areas))
            elif action.type == "DOCK":
                mean = mean + 50  # This is an estimated value based on observations
            elif action.type == "UNDOCK":
                mean = mean + 20  # This is an estimated value based on observations

        for edge in nx.utils.pairwise(subarea_plan):
            mean = mean + self.edges.get(edge, {}).get("mean", 0)
            variance = variance + self.edges.get(edge, {}).get("variance", 0)

        return mean, variance

    def get_subarea_plan(self, areas):
        sub_area_plan = list()
        for area in areas:
            if area.type == "door":
                sub_area_plan.append(area.id)
            else:
                for subarea in area.subareas:
                    sub_area_plan.append(subarea.id)

        return sub_area_plan

    @classmethod
    def load_graph(cls, **_):
        graph_yaml = load_file_from_module(
            "fleet_management.config.osm_map", "topology.yaml"
        )
        graph_data = load_yaml(graph_yaml)
        graph = nx.node_link_graph(graph_data)
        return cls(graph)
