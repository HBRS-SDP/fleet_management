import networkx as nx
import yaml
from importlib_resources import open_text


def load_yaml_file(module, file_name):
    """Load a yaml file from ``module``"""
    with open_text(module, file_name) as yaml_file:
        msg = yaml.load(yaml_file, Loader=yaml.Loader)

    return msg


def load_graph_from_file(map_name, file_name="topology.yaml"):
    data = load_yaml_file("maps." + map_name, file_name)
    return nx.node_link_graph(data)


def save_to_file(map_name, graph):
    yaml.Dumper.ignore_aliases = lambda *args: True
    with open(map_name + "/topology.yaml", "w") as yaml_file:
        yaml.dump(nx.node_link_data(graph), yaml_file, default_flow_style=False)