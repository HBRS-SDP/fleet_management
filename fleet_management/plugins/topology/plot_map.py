import matplotlib.pyplot as plt
import networkx as nx
from importlib_resources import path


def map_to_img(x, y, origin, resolution, y_max):
    x_ = (x - origin[0]) / resolution
    y_ = y_max - ((y - origin[1]) / resolution)
    return [x_, y_]


def plot(graph, occ_grid, meta_data, map_name, pos="pose"):
    plt.rcParams["figure.figsize"] = [50, 50]
    ax = plt.axes()
    ax.imshow(occ_grid, cmap="gray", interpolation="none", origin="lower")
    resolution = meta_data.get("resolution")
    origin = meta_data.get("origin")
    ymax, xmax = occ_grid.shape

    pose = nx.get_node_attributes(graph, pos)
    pos_ = {
        p: map_to_img(coord[0], coord[1], origin, resolution, ymax)
        for p, coord in pose.items()
    }

    nx.draw_networkx(graph, pos=pos_, node_size=1 / resolution, ax=ax)

    with path("fleet_management.plugins.topology.maps." + map_name, "roadmap.png") as p:
        roadmap_file = p

    plt.savefig(roadmap_file, dpi=100, bbox_inches="tight")
    plt.show()
