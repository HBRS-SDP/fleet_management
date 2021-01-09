import matplotlib.pyplot as plt
import networkx as nx

# from fleet_management.plugins.topology.map_loader import load_graph_from_file


def map_to_img(x, y, origin, resolution, y_max):
    x_ = (x - origin[0]) / resolution
    y_ = y_max - ((y - origin[1]) / resolution)
    return [x_, y_]


def plot(graph, occ_grid, meta_data, pos="pose", name="brsu"):
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
    plt.savefig("maps/%s/roadmap.png" % name, dpi=300, bbox_inches="tight")
    plt.show()


if __name__ == "__main__":

    map_name = "brsu-small-free"
    occ_grid = plt.imread("config/maps/%s/map.pgm" % map_name, True)
    meta_data = nx.read_yaml("config/maps/%s/map.yaml" % map_name)

    # G = load_graph_from_file(map_name, "topology.yaml")

    # plot(G, occ_grid, meta_data, name=map_name)
