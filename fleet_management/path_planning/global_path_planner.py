from .node import Node
from .way import Way
from .router import Router
from fleet_management.structs.area import Area, Waypoint

class GlobalPathPlanner:

  def __init__(self,api):
    self.api = api
    self.path_elements_ids = []

  def set_start_destination_locations(self, start, destination):
    start = start.split("_")
    destination = destination.split("_")
    if len(start) > 3 and len(destination) > 3:
      start_org = start[0]
      start_floor = start[2]
      start_building = start[1]
      start_element = start[3]

      destination_org = destination[0]
      destination_building = destination[1]
      destination_floor = destination[2]
      destination_element = destination[3]

      if start_org != destination_org:
        print("Cannot plan path across 2 different organisations")
        return False

      self.organisation_ref = start_org
      self.start_floor = self.organisation_ref + "_" + start_floor
      self.start_building = self.organisation_ref + "_" + start_building +"_" + start_floor
      self.start_element = self.start_building + '_' + start_element

      # self.organisation_ref = destination_org
      self.destination_floor = self.organisation_ref + "_" + destination_floor
      self.destination_building = self.organisation_ref + "_" + destination_building +"_" + destination_floor
      self.destination_element = self.destination_building + '_' + destination_element

      if len(start) > 4 and len(destination) > 4:
        self.start_local = self.start_building + '_' + start_element + '_' + start[4]
        self.destination_local = self.destination_building + '_' + destination_element + '_' + destination[4]
      return True
    else:
      print("Invalid start and/or destination location")
      return False


  def get_start_node(self):
    data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"level");relation[ref="' + self.start_floor + '"]; >;relation[ref="' + self.start_element + '"];node(r._:"topology");')
    if len(data.get('elements')) > 0:
      return Node(data.get('elements')[0])
    else:
      print('Start location {} does not exist'.format(self.start_element))
      return False

  def get_destination_node(self):
    data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"level");relation[ref="' + self.destination_floor + '"]; >;relation[ref="' + self.destination_element + '"];node(r._:"topology");')
    if len(data.get('elements')) > 0:
      return Node(data.get('elements')[0])
    else:
      print('Destination location {} does not exist'.format(self.destination_element))
      return False

  def get_connections(self, floor_ref):
    data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"level");relation[ref="' + floor_ref + '"];way(r._:"global_connection");')
    ways = []
    connections = data.get('elements')
    for connection in connections:
      w = Way(connection)
      for n in connection.get('nodes'):
        data = self.api.get('node('+ str(n) + ');')
        w.nodes.append(Node(data.get('elements')[0]))
      ways.append(w)
    return ways

  def get_elevators(self):
    data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"elevator"); node(r._:"topology");')
    return [Node(d) for d in data.get('elements')]

  def plan_path(self):
    start_node = self.get_start_node()
    destination_node = self.get_destination_node()

    if start_node and destination_node:
      if self.start_floor == self.destination_floor:
        connections = self.get_connections(self.start_floor)
        global_router = Router(start_node, destination_node, connections)
        global_router.route()
        self.path_nodes = global_router.nodes
      else:
        connections_start = self.get_connections(self.start_floor)
        start_floor_paths = []
        start_path_dist = []
        elevators = self.get_elevators()
        for elevator in elevators:
          start_floor_router = Router(start_node, elevator, connections_start)
          start_floor_router.route()
          start_path_dist.append(start_floor_router.path_distance)
          start_floor_paths.append(start_floor_router.nodes)

        best_start_path_idx = start_path_dist.index(min(start_path_dist))
        start_floor_path = start_floor_paths[best_start_path_idx]

        connections_destination = self.get_connections(self.destination_floor)

        destination_floor_router = Router(elevators[best_start_path_idx], destination_node, connections_destination)
        destination_floor_router.route()
        destination_path_dist = destination_floor_router.path_distance
        destination_floor_path = destination_floor_router.nodes
        
        self.path_nodes = start_floor_path + destination_floor_path
      return True
    else:
      print("[Invalid information] Cannot plan the path") 
      return False

  def prepare_path(self):
    path = [];
    for node in self.path_nodes:
      data = self.api.get('node('+str(node.id)+');rel(bn:"topology");')
      if len(data.get('elements')) > 0:
        tags = data.get('elements')[0].get('tags')
      else:
        print('Missing information in a map for node with id: {}'.format(node.id))
        return []
      area = Area()
      area.id = str(node.id)
      area.name = tags.get('ref')
      area.type = tags.get('type')
      data = self.api.get('node('+str(node.id)+');rel(bn:"topology");way(r._:"geometry");')
      if len(data.get('elements')) > 0:
        tags = data.get('elements')[0].get('tags')
      else:
        print('Missing information in a map for node with id: {}'.format(node.id))
        return False
      area.floor_number = tags.get('level')
      path.append(area)
    return path










