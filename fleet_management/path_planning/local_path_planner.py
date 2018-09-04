from .node import Node
from .way import Way
from .router import Router
from fleet_management.structs.area import Area, Waypoint

class LocalPathPlanner:

  def __init__(self, path, api):
    self.api = api
    self.path = path
    self.way_pts = []

  def set_start_destination_locations(self, start, destination):
    start = start.split("_")
    destination = destination.split("_")
    if len(start) > 4 and len(destination) > 4:
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

      self.start_local = self.start_building + '_' + start_element + '_' + start[4]
      self.destination_local = self.destination_building + '_' + destination_element + '_' + destination[4]
      return True
    else:
      print("Invalid start and/or destination locations local area")
      return False


  def get_start_node(self):
    data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"level");relation[ref="' + self.start_floor + '"]; >;relation[ref="' + self.start_element + '"];relation(r._:"local_area");>;relation[ref="' + self.start_local + '"];node(r._:"topology");')
    if len(data.get('elements')) > 0:
      return Node(data.get('elements')[0])
    else:
      print('Start location {} does not exist'.format(self.start_local))
      return False

  def get_destination_node(self):
    data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"level");relation[ref="' + self.destination_floor + '"]; >;relation[ref="' + self.destination_element + '"];relation(r._:"local_area");>;relation[ref="' + self.destination_local + '"];node(r._:"topology");')
    if len(data.get('elements')) > 0:
      return Node(data.get('elements')[0])
    else:
      print('Destination location {} does not exist'.format(self.destination_local))
      return False

  def get_connections(self):
    ways = []
    for area in self.path:
      data = self.api.get('node(' + area.id + ');rel(bn:"topology");way(r._:"local_connection");')
      connections = data.get('elements')
      for connection in connections:
        w = Way(connection)
        for n in connection.get('nodes'):
          data = self.api.get('node('+ str(n) + ');')
          w.nodes.append(Node(data.get('elements')[0]))
        ways.append(w)    
    return ways


  def plan_path(self):
    start_node = self.get_start_node()
    destination_node = self.get_destination_node()

    if start_node and destination_node:
      connections = self.get_connections()
      local_router = Router(start_node, destination_node, connections)
      local_router.route()
      self.path_nodes = local_router.nodes
      return True
    else:
      print("[Invalid information] Cannot plan the path") 
      return False

  def prepare_path(self):
    for node in self.path_nodes:
      data = self.api.get('node(' + str(node.id) + ');rel(bn:"topology");relation(br:"local_area");node(r._:"topology");')
      if len(data.get('elements')) > 0:
        global_node = Node(data.get('elements')[0])
      else:
        print('Missing information in a map for node with id: {}'.format(node.id))
        return []

      for area in self.path:
        if area.id == str(global_node.id):
          data = self.api.get('node(' + str(node.id) + ');rel(bn:"topology");')
          if len(data.get('elements')) > 0:
            tags = data.get('elements')[0].get('tags')
          else:
            print('Missing information in a map for node with id: {}'.format(node.id))
            return False
          wap_pt = Waypoint()
          wap_pt.semantic_id = tags.get('ref')
          wap_pt.area_id = str(node.id)
          area.waypoints.append(wap_pt)
    return self.path











