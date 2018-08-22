from .node import Node
from .way import Way
from .router import Router

class GlobalPathPlanner(object):

	def __init__(self,organisation_ref, api):
		self.api = api
		self.organisation_ref = organisation_ref
		self.path_elements_ids = []
		self.path_nodes = []

	def set_start_position(self, building_ref, floor_ref, element_ref):
		self.start_floor = self.organisation_ref + "_" + floor_ref
		self.start_building = self.organisation_ref + "_" + building_ref +"_" + floor_ref
		self.start_element = self.start_building + '_' + element_ref

	def set_destination_position(self, building_ref, floor_ref, element_ref):
		self.destination_floor = self.organisation_ref + "_" + floor_ref
		self.destination_building = self.organisation_ref + "_" + building_ref +"_" + floor_ref
		self.destination_element = self.destination_building + '_' + element_ref


	def get_start_node(self):
		data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"level");relation[ref="' + self.start_floor + '"]; >;relation[ref="' + self.start_element + '"];node(r._:"topology");')
		if len(data.get('elements')) > 0:
			return Node(data.get('elements')[0])
		else:
			return None

	def get_destination_node(self):
		data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"level");relation[ref="' + self.destination_floor + '"]; >;relation[ref="' + self.destination_element + '"];node(r._:"topology");')
		if len(data.get('elements')) > 0:
			return Node(data.get('elements')[0])
		else:
			return None

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

		if start_node is not None and destination_node is not None: 
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
		else:
			print("Invalid start or destination room/corridor")
		

	def fetch_path_info(self):
		for node in self.path_nodes:
			data = self.api.get('node('+str(node.id)+');rel(bn:"topology");')
			tags = data.get('elements')[0].get('tags')
			self.path_elements_ids.append(data.get('elements')[0].get('id'))
			#print(tags.get('type') + ":" + tags.get('ref'))










