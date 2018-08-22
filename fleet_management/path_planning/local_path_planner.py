from .node import Node
from .way import Way
from .router import Router

class LocalPathPlanner:

	def __init__(self,organisation_ref, path_ids, api):
		self.api = api
		self.organisation_ref = organisation_ref
		self.path_ids = path_ids
		self.path_nodes = []

	def set_start_position(self, building_ref, floor_ref, element_ref, local_ref):
		self.start_floor = self.organisation_ref + "_" + floor_ref
		self.start_building = self.organisation_ref + "_" + building_ref +"_" + floor_ref
		self.start_element = self.start_building + '_' + element_ref 
		self.start_local = self.start_building + '_' + element_ref + '_' + local_ref

	def set_destination_position(self, building_ref, floor_ref, element_ref, local_ref):
		self.destination_floor = self.organisation_ref + "_" + floor_ref
		self.destination_building = self.organisation_ref + "_" + building_ref +"_" + floor_ref
		self.destination_element = self.destination_building + '_' + element_ref 
		self.destination_local = self.destination_building + '_' + element_ref + '_' + local_ref


	def get_start_node(self):
		data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"level");relation[ref="' + self.start_floor + '"]; >;relation[ref="' + self.start_element + '"];relation(r._:"local_area");>;relation[ref="' + self.start_local + '"];node(r._:"topology");')
		if len(data.get('elements')) > 0:
			return Node(data.get('elements')[0])
		else:
			return None

	def get_destination_node(self):
		data = self.api.get('relation[ref="' + self.organisation_ref + '"]; relation(r._:"level");relation[ref="' + self.destination_floor + '"]; >;relation[ref="' + self.destination_element + '"];relation(r._:"local_area");>;relation[ref="' + self.destination_local + '"];node(r._:"topology");')
		if len(data.get('elements')) > 0:
			return Node(data.get('elements')[0])
		else:
			return None

	def get_connections(self):
		ways = []
		for element_id in self.path_ids:
			data = self.api.get('relation(' + str(element_id) + ');way(r._:"local_connection");')
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
		if start_node is not None and destination_node is not None: 
			connections = self.get_connections()
			local_router = Router(start_node, destination_node, connections)
			local_router.route()
			self.path_nodes = local_router.nodes
		else:
			print('Invalid source or destination local area')


	def fetch_path_info(self):
		path_list = []
		for node in self.path_nodes:
			data1 = self.api.get('node('+str(node.id)+');rel(bn:"topology");')
			tags = data1.get('elements')[0].get('tags')
			name = tags.get('ref')
			data2 = self.api.get('node('+str(node.id)+');rel(bn:"topology");way(r:"geometry");')
			waypoints = data2.get('elements')[0].get('nodes')
			level = data2.get('elements')[0].get('tags').get('level')
			path_list.append([node.id, name, waypoints, level])
			#print(name,"|",node.id,"|",waypoints,",",level)
		return path_list












