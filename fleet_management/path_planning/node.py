from .visited_node import VisitedNode

class Node():
	def __init__(self, elm):
		self.element = elm

	@property
	def id(self):
		return self.element.get('id')

	@property
	def lat(self):
		return float(self.element.get('lat'))

	@property
	def lon(self):
		return float(self.element.get('lon'))

	def __eq__(self, other):
		if isinstance(other, VisitedNode):
			return self.id == other.node.id
		if isinstance(other, (Node)):
			return self.id == other.id

	def __repr__(self):
		return "<Node id=%(id)s, lat=%(lat)s, lon=%(lon)s>" % {
				'id': self.id,
				'lat': self.lat,
				'lon': self.lon,
		}