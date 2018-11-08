class SubArea(object):
    def __init__(self):
        self.id = ''
        self.name = ''
        self.x = -1.
        self.y = -1.

    def to_dict(self):
        subarea_dict = dict()
        subarea_dict['name'] = self.name
        subarea_dict['id'] = self.id
        subarea_dict['x'] = self.x
        subarea_dict['y'] = self.y
        return subarea_dict

    @staticmethod
    def from_dict(waypoint_dict):
        subarea = SubArea()
        subarea.name = subarea_dict['name']
        subarea.id = subarea_dict['id']
        subarea.x = subarea_dict['x']
        subarea.y = subarea_dict['y']
        return subarea


class Area(object):
    def __init__(self):
        self.id = ''
        self.name = ''
        self.subareas = list()
        self.floor_number = 0
        self.type = ''

    def to_dict(self):
        area_dict = dict()
        area_dict['id'] = self.id
        area_dict['name'] = self.name
        area_dict['floor_number'] = self.floor_number
        area_dict['subareas'] = list()
        area_dict['type'] = self.type
        for subarea in self.subareas:
            area_dict['subareas'].append(subarea.to_dict())
        return area_dict

    @staticmethod
    def from_dict(area_dict):
        area = Area()
        area.id = area_dict['id']
        area.name = area_dict['name']
        area.floor_number = area_dict['floor_number']
        area.type = area_dict['type']
        for subareas_dict in area_dict['subareas']:
            subareas = SubArea.from_dict(subareas_dict)
            area.subareas.append(subarea)
        return area
