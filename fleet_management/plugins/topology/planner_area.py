
class PlannerArea:
    def __init__(self, pt):
        self.type = pt[1]["type"]
        self.id = pt[1]["topology_id"]
        self.ref = pt[1]["label"]
        self.level = pt[1]["floor_number"]
        self.navigation_areas = None
        self.exit_door = None

    def __repr__(self) :
        s = "\n<"
        s += "type:" + str(self.type) + ", "
        s += "id:" + str(self.id) + ", "
        s += "ref:" + str(self.ref) + ", "
        s += "navigation_areas:[" 
        for i in self.navigation_areas :
            s += "(id:" + str(i['id'])  + ", "
            s += "ref:" + str(i['label']) + "), "
        s += "], " 
        s += "exit_door " + str(self.exit_door)
        s += ">"
        return s