from fleet_management.db.models.environment import Position
from fmlib.models import robot as models
from fmlib.models.robot import Availability, ComponentStatus, RobotStatus
from pymodm import EmbeddedMongoModel, fields, MongoModel


class SensorCube(models.HardwareComponent):
    pass


class DockingMechanism(models.HardwareComponent):
    pass


class Platform(models.RobotHardware):
    docking_mechanism = fields.EmbeddedDocumentField(DockingMechanism)
    sensor_cubes = fields.EmbeddedDocumentListField(SensorCube)


class RopodSoftwareStack(models.SoftwareStack):
    diagnosis = fields.EmbeddedDocumentListField(models.SoftwareComponent)
    communication = fields.EmbeddedDocumentListField(models.SoftwareComponent)
    execution = fields.EmbeddedDocumentListField(models.SoftwareComponent)
    world_model = fields.EmbeddedDocumentListField(models.SoftwareComponent)
    uncategorized = fields.EmbeddedDocumentListField(models.SoftwareComponent)


class BlackBoxModel(MongoModel):
    black_box_id = fields.CharField(primary_key=True)
    version = fields.EmbeddedDocumentField(models.HardwareComponent)
    logger = fields.EmbeddedDocumentField(models.SoftwareComponent)
    fault_detection = fields.EmbeddedDocumentField(models.SoftwareComponent)


class RopodVersion(EmbeddedMongoModel):
    black_box = fields.EmbeddedDocumentField(BlackBoxModel)
    hardware = fields.EmbeddedDocumentField(Platform)
    software = fields.EmbeddedDocumentField(RopodSoftwareStack)


class Ropod(models.Robot):
    nickname = fields.CharField(default=None)
    version = fields.EmbeddedDocumentField(RopodVersion)
    position = fields.EmbeddedDocumentField(Position)

    @classmethod
    def create_new(cls, robot_id, **kwargs):
        robot = cls(robot_id, **kwargs)
        robot.position = Position()

        component_status = ComponentStatus()
        availability = Availability()
        hw_version = Platform()
        sw_version = RopodSoftwareStack()
        black_box = BlackBoxModel('black_box_001')  # TODO This shouldn't be hardcoded

        robot.status = RobotStatus(availability=availability, component_status=component_status)
        robot.version = RopodVersion(hardware=hw_version, software=sw_version, black_box=black_box)

        robot.save()

        return robot

    def update_position(self, **kwargs):
        self.position.update_position(**kwargs)
        self.save()

    def update_version(self, software, hardware):
        self.version.software = software
        self.save()
