from fleet_management.db.models.environment import Position
from fmlib.models.robot import HardwareComponent, Robot, RobotHardware, SoftwareComponent, SoftwareStack
from pymodm import EmbeddedMongoModel, fields, MongoModel


class SensorCube(HardwareComponent):
    pass


class DockingMechanism(HardwareComponent):
    pass


class Platform(RobotHardware):
    docking_mechanism = fields.EmbeddedDocumentField(DockingMechanism)
    sensor_cubes = fields.EmbeddedDocumentListField(SensorCube)


class RopodSoftwareStack(SoftwareStack):
    diagnosis = fields.EmbeddedDocumentListField(SoftwareComponent)
    communication = fields.EmbeddedDocumentListField(SoftwareComponent)
    execution = fields.EmbeddedDocumentListField(SoftwareComponent)


class BlackBoxModel(MongoModel):
    black_box_id = fields.CharField(primary_key=True)
    version = fields.EmbeddedDocumentField(HardwareComponent)
    logger = fields.EmbeddedDocumentField(SoftwareComponent)
    fault_detection = fields.EmbeddedDocumentField(SoftwareComponent)


class RopodVersion(EmbeddedMongoModel):
    black_box = fields.EmbeddedDocumentField(BlackBoxModel)
    hardware = fields.EmbeddedDocumentField(Platform)
    software = fields.EmbeddedDocumentField(RopodSoftwareStack)


class Ropod(Robot):
    nickname = fields.CharField(default=None)
    version = fields.EmbeddedDocumentField(RopodVersion)
    position = fields.EmbeddedDocumentField(Position)
