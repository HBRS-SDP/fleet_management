import uuid

from pymodm import EmbeddedMongoModel, fields, MongoModel


class HardwareComponent(EmbeddedMongoModel):
    uuid = fields.UUIDField(primary_key=True, default=uuid.uuid4())
    id = fields.CharField()
    model = fields.CharField()
    serial_number = fields.CharField(default='unknown')
    firmware_version = fields.CharField(default='unknown')
    version = fields.CharField(default='unknown')


class Wheel(HardwareComponent):
    pass


class Laser(HardwareComponent):
    pass


class RobotHardware(MongoModel):
    wheels = fields.EmbeddedDocumentListField(Wheel)
    laser = fields.EmbeddedDocumentListField(Laser)


class SoftwareComponent(EmbeddedMongoModel):
    name = fields.CharField(primary_key=True)
    package = fields.CharField()
    version = fields.CharField()


class SoftwareStack(MongoModel):
    navigation_stack = fields.EmbeddedDocumentListField(SoftwareComponent)
    interfaces = fields.EmbeddedDocumentListField(SoftwareComponent)


class Version(EmbeddedMongoModel):
    hardware = fields.EmbeddedDocumentField(RobotHardware)
    software = fields.EmbeddedDocumentField(SoftwareStack)
