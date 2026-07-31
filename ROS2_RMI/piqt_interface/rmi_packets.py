# rmi_packets.py

# =============================================================================
# IMPORTS
# =============================================================================

from dataclasses import dataclass, field, asdict
from typing import Optional, Union
import json

# =============================================================================
# BASE PACKET CLASS
# =============================================================================

class JsonPacket:

    """
    Base class used by all RMI packet definitions.

    Provides conversion to Python dictionaries and JSON strings.
    """

    def to_dict(self):
        return asdict(self)

    def to_json(self):

        return json.dumps(
            self.to_dict(),
            indent=2
        )
    
@dataclass
class FrameData:
    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0
    W: float = 0.0
    P: float = 0.0
    R: float = 0.0


@dataclass
class PositionData:
    X: float = 0.0
    Y: float = 0.0
    Z: float = 0.0
    W: float = 0.0
    P: float = 0.0
    R: float = 0.0
    Ext1: float = 0.0
    Ext2: float = 0.0
    Ext3: float = 0.0


@dataclass
class ConfigurationData:
    UToolNumber: int = 0
    UFrameNumber: int = 0
    Front: int = 0
    Up: int = 0
    Left: int = 0
    Flip: int = 0
    Turn4: int = 0
    Turn5: int = 0
    Turn6: int = 0


@dataclass
class JointAngleData:
    J1: float = 0.0
    J2: float = 0.0
    J3: float = 0.0
    J4: float = 0.0
    J5: float = 0.0
    J6: float = 0.0
    J7: float = 0.0
    J8: float = 0.0
    J9: float = 0.0

@dataclass
class ConnectPacket(JsonPacket):
    Communication: str = "FRC_Connect"


@dataclass
class ConnectROS2Packet(JsonPacket):
    Communication: str = "FRC_Connect_STMO"


@dataclass
class DisconnectPacket(JsonPacket):
    Communication: str = "FRC_Disconnect"


@dataclass
class TimeoutTerminatePacket(JsonPacket):
    Communication: str = "FRC_Terminate"


@dataclass
class CommunicationPacket(JsonPacket):
    Communication: str = "FRC_AsbnReady"


@dataclass
class SystemFaultPacket(JsonPacket):
    Communication: str = "FRC_SystemFault"
    SequenceID: int = 0

@dataclass
class InitializePacket(JsonPacket):
    Command: str = "FRC_Initialize"
    GroupMask: Optional[int] = None
    RTSA: Optional[str] = None
    PLTZMODE: Optional[str] = None


@dataclass
class AbortPacket(JsonPacket):
    Command: str = "FRC_Abort"


@dataclass
class PausePacket(JsonPacket):
    Command: str = "FRC_Pause"


@dataclass
class ContinuePacket(JsonPacket):
    Command: str = "FRC_Continue"


@dataclass
class ResetRobotPacket(JsonPacket):
    Command: str = "FRC_Reset"


@dataclass
class StatusRequestPacket(JsonPacket):
    Command: str = "FRC_GetStatus"


@dataclass
class GetExtendedStatusPacket(JsonPacket):
    Command: str = "FRC_GetExtStatus"

@dataclass
class SetSpeedOverridePacket(JsonPacket):

    Command: str = "FRC_SetOverRide"

    Value: int = 100

@dataclass
class ReadErrorPacket(JsonPacket):
    Command: str = "FRC_ReadError"
    Count: Optional[int] = None

@dataclass
class SetUFrameToolFramePacket(JsonPacket):
    Command: str = "FRC_SetUFrameUTool"
    UFrameNumber: int = 0
    UToolNumber: int = 0
    Group: Optional[int] = None


@dataclass
class GetUFrameToolFramePacket(JsonPacket):
    Command: str = "FRC_GetUFrameUTool"
    Group: Optional[int] = None


@dataclass
class ReadUFrameDataPacket(JsonPacket):
    Command: str = "FRC_ReadUFrameData"
    FrameNumber: int = 1
    Group: Optional[int] = None


@dataclass
class WriteUFrameDataPacket(JsonPacket):
    Command: str = "FRC_WriteUFrameData"
    FrameNumber: int = 1
    Frame: FrameData = field(default_factory=FrameData)
    Group: Optional[int] = None


@dataclass
class ReadUToolDataPacket(JsonPacket):
    Command: str = "FRC_ReadUToolData"
    ToolNumber: int = 1
    Group: Optional[int] = None


@dataclass
class WriteUToolDataPacket(JsonPacket):
    Command: str = "FRC_WriteUToolData"
    ToolNumber: int = 1
    Frame: FrameData = field(default_factory=FrameData)
    Group: Optional[int] = None

@dataclass
class ReadJointAnglesPacket(JsonPacket):
    Command: str = "FRC_ReadJointAngles"
    Group: Optional[int] = None


@dataclass
class GetCartesianPositionPacket(JsonPacket):
    Command: str = "FRC_ReadCartesianPosition"
    Group: Optional[int] = None


@dataclass
class GetTCPSpeedPacket(JsonPacket):
    Command: str = "FRC_ReadTCPSpeed"

@dataclass
class ReadNumericRegisterPacket(JsonPacket):
    Command: str = "FRC_ReadRegister"
    RegisterNumber: int = 1


@dataclass
class WriteNumericRegisterPacket(JsonPacket):
    Command: str = "FRC_WriteRegister"
    RegisterNumber: int = 1
    RegisterValue: Union[int, float] = 0
    DataType: str = "Integer"

@dataclass
class ReadPositionRegisterPacket(JsonPacket):
    Command: str = "FRC_ReadPositionRegister"
    RegisterNumber: int = 1
    Group: Optional[int] = None


@dataclass
class WritePositionRegisterPacket(JsonPacket):
    Command: str = "FRC_WritePositionRegister"
    RegisterNumber: int = 1

    Representation: Optional[str] = None

    Configuration: Optional[ConfigurationData] = None

    Position: Optional[PositionData] = None

    JointAngle: Optional[JointAngleData] = None

    Group: Optional[int] = None

@dataclass
class ReadDigitalInputPortPacket(JsonPacket):
    Command: str = "FRC_ReadDIN"
    PortNumber: int = 1


@dataclass
class WriteDigitalOutputPacket(JsonPacket):
    Command: str = "FRC_WriteDOUT"
    PortNumber: int = 1
    PortValue: str = "OFF"


@dataclass
class ReadIOPortPacket(JsonPacket):
    Command: str = "FRC_ReadIOPort"
    PortType: str = "DO"
    PortNumber: int = 1


@dataclass
class WriteIOPortPacket(JsonPacket):
    Command: str = "FRC_WriteIOPort"
    PortType: str = "DO"
    PortNumber: int = 1
    PortValue: Union[int, float] = 0

@dataclass
class ReadVariablePacket(JsonPacket):
    Command: str = "FRC_ReadVariable"
    VariableName: str = ""


@dataclass
class WriteVariablePacket(JsonPacket):
    Command: str = "FRC_WriteVariable"
    VariableName: str = ""
    VariableType: str = "Integer"
    VariableValue: Union[int, float] = 0

@dataclass
class SetPayloadPacket(JsonPacket):
    Command: str = "FRC_SetPayloadID"
    ScheduleNumber: int = 1
    Group: Optional[int] = None


@dataclass
class SetPayloadValuePacket(JsonPacket):

    Command: str = "FRC_SetPayloadValue"

    ScheduleNumber: int = 1

    Group: Optional[int] = None

    Mass: float = 0.0

    CG_X: float = 0.0
    CG_Y: float = 0.0
    CG_Z: float = 0.0

    IN_X: Optional[float] = None
    IN_Y: Optional[float] = None
    IN_Z: Optional[float] = None


@dataclass
class SetPayloadCompPacket(JsonPacket):

    Command: str = "FRC_SetPayloadComp"

    ScheduleNumber: int = 1

    Group: Optional[int] = None

    Mass: float = 0.0

    CG_X: float = 0.0
    CG_Y: float = 0.0
    CG_Z: float = 0.0

    IN_X: float = 0.0
    IN_Y: float = 0.0
    IN_Z: float = 0.0

@dataclass
class JointMotionJRepPacket(JsonPacket):

    Instruction: str = "FRC_JointMotionJRep"

    SequenceID: int = 1

    JointAngle: JointAngleData = field(
        default_factory=JointAngleData
    )

    SpeedType: str = "%"

    Speed: int = 50

    TermType: str = "CNT"

    TermValue: int = 100

@dataclass
class WaitForDINPacket(JsonPacket):

    Instruction: str = "FRC_WaitDIN"

    SequenceID: int = 1

    PortNumber: int = 1

    PortValue: str = "ON"

@dataclass
class SetUFramePacket(JsonPacket):

    Instruction: str = "FRC_SetUFrame"

    SequenceID: int = 1

    FrameNumber: int = 1

@dataclass
class SetToolFramePacket(JsonPacket):

    Instruction: str = "FRC_SetUTool"

    SequenceID: int = 1

    ToolNumber: int = 1

@dataclass
class WaitForTimePacket(JsonPacket):

    Instruction: str = "FRC_WaitTime"

    SequenceID: int = 1

    Time: float = 1.0

@dataclass
class ProgramCallPacket(JsonPacket):

    Instruction: str = "FRC_Call"

    SequenceID: int = 1

    ProgramName: str = ""

@dataclass
class JointMotionPacket(JsonPacket):

    Instruction: str = "FRC_JointMotion"

    SequenceID: int = 1

    Configuration: ConfigurationData = field(
        default_factory=ConfigurationData
    )

    Position: PositionData = field(
        default_factory=PositionData
    )

    SpeedType: str = "%"

    Speed: int = 50

    TermType: str = "CNT"

    TermValue: int = 100

    ACC: Optional[int] = None

    OffsetPRNumber: Optional[int] = None

    VisionPRNumber: Optional[int] = None

    MROT: Optional[str] = None

    LCBType: Optional[str] = None

    LCBValue: Optional[int] = None

    PortType: Optional[int] = None

    portNumber: Optional[int] = None

    portValue: Optional[str] = None

    ToolOffsetPRNumber: Optional[int] = None

    NoBlend: Optional[str] = None

@dataclass
class JointRelativePacket(JsonPacket):

    Instruction: str = "FRC_JointRelative"

    SequenceID: int = 1

    Configuration: ConfigurationData = field(
        default_factory=ConfigurationData
    )

    Position: PositionData = field(
        default_factory=PositionData
    )

    SpeedType: str = "%"

    Speed: int = 50

    TermType: str = "CNT"

    TermValue: int = 100

    ACC: Optional[int] = None

    OffsetPRNumber: Optional[int] = None

    VisionPRNumber: Optional[int] = None

    MROT: Optional[str] = None

    LCBType: Optional[str] = None

    LCBValue: Optional[int] = None

    PortType: Optional[int] = None

    portNumber: Optional[int] = None

    portValue: Optional[str] = None

    ToolOffsetPRNumber: Optional[int] = None

    NoBlend: Optional[str] = None

@dataclass
class LinearRelativePacket(JsonPacket):

    Instruction: str = "FRC_LinearRelative"

    SequenceID: int = 1

    Configuration: ConfigurationData = field(
        default_factory=ConfigurationData
    )

    Position: PositionData = field(
        default_factory=PositionData
    )

    SpeedType: str = "mm/sec"

    Speed: int = 100

    TermType: str = "CNT"

    TermValue: int = 100

    ACC: Optional[int] = None

    OffsetPRNumber: Optional[int] = None

    VisionPRNumber: Optional[int] = None

    WristJoint: Optional[str] = None

    MROT: Optional[str] = None

    LCBType: Optional[str] = None

    LCBValue: Optional[int] = None

    PortType: Optional[int] = None

    portNumber: Optional[int] = None

    portValue: Optional[str] = None

    ToolOffsetPRNumber: Optional[int] = None

    ALIM: Optional[int] = None

    ALIMREG: Optional[int] = None

    NoBlend: Optional[str] = None

@dataclass
class CircularMotionPacket(JsonPacket):

    Instruction: str = "FRC_CircularMotion"

    SequenceID: int = 1

    Configuration: ConfigurationData = field(
        default_factory=ConfigurationData
    )

    Position: PositionData = field(
        default_factory=PositionData
    )

    ViaConfiguration: ConfigurationData = field(
        default_factory=ConfigurationData
    )

    ViaPosition: PositionData = field(
        default_factory=PositionData
    )

    SpeedType: str = "mm/sec"

    Speed: int = 100

    TermType: str = "CNT"

    TermValue: int = 100

    ACC: Optional[int] = None

    OffsetPRNumber: Optional[int] = None

    VisionPRNumber: Optional[int] = None

    WristJoint: Optional[str] = None

    MROT: Optional[str] = None

    LCBType: Optional[str] = None

    LCBValue: Optional[int] = None

    PortType: Optional[int] = None

    portNumber: Optional[int] = None

    portValue: Optional[str] = None

    ToolOffsetPRNumber: Optional[int] = None

    NoBlend: Optional[str] = None

@dataclass
class CircularRelativePacket(JsonPacket):

    Instruction: str = "FRC_CircularRelative"

    SequenceID: int = 1

    Configuration: ConfigurationData = field(
        default_factory=ConfigurationData
    )

    Position: PositionData = field(
        default_factory=PositionData
    )

    ViaConfiguration: ConfigurationData = field(
        default_factory=ConfigurationData
    )

    ViaPosition: PositionData = field(
        default_factory=PositionData
    )

    SpeedType: str = "mm/sec"

    Speed: int = 100

    TermType: str = "CNT"

    TermValue: int = 100

    ACC: Optional[int] = None

    OffsetPRNumber: Optional[int] = None

    VisionPRNumber: Optional[int] = None

    WristJoint: Optional[str] = None

    MROT: Optional[str] = None

    LCBType: Optional[str] = None

    LCBValue: Optional[int] = None

    PortType: Optional[int] = None

    portNumber: Optional[int] = None

    portValue: Optional[str] = None

    ToolOffsetPRNumber: Optional[int] = None

    NoBlend: Optional[str] = None

@dataclass
class LinearMotionJRepPacket(JsonPacket):

    Instruction: str = "FRC_LinearMotionJRep"

    SequenceID: int = 1

    JointAngle: JointAngleData = field(
        default_factory=JointAngleData
    )

    SpeedType: str = "mm/sec"

    Speed: int = 100

    TermType: str = "CNT"

    TermValue: int = 100

@dataclass
class LinearRelativeJRepPacket(JsonPacket):

    Instruction: str = "FRC_LinearRelativeJRep"

    SequenceID: int = 1

    JointAngle: JointAngleData = field(
        default_factory=JointAngleData
    )

    SpeedType: str = "mm/sec"

    Speed: int = 100

    TermType: str = "CNT"

    TermValue: int = 100

@dataclass
class SplineMotionPacket(JsonPacket):

    Instruction: str = "FRC_SplineMotion"

    SequenceID: int = 1

    Configuration: ConfigurationData = field(
        default_factory=ConfigurationData
    )

    Position: PositionData = field(
        default_factory=PositionData
    )

    SpeedType: str = "mm/sec"

    Speed: int = 100

    TermType: str = "CNT"

    TermValue: int = 100

    ACC: Optional[int] = None

    OffsetPRNumber: Optional[int] = None

    VisionPRNumber: Optional[int] = None

    LCBType: Optional[str] = None

    LCBValue: Optional[int] = None

    PortType: Optional[int] = None

    portNumber: Optional[int] = None

    portValue: Optional[str] = None

    ToolOffsetPRNumber: Optional[int] = None

@dataclass
class SplineMotionJRepPacket(JsonPacket):

    Instruction: str = "FRC_SplineMotionJRep"

    SequenceID: int = 1

    JointAngle: JointAngleData = field(
        default_factory=JointAngleData
    )

    SpeedType: str = "mm/sec"

    Speed: int = 100

    TermType: str = "CNT"

    TermValue: int = 100

    ACC: Optional[int] = None

    OffsetPRNumber: Optional[int] = None

    VisionPRNumber: Optional[int] = None

    LCBType: Optional[str] = None

    LCBValue: Optional[int] = None

    PortType: Optional[int] = None

    portNumber: Optional[int] = None

    portValue: Optional[str] = None

    ToolOffsetPRNumber: Optional[int] = None

@dataclass
class LinearMotionPacket(JsonPacket):

    Instruction: str = "FRC_LinearMotion"

    SequenceID: int = 1

    Configuration: ConfigurationData = field(
        default_factory=ConfigurationData
    )

    Position: PositionData = field(
        default_factory=PositionData
    )

    SpeedType: str = "mm/sec"

    Speed: int = 100

    TermType: str = "CNT"

    TermValue: int = 100

    ACC: Optional[int] = None

    OffsetPRNumber: Optional[int] = None

    VisionPRNumber: Optional[int] = None

    WristJoint: Optional[str] = None

    MROT: Optional[str] = None

    LCBType: Optional[str] = None

    LCBValue: Optional[int] = None

    PortType: Optional[int] = None

    portNumber: Optional[int] = None

    portValue: Optional[str] = None

    ToolOffsetPRNumber: Optional[int] = None

    ALIM: Optional[int] = None

    ALIMREG: Optional[int] = None

    NoBlend: Optional[str] = None

@dataclass
class JointRelativeJRepPacket(JsonPacket):

    Instruction: str = "FRC_JointRelativeJRep"

    SequenceID: int = 1

    JointAngle: JointAngleData = field(
        default_factory=JointAngleData
    )

    SpeedType: str = "%"

    Speed: int = 50

    TermType: str = "CNT"

    TermValue: int = 100

    ACC: Optional[int] = None

    OffsetPRNumber: Optional[int] = None

    VisionPRNumber: Optional[int] = None

    MROT: Optional[str] = None

    LCBType: Optional[str] = None

    LCBValue: Optional[int] = None

    PortType: Optional[int] = None

    portNumber: Optional[int] = None

    portValue: Optional[str] = None

    ToolOffsetPRNumber: Optional[int] = None

    NoBlend: Optional[str] = None

@dataclass
class UnknownPacket(JsonPacket):

    Command: str = "Unknown"

    ErrorID: int = 0

    SequenceID: Optional[int] = None

@dataclass
class CreateASCIIPacket(JsonPacket):

    ASCII: str = ""

    SequenceID: int = 1

PACKET_REGISTRY = {
    "FRC_Connect": ConnectPacket,
    "FRC_Connect_STMO": ConnectROS2Packet,
    "FRC_Disconnect": DisconnectPacket,

    "FRC_Initialize": InitializePacket,
    "FRC_Abort": AbortPacket,
    "FRC_Pause": PausePacket,
    "FRC_Continue": ContinuePacket,
    "FRC_Reset": ResetRobotPacket,

    "FRC_GetStatus": StatusRequestPacket,
    "FRC_GetExtStatus": GetExtendedStatusPacket,
    "FRC_ReadError": ReadErrorPacket,
    "FRC_SetOverRide": SetSpeedOverridePacket,

    "FRC_ReadJointAngles": ReadJointAnglesPacket,
    "FRC_ReadCartesianPosition": GetCartesianPositionPacket,
    "FRC_ReadTCPSpeed": GetTCPSpeedPacket,

    "FRC_ReadRegister": ReadNumericRegisterPacket,
    "FRC_WriteRegister": WriteNumericRegisterPacket,

    "FRC_ReadPositionRegister": ReadPositionRegisterPacket,
    "FRC_WritePositionRegister": WritePositionRegisterPacket,

    "FRC_ReadDIN": ReadDigitalInputPortPacket,
    "FRC_WriteDOUT": WriteDigitalOutputPacket,

    "FRC_ReadIOPort": ReadIOPortPacket,
    "FRC_WriteIOPort": WriteIOPortPacket,

    "FRC_ReadVariable": ReadVariablePacket,
    "FRC_WriteVariable": WriteVariablePacket,

    "FRC_SetPayloadID": SetPayloadPacket,
    "FRC_SetPayloadValue": SetPayloadValuePacket,
    "FRC_SetPayloadComp": SetPayloadCompPacket,

    "FRC_SetUFrameUTool": SetUFrameToolFramePacket,
    "FRC_GetUFrameUTool": GetUFrameToolFramePacket,

    "FRC_ReadUFrameData": ReadUFrameDataPacket,
    "FRC_WriteUFrameData": WriteUFrameDataPacket,

    "FRC_ReadUToolData": ReadUToolDataPacket,
    "FRC_WriteUToolData": WriteUToolDataPacket,

    "FRC_WaitDIN": WaitForDINPacket,
    "FRC_SetUFrame": SetUFramePacket,
    "FRC_SetUTool": SetToolFramePacket,
    "FRC_WaitTime": WaitForTimePacket,
    "FRC_Call": ProgramCallPacket,

    "FRC_LinearMotion": LinearMotionPacket,
    "FRC_LinearRelative": LinearRelativePacket,

    "FRC_JointMotion": JointMotionPacket,
    "FRC_JointRelative": JointRelativePacket,

    "FRC_CircularMotion": CircularMotionPacket,
    "FRC_CircularRelative": CircularRelativePacket,

    "FRC_JointMotionJRep": JointMotionJRepPacket,
    "FRC_JointRelativeJRep": JointRelativeJRepPacket,

    "FRC_LinearMotionJRep": LinearMotionJRepPacket,
    "FRC_LinearRelativeJRep": LinearRelativeJRepPacket,

    "FRC_SplineMotion": SplineMotionPacket,
    "FRC_SplineMotionJRep": SplineMotionJRepPacket,

    "Unknown": UnknownPacket,
    "ASCII": CreateASCIIPacket,

}

def create_packet(command):

    packet_class = PACKET_REGISTRY.get(command)

    if packet_class is None:
        raise ValueError(
            f"Unknown packet: {command}"
        )

    return packet_class()

def packet_fields(command):

    from dataclasses import fields

    packet = PACKET_REGISTRY[command]

    return [
        f.name
        for f in fields(packet)
    ]