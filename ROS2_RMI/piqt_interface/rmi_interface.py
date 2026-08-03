#!/usr/bin/env python3

"""
FANUC RMI Interface

High-level module description
-----------------------------
This module implements a PyQt-based operator interface for FANUC robot
controllers using the FANUC Remote Motion Interface (RMI). It provides a
separation between UI logic, packet construction, and low-level socket
communication so the UI can dynamically build forms from packet
definitions and transmit them to the controller over RMI.

Key responsibilities
- Establish and manage the FANUC RMI two-step handshake and session socket
- Provide `RMIClient` for JSON packet send/receive and sequence tracking
- Run a background `ReceiverWorker` in a Qt `QThread` to avoid blocking the UI
- Provide `MainWindow` which generates dynamic command forms from packet
    dataclasses, validates inputs, constructs packets, and displays responses
- Integrate with ROS2 via `RmiInterfaceNode` while allowing Qt and ROS2
    callbacks to coexist using `rclpy.spin_once()` on a Qt timer

Sequence ID handling
- This module tracks sequence state (last sent, last received, and the next outgoing
    sequence) and displays them in the UI. Sequence assignment is automatic —
    users do not manually edit SequenceIDs for motion commands.

Usage notes
- Packet definitions are in `piqt_interface.rmi_packets` and the UI will
    automatically discover and render input widgets for registered packets.
- The code intentionally prints TX/RX JSON for visibility; moving to Python's
    `logging` module is recommended for production.

Author: John Castellani
"""

# =============================================================================
# Standard Library Imports
# =============================================================================
import sys
import socket
import json

from typing import (
    get_origin,
    get_args,
    Union
)

from dataclasses import fields

# =============================================================================
# ROS2 Imports
# =============================================================================
import rclpy
from rclpy.node import Node

# =============================================================================
# PyQt Imports
# =============================================================================
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QLabel,
    QLineEdit,
    QCheckBox,
    QWidget,
    QHBoxLayout,
    QPushButton,
    QComboBox
)

from PyQt5.QtCore import (
    QTimer,
    QObject,
    QThread,
    pyqtSignal
)

# =============================================================================
# Application Imports
# =============================================================================
from piqt_interface.main_window import (
    Ui_MainWindow
)

from piqt_interface.rmi_packets import (
    PACKET_REGISTRY,
    create_packet,
    packet_fields,
    ConnectROS2Packet,
    PositionData,
    JointAngleData,
    ConfigurationData,
    FrameData
)

# =============================================================================
# Constants
# =============================================================================

DEFAULT_ROBOT_IP = "192.168.1.100"

DEFAULT_USER_FRAME = 0
DEFAULT_TOOL_FRAME = 1

DEFAULT_TURN_VALUE = 0

DEFAULT_GROUP_NUMBER = 1

DEFAULT_SEQUENCE_ID = 1

SOCKET_TIMEOUT = 300
COMMAND_TIMEOUT = 300

RMI_HANDSHAKE_PORT = 16001

# =============================================================================
# Utility Functions
# =============================================================================
def is_optional_type(field_type):
    """
    Determine if a packet field is Optional.

    Used when generating dynamic packet widgets.
    """

    return (
        get_origin(field_type) is Union
        and type(None) in get_args(field_type)
    )

class RMIClient:
    """
    FANUC RMI communication client.

    Responsibilities:
        - Establishing the RMI handshake
        - Opening the session socket
        - Sending packets
        - Receiving packets
        - Tracking robot SequenceIDs

    This class contains no UI logic and is responsible only
    for communication with the robot controller.
    """

    def __init__(self):

        self.socket = None
        self.connected = False

        self.port = RMI_HANDSHAKE_PORT

        self.sequence_id = None
        self.last_received_sequence_id = None
        self.expected_sequence_id = None

    def reset_transport(self):
        """Close any active socket and restore default transport state."""

        if self.socket is not None:

            try:

                self.socket.close()

            except OSError:

                pass

            self.socket = None

        self.connected = False
        self.port = RMI_HANDSHAKE_PORT

    # -------------------------------------------------------------------------
    # Connection Management
    # -------------------------------------------------------------------------
    def connect(self, robot_ip):
        """
        Connect to a FANUC RMI server.

        FANUC RMI uses a two-step connection process:

        1. Connect to the fixed handshake port (16001)
        2. Send a ConnectROS2Packet
        3. Receive an assigned session port
        4. Open a second socket to the assigned port

        All subsequent RMI communication occurs on the
        assigned session port.
        """

        self.robot_ip = robot_ip
        self.reset_transport()

        try:

            # Create a temporary handshake connection to the robot.
            self.socket = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            self.socket.settimeout(
                SOCKET_TIMEOUT
            )

            self.socket.connect(
                (
                    robot_ip,
                    RMI_HANDSHAKE_PORT
                )
            )

            #
            # Send FANUC connection request
            #
            connect_packet = ConnectROS2Packet()

            connect_json = (
                connect_packet.to_json()
                + "\r\n"
            )

            print(
                f"TX: {connect_json}"
            )

            self.socket.sendall(
                connect_json.encode("utf-8")
            )

            #
            # Receive connection response
            #
            try:

                data = self.socket.recv(
                    4096
                )

            except socket.timeout:

                raise RuntimeError(
                    "Robot response timeout exceeded "
                    f"({SOCKET_TIMEOUT} seconds)"
                )

            if not data:

                raise RuntimeError(
                    "Robot closed the handshake connection"
                )

            response = data.decode(
                "utf-8"
            )

            print(
                f"CONNECT RESPONSE: {response}"
            )

            response_json = json.loads(
                response
            )

            if response_json["ErrorID"] != 0:

                raise RuntimeError(
                    f"RMI Error: {response_json['ErrorID']}"
                )

            new_port = response_json[
                "PortNumber"
            ]

            print(
                f"RMI Version: "
                f"{response_json['MajorVersion']}."
                f"{response_json['MinorVersion']}"
            )

            print(
                f"Assigned RMI Port: {new_port}"
            )

            #
            # Close the handshake socket
            #
            self.socket.close()

            #
            # Create the actual RMI session socket
            #
            self.socket = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            self.socket.settimeout(
                COMMAND_TIMEOUT
            )

            self.socket.connect(
                (
                    robot_ip,
                    new_port
                )
            )

            self.port = new_port

            print(
                f"Connected to RMI Port: {new_port}"
            )

            self.connected = True

        except Exception:

            self.reset_transport()
            raise

    def disconnect(self):
        """
        Disconnect from the FANUC RMI server.

        Sends a FRC_Disconnect packet before closing the socket.
        The socket is closed and the connected flag is cleared in
        the finally block regardless of whether the send succeeds.
        """

        try:

            if self.connected:

                packet = create_packet(
                    "FRC_Disconnect"
                )

                json_data = (
                    packet.to_json()
                    + "\r\n"
                )

                if (
                    '"Command": "FRC_GetStatus"'
                    not in json_data
                ):
                    print(
                        f"TX: {json_data}"
                    )

                self.socket.sendall(
                    json_data.encode("utf-8")
                )

        except Exception as e:

            print(e)

        finally:

            self.reset_transport()

    # -------------------------------------------------------------------------
    # Communication Functions
    # -------------------------------------------------------------------------
    def send_packet(self, packet):
        """
        Serialize a packet to JSON and transmit it over the RMI socket.

        Raises RuntimeError if the client is not connected.
        """

        if not self.connected:

            raise RuntimeError(
                "Not connected"
            )

        json_data = (
            packet.to_json()
            + "\r\n"
        )

        print(
            f"TX: {json_data}"
        )

        try:

            self.socket.sendall(
                json_data.encode("utf-8")
            )

        except OSError as exc:

            self.reset_transport()

            raise RuntimeError(
                f"Socket send failed: {exc}"
            ) from exc

    def receive(self):
        """
        Block until one packet is received from the RMI socket.

        Returns the decoded response as a dict when the payload is
        valid JSON, otherwise returns the raw string.
        Raises RuntimeError on timeout or if not connected.
        """

        if not self.connected:

            raise RuntimeError(
                "Not connected"
            )
        
        try:

            data = self.socket.recv(
                4096
            )

        except socket.timeout:

            self.reset_transport()

            raise RuntimeError(
                f"Robot response timeout "
                f"({COMMAND_TIMEOUT} seconds)"
            )

        except OSError as exc:

            self.reset_transport()

            raise RuntimeError(
                f"Socket receive failed: {exc}"
            ) from exc

        if not data:

            self.reset_transport()

            raise RuntimeError(
                "Robot closed the connection"
            )

        response = data.decode(
            "utf-8"
        )

        if (
            '"Command" : "FRC_GetStatus"'
            not in response
        ):
            print(
                f"RX: {response}"
            )

        try:

            response_json = json.loads(
                response
            )

            self.update_sequence_id(
                response_json
            )

            return response_json

        except Exception:

            return response
    
    # -------------------------------------------------------------------------
    # Sequence ID Handling
    # -------------------------------------------------------------------------
    def update_sequence_id(self, response):
        """
        Track the latest robot SequenceID.

        FANUC motion commands use SequenceIDs to monitor
        queued and completed instructions.

        The robot may return:
            SequenceID
            NextSequenceID

        The interface uses these values to determine the
        next available sequence number and command completion.
        """

        if not isinstance(
            response,
            dict
        ):
            return

        #
        # Ignore failed commands
        #
        if response.get(
            "ErrorID",
            0
        ) != 0:

            return

        if "SequenceID" in response:

            self.sequence_id = response[
                "SequenceID"
            ]

            print(
                f"Sequence ID Updated: "
                f"{self.sequence_id}"
            )

        elif "NextSequenceID" in response:

            self.expected_sequence_id = response[
                "NextSequenceID"
            ]

            self.sequence_id = (
                self.expected_sequence_id - 1
            )

    def next_sequence_id(self):
        """
        Return the next available SequenceID for a motion command.

        Returns DEFAULT_SEQUENCE_ID when no prior SequenceID has
        been received from the robot.
        """

        if self.expected_sequence_id is not None:

            return self.expected_sequence_id

        if self.sequence_id is None:

            return DEFAULT_SEQUENCE_ID

        return self.sequence_id + 1

# =============================================================================
# Receiver Worker
# =============================================================================
class ReceiverWorker(QObject):
    """
    Background receiver thread.

    Continuously reads packets from the RMI socket and
    forwards them to the Qt UI through signals.

    This prevents socket reads from blocking the GUI.
    """

    packet_received = pyqtSignal(object)
    error = pyqtSignal(str)

    def __init__(
        self,
        rmi
    ):
        super().__init__()

        self.rmi = rmi

        self.running = True

    def run(self):
        """
        Main receive loop. Runs in a background QThread.

        Continuously reads inbound packets and emits them as Qt
        signals to avoid blocking the GUI thread. Stops when
        stop() is called or the connection is lost.
        """

        while (
            self.running
            and
            self.rmi.connected
        ):

            try:

                packet = self.rmi.receive()

                self.packet_received.emit(
                    packet
                )

            except Exception as e:

                self.error.emit(
                    str(e)
                )

                break

    def stop(self):
        """Signal the receive loop to exit on its next iteration."""

        self.running = False

# =============================================================================
# ROS2 Node
# =============================================================================
class RmiInterfaceNode(Node):
    """
    ROS2 node that owns the MainWindow instance.

    Provides the ROS2 infrastructure foundation for future
    publishers, subscribers, services, and actions.
    """

    def __init__(self):

        super().__init__("rmi_interface_node")

        self.window = MainWindow(self)

# =============================================================================
# Main Application Window
# =============================================================================
class MainWindow(QMainWindow):
    """
    Main FANUC RMI user interface.

    Responsibilities:
        - Dynamic command form generation
        - Packet construction
        - Robot state monitoring
        - Sequence tracking
        - User interaction
    """

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------
    def __init__(self, node):

        super().__init__()

        self.node = node

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        #
        # Dynamic form widget registry.
        #
        # Key:
        #     Packet field name
        #
        # Value:
        #     Widget metadata used for validation,
        #     packet generation, and UI updates.
        #
        self.parameter_inputs = {}

        self.last_sent_sequence_id = None
        self.next_outgoing_sequence_id = DEFAULT_SEQUENCE_ID
        self.status_retry_attempts = 0
        self.rmi_initialized = False

        self.sequence_id_widget = None
        self.speed_type_widget = None
        self.term_type_widget = None
        self.term_value_widget = None

        self.receiver_thread = None
        self.receiver_worker = None 

        self.rmi = RMIClient()

        #
        # SequenceIDs that have been sent to the robot
        # but have not yet been reported as complete.
        #
        self.pending_sequences = []

        self.update_sequence_label()
        self.update_queued_packets_display()
        self.update_connection_status_label()

        self.load_commands()

        #
        # Button Connections
        #
        self.ui.connectButton.clicked.connect(
            self.connect_robot
        )

        self.ui.disconnectButton.clicked.connect(
            self.disconnect_robot
        )

        self.ui.initializeButton.clicked.connect(
            self.initialize_robot
        )

        self.ui.resetButton.clicked.connect(
            self.reset_robot
        )

        self.ui.abortButton.clicked.connect(
            self.abort_robot
        )

        self.ui.getStatusButton.clicked.connect(
            self.get_status
        )

        self.ui.sendCommandButton.clicked.connect(
            self.send_command
        )

        self.ui.commandComboBox.currentTextChanged.connect(
            self.command_changed
        )

        self.command_changed(
            self.ui.commandComboBox.currentText()
        )

        self.ui.sendCommandButton.setEnabled(
            False
        )
            
        self.ui.clearOutputButton.clicked.connect(
            self.clear_output
        )

    # -------------------------------------------------------------------------
    # Logging / Output Helpers
    # -------------------------------------------------------------------------
    def log(self, text):

        self.ui.responsePlainTextEdit.appendPlainText(
            str(text)
        )

    def clear_output(self):

        self.ui.responsePlainTextEdit.clear()

    def log_section(self, title):

        self.log("")

        self.log(
            "=" * 80
        )

        self.log(
            title
        )

        self.log(
            "=" * 80
        )

    def update_connection_status_label(self):
        """Update the top connection status label from socket/init state."""

        if not self.rmi.connected:

            self.ui.statusLabel.setText(
                "Disconnected"
            )

            return

        if self.rmi_initialized:

            self.ui.statusLabel.setText(
                "Connected and Initialized"
            )

            return

        self.ui.statusLabel.setText(
            "Connected - Not Initialized"
        )

    def update_initialized_from_status(self, response):
        """Update initialized state from documented FRC_GetStatus fields."""

        if response.get("ErrorID", 0) != 0:

            return

        motion_status = response.get("RMIMotionStatus")

        if motion_status is None:

            return

        initialized = int(motion_status) == 1

        if initialized != self.rmi_initialized:

            self.rmi_initialized = initialized
            self.update_connection_status_label()
    
    # -------------------------------------------------------------------------
    # Sequence ID Helpers
    # -------------------------------------------------------------------------
    def update_sequence_label(self):
        """Refresh sequence status labels in the command panel."""

        def display(value):
            return "--" if value is None else str(value)

        self.ui.lastSentSequenceLabel.setText(
            f"Last Sent Seq: {display(self.last_sent_sequence_id)}"
        )

        self.ui.lastReceivedSequenceLabel.setText(
            "Last Received Seq: "
            f"{display(self.rmi.last_received_sequence_id)}"
        )

        self.ui.expectedSequenceLabel.setText(
            f"Next Seq: {display(self.next_outgoing_sequence_id)}"
        )

    def sync_outgoing_sequence_cursor(self):
        """
        Recompute the local outgoing SequenceID cursor.

        Priority:
            1) Highest queued SequenceID + 1
            2) Robot-reported expected SequenceID
            3) Last successful sent SequenceID + 1
            4) DEFAULT_SEQUENCE_ID
        """

        if self.pending_sequences:

            max_pending = max(
                entry["sequence_id"]
                for entry in self.pending_sequences
            )

            self.next_outgoing_sequence_id = max_pending + 1

            return

        if self.rmi.expected_sequence_id is not None:

            self.next_outgoing_sequence_id = (
                self.rmi.expected_sequence_id
            )

            return

        if self.last_sent_sequence_id is not None:

            self.next_outgoing_sequence_id = (
                self.last_sent_sequence_id + 1
            )

            return

        self.next_outgoing_sequence_id = DEFAULT_SEQUENCE_ID

    def update_queued_packets_display(self):
        """Refresh the queued packets panel from pending SequenceIDs."""

        if not self.pending_sequences:

            self.ui.queuedPacketsPlainTextEdit.setPlainText(
                "None"
            )

            return

        lines = [
            f"Seq {entry['sequence_id']} - {entry['command_name']}"
            for entry in self.pending_sequences
        ]

        self.ui.queuedPacketsPlainTextEdit.setPlainText(
            "\n".join(lines)
        )

    def sync_sequence_id(self):

        if self.sequence_id_widget is None:

            return

        self.sequence_id_widget.setText(
            str(
                self.next_outgoing_sequence_id
            )
        )
    
    # -------------------------------------------------------------------------
    # Widget Callbacks
    # -------------------------------------------------------------------------
    def term_type_changed(self, value):

        if self.term_value_widget is None:

            return

        if value == "FINE":

            self.term_value_widget.setEnabled(
                False
            )

            self.term_value_widget.setText(
                "0"
            )

        else:

            self.term_value_widget.setEnabled(
                True
            )

    def left_right_enabled_changed(
        self,
        checked,
        left_combo,
        flip_combo,
        up_combo,
        front_combo
    ):

        left_combo.setEnabled(
            checked
        )

        flip_combo.setEnabled(
            not checked
        )

        up_combo.setEnabled(
            not checked
        )

        front_combo.setEnabled(
            not checked
        )

    def position_representation_changed(
        self,
        value
    ):

        position_info = self.parameter_inputs.get(
            "Position"
        )

        joint_info = self.parameter_inputs.get(
            "JointAngle"
        )

        if not position_info or not joint_info:

            return

        show_position = (
            value == "Cartesian"
        )

        position_info[
            "container"
        ].setVisible(
            show_position
        )

        position_info[
            "label"
        ].setVisible(
            show_position
        )

        joint_info[
            "container"
        ].setVisible(
            not show_position
        )

        joint_info[
            "label"
        ].setVisible(
            not show_position
        )

        self.validate_command_inputs()

    # -------------------------------------------------------------------------
    # Widget Helpers
    # -------------------------------------------------------------------------
    def update_speed_type_options(
        self,
        command_name
    ):
        """
        Populate the SpeedType dropdown with options appropriate for the command.

        Joint motion commands use percentage-based speed types.
        Linear and circular commands use distance-based types.
        """

        if self.speed_type_widget is None:
            return

        try:

            self.speed_type_widget.clear()

        except RuntimeError:

            return

        if "Joint" in command_name:

            self.speed_type_widget.addItems([
                "Percent",
                "Time",
                "mSec"
            ])

        else:

            self.speed_type_widget.addItems([
                "mmSec",
                "InchMin",
                "Time",
                "mSec"
            ])

    def refresh_sequence_id_field(self):

        if self.sequence_id_widget is None:

            return

        self.sequence_id_widget.setText(
            str(
                self.next_outgoing_sequence_id
            )
        )

    def packet_name(self, packet):
        """Return a human-friendly command/instruction name for a packet."""

        if hasattr(packet, "Command"):

            return packet.Command

        if hasattr(packet, "Instruction"):

            return packet.Instruction

        if hasattr(packet, "Communication"):

            return packet.Communication

        return type(packet).__name__

    def remove_pending_sequence(self, sequence_id):
        """Remove a queued packet entry by SequenceID and return it."""

        for idx, entry in enumerate(self.pending_sequences):

            if entry["sequence_id"] == sequence_id:

                return self.pending_sequences.pop(idx)

        return None

    # -------------------------------------------------------------------------
    # Command Changed Helpers
    # -------------------------------------------------------------------------

    def build_sequence_widget(self):
        """
        Add a read-only SequenceID row to the parameter form.

        The value is managed automatically by the interface and
        reflects the SequenceID that will be used on send.
        """

        layout = self.ui.parameterFormLayout

        edit = QLineEdit()

        edit.setReadOnly(True)

        edit.setText(
            str(
                self.next_outgoing_sequence_id
            )
        )

        layout.addRow(
            "SequenceID",
            edit
        )

        self.sequence_id_widget = edit

        self.parameter_inputs[
            "SequenceID"
        ] = {
            "widget": edit,
            "optional": False,
            "checkbox": None
        }

    def build_term_type_widget(self):

        layout = self.ui.parameterFormLayout

        combo = QComboBox()

        combo.addItems([
            "FINE",
            "CNT",
            "CR"
        ])

        combo.currentTextChanged.connect(
            self.term_type_changed
        )

        self.term_type_widget = combo

        layout.addRow(
            "TermType",
            combo
        )

        self.parameter_inputs[
            "TermType"
        ] = {
            "widget": combo,
            "optional": False,
            "checkbox": None
        }
    
    def build_term_value_widget(self):

        layout = self.ui.parameterFormLayout

        edit = QLineEdit()

        edit.textChanged.connect(
            self.validate_command_inputs
        )

        layout.addRow(
            "TermValue",
            edit
        )

        self.term_value_widget = edit

        self.parameter_inputs[
            "TermValue"
        ] = {
            "widget": edit,
            "optional": False,
            "checkbox": None
        }

        if self.term_type_widget is not None:

            self.term_type_changed(
                self.term_type_widget.currentText()
            )

    def build_speed_type_widget(self):

        layout = self.ui.parameterFormLayout

        combo = QComboBox()

        combo.addItems([
            "mmSec",
            "InchMin",
            "Time",
            "mSec"
        ])

        layout.addRow(
            "SpeedType",
            combo
        )

        self.speed_type_widget = combo

        self.parameter_inputs[
            "SpeedType"
        ] = {
            "widget": combo,
            "optional": False,
            "checkbox": None
        }

    def build_frame_widget(self):

        layout = self.ui.parameterFormLayout

        frame_widget = QWidget()

        frame_layout = QHBoxLayout(
            frame_widget
        )

        frame_layout.setContentsMargins(
            0, 0, 0, 0
        )

        edits = {}

        for axis in (
            "X",
            "Y",
            "Z",
            "W",
            "P",
            "R"
        ):

            label = QLabel(axis)

            edit = QLineEdit("0")

            edit.textChanged.connect(
                self.validate_command_inputs
            )

            edit.setMaximumWidth(60)

            frame_layout.addWidget(
                label
            )

            frame_layout.addWidget(
                edit
            )

            edits[axis] = edit

        frame_label = QLabel(
            "Frame"
        )

        layout.addRow(
            frame_label,
            frame_widget
        )

        edits["container"] = frame_widget
        edits["label"] = frame_label

        self.parameter_inputs[
            "Frame"
        ] = edits

    def build_representation_widget(self):

        layout = self.ui.parameterFormLayout

        combo = QComboBox()

        combo.addItems([
            "Cartesian",
            "Joint"
        ])

        combo.setCurrentText(
            "Cartesian"
        )

        combo.currentTextChanged.connect(
            self.position_representation_changed
        )

        layout.addRow(
            "Representation",
            combo
        )

        self.parameter_inputs[
            "Representation"
        ] = {
            "widget": combo,
            "optional": False,
            "checkbox": None
        }

    def build_port_type_widget(self):

        layout = self.ui.parameterFormLayout

        combo = QComboBox()

        combo.addItems([
            "AO",
            "GO",
            "DO",
            "RO",
            "FLAG",
            "AI",
            "GI",
            "DI",
            "RI"
        ])

        combo.setCurrentText(
            "DO"
        )

        layout.addRow(
            "Port Type",
            combo
        )

        self.parameter_inputs[
            "PortType"
        ] = {
            "widget": combo,
            "optional": False,
            "checkbox": None
        }

    def build_variable_type_widget(self):

        layout = self.ui.parameterFormLayout

        combo = QComboBox()

        combo.addItems([
            "Integer",
            "Float"
        ])

        combo.setCurrentText(
            "Integer"
        )

        layout.addRow(
            "Type",
            combo
        )

        self.parameter_inputs[
            "VariableType"
        ] = {
            "widget": combo,
            "optional": False,
            "checkbox": None
        }

    def build_register_data_type_widget(self):

        layout = self.ui.parameterFormLayout

        combo = QComboBox()

        combo.addItems([
            "Integer",
            "Float"
        ])

        combo.setCurrentText(
            "Integer"
        )

        layout.addRow(
            "Data Type",
            combo
        )

        self.parameter_inputs[
            "DataType"
        ] = {
            "widget": combo,
            "optional": False,
            "checkbox": None
        }

    def build_configuration_widget(self, field_name):
        """
        Add a robot configuration row to the parameter form.

        Renders UFrame/UTool numbers, Flip/Up/Front/Left dropdowns,
        and Turn4/5/6 inputs in a single compact horizontal row.
        """

        layout = self.ui.parameterFormLayout

        config_widget = QWidget()

        config_layout = QHBoxLayout(
            config_widget
        )

        config_layout.setContentsMargins(
            0, 0, 0, 0
        )

        uf_edit = QLineEdit(
            str(DEFAULT_USER_FRAME)
        )

        ut_edit = QLineEdit(
            str(DEFAULT_TOOL_FRAME)
        )

        uf_edit.setMaximumWidth(40)
        ut_edit.setMaximumWidth(40)

        flip_combo = QComboBox()
        flip_combo.addItems(["N", "F"])

        up_combo = QComboBox()
        up_combo.addItems(["D", "U"])

        front_combo = QComboBox()
        front_combo.addItems(["B", "T"])

        flip_combo.setCurrentText("N")
        up_combo.setCurrentText("U")
        front_combo.setCurrentText("T")

        left_enable = QCheckBox(
            "Use L/R"
        )

        left_combo = QComboBox()

        left_combo.addItems([
            "R",
            "L"
        ])

        left_combo.setEnabled(False)

        left_enable.toggled.connect(
            lambda checked:
            self.left_right_enabled_changed(
                checked,
                left_combo,
                flip_combo,
                up_combo,
                front_combo
            )
        )

        turn4_edit = QLineEdit(
            str(DEFAULT_TURN_VALUE)
        )

        turn5_edit = QLineEdit(
            str(DEFAULT_TURN_VALUE)
        )

        turn6_edit = QLineEdit(
            str(DEFAULT_TURN_VALUE)
        )

        turn4_edit.setMaximumWidth(40)
        turn5_edit.setMaximumWidth(40)
        turn6_edit.setMaximumWidth(40)

        config_layout.addWidget(
            QLabel("UF")
        )
        config_layout.addWidget(
            uf_edit
        )

        config_layout.addWidget(
            QLabel("UT")
        )
        config_layout.addWidget(
            ut_edit
        )

        config_layout.addWidget(
            flip_combo
        )

        config_layout.addWidget(
            up_combo
        )

        config_layout.addWidget(
            front_combo
        )

        config_layout.addWidget(
            left_enable
        )

        config_layout.addWidget(
            left_combo
        )

        config_layout.addWidget(
            QLabel("T4")
        )
        config_layout.addWidget(
            turn4_edit
        )

        config_layout.addWidget(
            QLabel("T5")
        )
        config_layout.addWidget(
            turn5_edit
        )

        config_layout.addWidget(
            QLabel("T6")
        )
        config_layout.addWidget(
            turn6_edit
        )

        layout.addRow(
            field_name,
            config_widget
        )

        self.parameter_inputs[
            field_name
        ] = {
            "UF": uf_edit,
            "UT": ut_edit,
            "Flip": flip_combo,
            "Up": up_combo,
            "Front": front_combo,
            "LeftEnable": left_enable,
            "Left": left_combo,
            "Turn4": turn4_edit,
            "Turn5": turn5_edit,
            "Turn6": turn6_edit
        }

    def build_axis_widget(self,field_name,axes,ext_axes):
        """
        Build a generic axis-entry widget.

        Used by:
            - PositionData
            - JointAngleData

        Optional external axes are hidden until
        the Ext checkbox is enabled.
        """

        layout = self.ui.parameterFormLayout

        widget = QWidget()

        widget_layout = QHBoxLayout(
            widget
        )

        widget_layout.setContentsMargins(
            0, 0, 0, 0
        )

        edits = {}

        for axis in axes:

            label = QLabel(axis)

            edit = QLineEdit()

            edit.textChanged.connect(
                self.validate_command_inputs
            )

            edit.setMaximumWidth(60)

            widget_layout.addWidget(
                label
            )

            widget_layout.addWidget(
                edit
            )

            edits[axis] = edit

        ext_checkbox = QCheckBox(
            "Ext"
        )

        widget_layout.addWidget(
            ext_checkbox
        )

        for axis in ext_axes:

            label = QLabel(axis)

            edit = QLineEdit()

            edit.textChanged.connect(
                self.validate_command_inputs
            )

            edit.setMaximumWidth(60)

            label.hide()
            edit.hide()

            widget_layout.addWidget(
                label
            )

            widget_layout.addWidget(
                edit
            )

            edits[axis] = edit

            ext_checkbox.toggled.connect(
                label.setVisible
            )

            ext_checkbox.toggled.connect(
                edit.setVisible
            )

        field_label = QLabel(
            field_name
        )

        layout.addRow(
            field_label,
            widget
        )

        edits["extended_checkbox"] = (
            ext_checkbox
        )

        edits["label"] = field_label
        edits["container"] = widget

        self.parameter_inputs[
            field_name
        ] = edits

    def build_position_widget(
        self,
        field_name
    ):

        self.build_axis_widget(
            field_name,
            (
                "X",
                "Y",
                "Z",
                "W",
                "P",
                "R"
            ),
            (
                "Ext1",
                "Ext2",
                "Ext3"
            )
        )

    def build_joint_widget(self):

        self.build_axis_widget(
            "JointAngle",
            (
                "J1",
                "J2",
                "J3",
                "J4",
                "J5",
                "J6"
            ),
            (
                "J7",
                "J8",
                "J9"
            )
        )

    def build_standard_widget(
        self,
        field_name,
        packet_types
    ):

        layout = self.ui.parameterFormLayout

        edit = QLineEdit()

        if field_name == "VariableName":

            edit.setText("$")

        if field_name in (
            "Group",
            "GroupNumber"
        ):

            edit.setText(
                str(DEFAULT_GROUP_NUMBER)
            )

        edit.textChanged.connect(
            self.validate_command_inputs
        )

        field_type = packet_types.get(
            field_name
        )

        optional = is_optional_type(
            field_type
        )

        #
        # Group should always be enabled
        #
        if field_name in (
            "Group",
            "GroupNumber"
        ):

            optional = False

        if optional:

            checkbox = QCheckBox()

            row_widget = QWidget()

            row_layout = QHBoxLayout(
                row_widget
            )

            row_layout.setContentsMargins(
                0, 0, 0, 0
            )

            row_layout.addWidget(
                checkbox
            )

            row_layout.addWidget(
                edit
            )

            edit.setEnabled(False)

            checkbox.toggled.connect(
                edit.setEnabled
            )

            layout.addRow(
                field_name,
                row_widget
            )

            self.parameter_inputs[
                field_name
            ] = {
                "widget": edit,
                "optional": True,
                "checkbox": checkbox
            }

        else:

            layout.addRow(
                field_name,
                edit
            )

            self.parameter_inputs[
                field_name
            ] = {
                "widget": edit,
                "optional": False,
                "checkbox": None
            }

    # -------------------------------------------------------------------------
    # Form Helpers
    # -------------------------------------------------------------------------
    def clear_parameter_form(self):
        """
        Remove all rows from the parameter form and reset widget references.

        Called each time the user selects a new command so the previous
        form fields are discarded before rebuilding for the new command.
        """

        layout = self.ui.parameterFormLayout

        while layout.rowCount():

            layout.removeRow(0)

        self.parameter_inputs = {}

        #
        # Reset widget references
        #
        self.sequence_id_widget = None
        self.speed_type_widget = None
        self.term_type_widget = None
        self.term_value_widget = None

    def get_visible_fields(
        self,
        command_name
    ):
        """
        Return the user-editable field names for a given command.

        Excludes fixed identifier fields (Command, Instruction,
        Communication) that are set automatically and should not
        appear in the dynamic form.
        """

        try:

            visible_fields = packet_fields(
                command_name
            )

        except Exception:

            return []

        skip_fields = {
            "Command",
            "Instruction",
            "Communication"
        }

        return [

            field
            for field in visible_fields

            if field not in skip_fields
        ]
    
    # -------------------------------------------------------------------------
    # Validate Command Input Helpers
    # -------------------------------------------------------------------------

    def validate_axis_fields(
        self,
        info,
        axes
    ):

        for axis in axes:

            if not info[axis].text().strip():

                return False

        return True
    
    def validate_required_fields(
        self,
        info,
        fields
    ):

        for field in fields:

            if not info[field].text().strip():

                return False

        return True

    # -------------------------------------------------------------------------
    # Form Validation
    # -------------------------------------------------------------------------
    def validate_command_inputs(self):
        """
        Validate all visible command parameters.

        Validation rules depend on the currently selected
        position representation (Cartesian or Joint).

        The Send button is enabled only when all required
        fields contain values.
        """

        if not self.parameter_inputs:

            self.ui.sendCommandButton.setEnabled(
                True
            )

            return

        valid = True

        representation = None

        if "Representation" in self.parameter_inputs:

            representation = (
                self.parameter_inputs[
                    "Representation"
                ]["widget"].currentText()
            )

        #
        # Build packet fields from UI widgets
        #
        for name, info in self.parameter_inputs.items():

            if name in (
                "Configuration",
                "ViaConfiguration"
            ):

                if not self.validate_required_fields(
                    info,
                    (
                        "UF",
                        "UT",
                        "Turn4",
                        "Turn5",
                        "Turn6"
                    )
                ):

                    valid = False

                continue

            #
            # Position widget
            #
            if name in (
                "Position",
                "ViaPosition"
            ):

                #
                # Ignore Position when Joint mode
                #
                if representation != "Cartesian":

                    continue

                if not self.validate_axis_fields(
                    info,
                    (
                        "X",
                        "Y",
                        "Z",
                        "W",
                        "P",
                        "R"
                    )
                ):

                    valid = False

                continue

            #
            # Joint widget
            #
            if name == "JointAngle":

                #
                # Ignore Joint when Cartesian mode
                #
                if representation != "Joint":

                    continue

                if not self.validate_axis_fields(
                    info,
                    (
                        "J1",
                        "J2",
                        "J3",
                        "J4",
                        "J5",
                        "J6"
                    )
                ):

                    valid = False

                continue

            if name == "Frame":

                if not self.validate_axis_fields(
                    info,
                    (
                        "X",
                        "Y",
                        "Z",
                        "W",
                        "P",
                        "R"
                    )
                ):

                    valid = False

                continue

            #
            # Optional fields don't matter
            #
            if info.get("optional", False):

                continue

            widget = info["widget"]

            if isinstance(
                widget,
                QComboBox
            ):

                if not widget.currentText().strip():

                    valid = False

            else:

                if not widget.text().strip():

                    valid = False

        self.ui.sendCommandButton.setEnabled(
            valid
        )

    # -------------------------------------------------------------------------
    # Command Loading
    # -------------------------------------------------------------------------
    def load_commands(self):

        self.ui.commandComboBox.addItems(
            sorted(PACKET_REGISTRY.keys())
        )

    # -------------------------------------------------------------------------
    # Dynamic Command Form Builder
    #
    # Creates packet-specific input widgets based on the
    # selected FANUC packet definition.
    #
    # Special widget support:
    #   - SequenceID
    #   - SpeedType
    #   - TermType
    #   - ConfigurationData
    #   - PositionData
    #   - JointAngleData
    # -------------------------------------------------------------------------
    def command_changed(self, command_name):
        """
        Rebuild the parameter form for the selected command.

        Packet definitions are obtained dynamically from
        the packet registry and appropriate widgets are
        generated for each field type.
        """

        self.clear_parameter_form()

        layout = self.ui.parameterFormLayout

        visible_fields = self.get_visible_fields(
            command_name
        )

        if not visible_fields:

            layout.addRow(
                QLabel("No Parameters"),
                QLabel("")
            )

            self.ui.sendCommandButton.setEnabled(
                True
            )

            return

        packet = create_packet(
            command_name
        )

        packet_types = {
            f.name: f.type
            for f in fields(type(packet))
        }

        for field_name in visible_fields:

            if field_name == "SequenceID":

                self.build_sequence_widget()

                continue

            if field_name == "TermType":

                self.build_term_type_widget()

                continue

            if field_name == "TermValue":

                self.build_term_value_widget()

                continue

            if field_name == "SpeedType":

                self.build_speed_type_widget()

                continue

            if field_name in (
                "Configuration",
                "ViaConfiguration"
            ):

                self.build_configuration_widget(
                    field_name
                )

                continue

            if field_name in (
                "Position",
                "ViaPosition"
            ):

                self.build_position_widget(
                    field_name
                )

                continue

            if field_name == "JointAngle":

                self.build_joint_widget()

                continue

            if field_name == "Representation":

                self.build_representation_widget()

                continue

            if (
                field_name == "PortType"
                and
                command_name in (
                    "FRC_ReadIOPort",
                    "FRC_WriteIOPort"
                )
            ):

                self.build_port_type_widget()

                continue

            if (
                field_name == "VariableType"
                and
                command_name in (
                    "FRC_ReadVariable",
                    "FRC_WriteVariable"
                )
            ):

                self.build_variable_type_widget()

                continue

            if (
                field_name == "DataType"
                and
                command_name == "FRC_WriteRegister"
            ):

                self.build_register_data_type_widget()

                continue

            if field_name == "Frame":

                self.build_frame_widget()

                continue

            self.build_standard_widget(
                field_name,
                packet_types
            )
            
        self.update_speed_type_options(
            command_name
        )

        if "Representation" in self.parameter_inputs:

            self.position_representation_changed(
                self.parameter_inputs[
                    "Representation"
                ]["widget"].currentText()
            )

        self.validate_command_inputs()

    # -------------------------------------------------------------------------
    # UI Button Callbacks
    # -------------------------------------------------------------------------
    def connect_robot(self):

        ip = self.ui.robotIpLineEdit.text().strip()

        if not ip:

            ip = DEFAULT_ROBOT_IP

        self.log_section(
            f"CONNECT : {ip}"
        )

        self.on_connect(ip)

    def disconnect_robot(self):

        self.log(
            "DISCONNECT"
        )

        self.on_disconnect()

    def initialize_robot(self):

        self.log_section(
            "INITIALIZE"
        )

        self.on_initialize()

    def reset_robot(self):

        self.log_section(
            "RESET"
        )

        self.on_reset()

    def abort_robot(self):

        self.log_section(
            "ABORT"
        )

        self.on_abort()

    def get_status(self):

        self.log_section(
            "GET STATUS"
        )

        self.on_status()

    # -------------------------------------------------------------------------
    # Send Command Helpers
    # -------------------------------------------------------------------------

    def build_configuration_data(
        self,
        info
    ):

        return ConfigurationData(

            UFrameNumber=int(
                info["UF"].text()
            ),

            UToolNumber=int(
                info["UT"].text()
            ),

            Flip=1 if
            info["Flip"].currentText()
            == "F"
            else 0,

            Up=1 if
            info["Up"].currentText()
            == "U"
            else 0,

            Front=1 if
            info["Front"].currentText()
            == "T"
            else 0,

            Left=1 if (
                info["LeftEnable"].isChecked()
                and
                info["Left"].currentText() == "L"
            ) else 0,

            Turn4=int(
                info["Turn4"].text()
            ),

            Turn5=int(
                info["Turn5"].text()
            ),

            Turn6=int(
                info["Turn6"].text()
            )
        )
    
    def build_position_data(
        self,
        info
    ):

        return PositionData(

            X=float(info["X"].text() or 0),
            Y=float(info["Y"].text() or 0),
            Z=float(info["Z"].text() or 0),

            W=float(info["W"].text() or 0),
            P=float(info["P"].text() or 0),
            R=float(info["R"].text() or 0),

            Ext1=float(info["Ext1"].text() or 0),
            Ext2=float(info["Ext2"].text() or 0),
            Ext3=float(info["Ext3"].text() or 0)
        )
    
    def build_joint_data(
        self,
        info
    ):

        return JointAngleData(

            J1=float(info["J1"].text() or 0),
            J2=float(info["J2"].text() or 0),
            J3=float(info["J3"].text() or 0),

            J4=float(info["J4"].text() or 0),
            J5=float(info["J5"].text() or 0),
            J6=float(info["J6"].text() or 0),

            J7=float(info["J7"].text() or 0),
            J8=float(info["J8"].text() or 0),
            J9=float(info["J9"].text() or 0)
        )
    
    def build_frame_data(
        self,
        info
    ):

        return FrameData(

            X=float(info["X"].text() or 0),
            Y=float(info["Y"].text() or 0),
            Z=float(info["Z"].text() or 0),

            W=float(info["W"].text() or 0),
            P=float(info["P"].text() or 0),
            R=float(info["R"].text() or 0)
        )
    
    def convert_widget_value(
        self,
        text
    ):
        """
        Convert a widget string value to the most specific Python type.

        Conversion order: int -> float -> str.

        Examples:
            "10"    -> 10
            "10.5"  -> 10.5
            "AUTO"  -> "AUTO"
        """

        try:
            return int(text)

        except ValueError:

            try:
                return float(text)

            except ValueError:

                return text

    # -------------------------------------------------------------------------
    # Packet Construction / Transmission
    # -------------------------------------------------------------------------
    def send_command(self):

        if len(self.pending_sequences) >= 8:

            self.log(
                "Robot queue full (8 commands)"
            )

            return

        command = (
            self.ui.commandComboBox.currentText()
        )

        packet = create_packet(
            command
        )

        for name, info in self.parameter_inputs.items():

            if name == "SequenceID":

                #
                # SequenceID is managed automatically in start_command.
                #
                continue

            if name in (
                "Configuration",
                "ViaConfiguration"
            ):

                config = self.build_configuration_data(
                    info
                )
                setattr(
                    packet,
                    name,
                    config
                )

                continue

            if name in (
                "Position",
                "ViaPosition"
            ):

                #
                # Skip Position fields when Joint representation is active
                #
                rep_info = self.parameter_inputs.get(
                    "Representation"
                )

                if (
                    rep_info
                    and rep_info["widget"].currentText()
                    == "Joint"
                ):
                    setattr(packet, name, None)
                    continue

                setattr(
                    packet,
                    name,
                    self.build_position_data(info)
                )

                continue

            if name == "JointAngle":

                #
                # Skip JointAngle when Cartesian representation is active
                #
                rep_info = self.parameter_inputs.get(
                    "Representation"
                )

                if (
                    rep_info
                    and rep_info["widget"].currentText()
                    == "Cartesian"
                ):
                    packet.JointAngle = None
                    continue

                packet.JointAngle = self.build_joint_data(info)

                continue

            if name == "Frame":

                packet.Frame = (
                    self.build_frame_data(
                        info
                    )
                )

                continue

            widget = info["widget"]

            if info["optional"]:

                if not info["checkbox"].isChecked():

                    setattr(
                        packet,
                        name,
                        None
                    )

                    continue

            if isinstance(
                widget,
                QComboBox
            ):

                value = widget.currentText()

            else:

                value = self.convert_widget_value(
                    widget.text()
                )

            setattr(
                packet,
                name,
                value
            )

        self.log_section(
            f"COMMAND : {command}"
        )

        self.log(
            packet.to_json()
        )

        self.start_command(
            packet
        )

        self.refresh_sequence_id_field()

    def start_command(self, packet):
        """
        Send a packet to the robot.

        If the packet contains a SequenceID it is added
        to the pending sequence list so completion can
        be monitored asynchronously.
        """

        #
        # Motion packet — track SequenceID as pending
        #
        if hasattr(packet, "SequenceID"):

            try:

                sequence_id = self.next_outgoing_sequence_id

                self.next_outgoing_sequence_id += 1

                packet.SequenceID = sequence_id

                if self.sequence_id_widget is not None:

                    self.sequence_id_widget.setText(
                        str(sequence_id)
                    )

                self.rmi.send_packet(packet)

                self.last_sent_sequence_id = sequence_id

                self.pending_sequences.append(
                    {
                        "sequence_id": sequence_id,
                        "command_name": self.packet_name(packet)
                    }
                )

                self.log(
                    f"Queued Sequence {sequence_id}"
                )

            except Exception as e:

                self.log(str(e))

                self.sync_outgoing_sequence_cursor()

            self.update_sequence_label()
            self.update_queued_packets_display()

            return

        #
        # Non-motion command packet
        #
        try:

            self.rmi.send_packet(packet)

        except Exception as e:

            self.log(str(e))

    def handle_packet(self,packet):
        """
        Route incoming packets to the correct handler.

        Packet types:
            - Motion completion notifications
            - Status updates
            - Generic responses
        """

        if isinstance(packet, dict):

            if packet.get("ErrorID", 0) != 0:

                self.handle_error_packet(packet)

                return

        #
        # Sequence completion
        #
        if (
            isinstance(packet, dict)
            and
            "Instruction" in packet
            and
            "SequenceID" in packet
        ):

            self.handle_sequence_complete(
                packet
            )

            return

        #
        # Status packet
        #
        if (
            isinstance(packet, dict)
            and
            packet.get("Command")
            == "FRC_GetStatus"
        ):

            self.handle_status_packet(
                packet
            )

            return

        #
        # Initialize response
        #
        if (
            isinstance(packet, dict)
            and
            packet.get("Command")
            == "FRC_Initialize"
        ):

            self.handle_initialize_response(packet)

            return

        #
        # Generic response
        #
        self.log(
            str(packet)
        )

        self.sync_outgoing_sequence_cursor()
        self.refresh_sequence_id_field()
        self.update_sequence_label()

    def handle_error_packet(self, packet):
        """
        Handle command errors returned by the robot.

        If the error packet includes a SequenceID, that queued command
        is removed and not counted as successfully sent.
        """

        sequence_id = packet.get("SequenceID")
        error_id = packet.get("ErrorID")

        if sequence_id is not None:

            removed = self.remove_pending_sequence(sequence_id)

            if removed is not None:

                self.log(
                    f"{removed['command_name']} "
                    f"(Seq {sequence_id}) Failed: ErrorID={error_id}"
                )

        self.log(str(packet))

        if packet.get("Command") == "FRC_Initialize":

            self.rmi_initialized = False
            self.update_connection_status_label()

        if packet.get("Command") == "FRC_GetStatus":

            if self.status_retry_attempts < 2:

                self.status_retry_attempts += 1

                self.log(
                    "Status request failed, retrying..."
                )

                QTimer.singleShot(
                    250,
                    self.on_status
                )

        self.sync_outgoing_sequence_cursor()
        self.refresh_sequence_id_field()
        self.update_sequence_label()
        self.update_queued_packets_display()

    def handle_initialize_response(self, packet):
        """
        Handle successful initialize responses.

        After initialization completes, request a fresh status packet
        to synchronize Next/Received sequence UI state.
        """

        self.log(str(packet))

        # FRC_GetStatus provides the source-of-truth initialization state.
        # We do not force initialized here.

        # Allow the controller a brief moment before requesting status.
        QTimer.singleShot(
            250,
            self.on_status
        )

    def handle_sequence_complete(
        self,
        packet
    ):
        """
        Process a motion completion notification from the robot.

        Removes the completed SequenceID from the pending list
        and refreshes the sequence display in the UI.
        """

        sequence_id = packet[
            "SequenceID"
        ]

        instruction = packet.get(
            "Instruction",
            "Unknown"
        )

        self.log(
            f"{instruction} "
            f"(Seq {sequence_id}) Complete"
        )

        self.remove_pending_sequence(sequence_id)

        self.rmi.last_received_sequence_id = sequence_id

        if self.rmi.expected_sequence_id is None:

            self.rmi.expected_sequence_id = sequence_id + 1

        else:

            self.rmi.expected_sequence_id = max(
                self.rmi.expected_sequence_id,
                sequence_id + 1
            )

        self.sync_outgoing_sequence_cursor()

        self.refresh_sequence_id_field()

        self.update_sequence_label()
        self.update_queued_packets_display()

    def handle_status_packet(
        self,
        response
    ):
        """
        Process a FRC_GetStatus response from the robot.

        Uses NextSequenceID to bulk-complete any pending sequences
        that have already finished, then updates the UI display.
        """

        self.status_retry_attempts = 0

        self.update_initialized_from_status(
            response
        )

        next_sequence = response.get(
            "NextSequenceID"
        )

        if next_sequence is not None:

            completed = []

            for entry in self.pending_sequences:

                seq = entry["sequence_id"]

                if seq < next_sequence:

                    completed.append(entry)

            for entry in completed:

                seq = entry["sequence_id"]

                self.remove_pending_sequence(seq)

                self.rmi.last_received_sequence_id = seq

                self.log(
                    f"{entry['command_name']} "
                    f"(Seq {seq}) Complete"
                )

        self.sync_outgoing_sequence_cursor()

        self.log(
            str(response)
        )

        self.refresh_sequence_id_field()

        self.update_sequence_label()
        self.update_queued_packets_display()

    # -------------------------------------------------------------------------
    # FANUC RMI Operations
    # -------------------------------------------------------------------------
    def on_connect(self, ip):
        """
        Connect to the robot and start the background receiver thread.

        Creates a ReceiverWorker in a dedicated QThread to handle
        inbound packets without blocking the GUI event loop.
        """

        try:

            self.rmi.connect(ip)
            self.receiver_thread = QThread()

            self.receiver_worker = ReceiverWorker(
                self.rmi
            )

            self.receiver_worker.moveToThread(
                self.receiver_thread
            )

            self.receiver_thread.started.connect(
                self.receiver_worker.run
            )

            self.receiver_worker.packet_received.connect(
                self.handle_packet
            )

            self.receiver_worker.error.connect(
                self.handle_receiver_error
            )

            self.receiver_thread.start()

            self.log(
                f"Connected to {ip}"
            )

            self.rmi_initialized = False
            self.update_connection_status_label()

            self.sync_outgoing_sequence_cursor()
            self.refresh_sequence_id_field()

            self.on_status()

        except Exception as e:

            self.rmi_initialized = False
            self.update_connection_status_label()

            self.log(
                f"Connection Failed: {e}"
            )

    def handle_receiver_error(self, message):
        """
        Handle errors emitted by the background ReceiverWorker.

        Called when the receiver thread encounters a socket error
        or the connection is lost unexpectedly.
        """

        self.log(
            f"Receiver Error: {message}"
        )

        self.on_disconnect()

    def on_disconnect(self):
        """
        Disconnect from the robot and stop the receiver thread cleanly.
        """

        try:
            self.rmi.disconnect()

            if self.receiver_worker is not None:

                self.receiver_worker.stop()

            if self.receiver_thread is not None:

                self.receiver_thread.quit()

                self.receiver_thread.wait()

                self.receiver_thread = None
                self.receiver_worker = None

            self.pending_sequences.clear()

            self.last_sent_sequence_id = None
            self.rmi_initialized = False

            self.rmi.last_received_sequence_id = None
            self.rmi.expected_sequence_id = None
            self.rmi.sequence_id = None
            self.next_outgoing_sequence_id = DEFAULT_SEQUENCE_ID

            self.refresh_sequence_id_field()
            self.update_queued_packets_display()
            self.update_sequence_label()
            self.update_connection_status_label()

            self.log(
                "Disconnected"
            )

        except Exception as e:

            self.rmi_initialized = False
            self.update_connection_status_label()

            self.log(
                str(e)
            )

    def on_initialize(self):
        """
        Send FRC_Initialize and immediately request the robot status.

        The follow-up status request ensures the latest
        NextSequenceID is returned right after initialization.
        """

        try:

            packet = create_packet(
                "FRC_Initialize"
            )

            self.log(
                packet.to_json()
            )

            self.rmi.send_packet(
                packet
            )

        except Exception as e:

            self.log(
                str(e)
            )

    def on_reset(self):
        """
        Send a reset command to clear robot faults and resume standby.
        """

        try:

            packet = create_packet(
                "FRC_Reset"
            )

            self.rmi.send_packet(packet)

            self.update_sequence_label()

        except Exception as e:

            self.log(str(e))

    def on_abort(self):
        """
        Send an abort command to immediately stop all robot motion.
        """

        try:

            packet = create_packet(
                "FRC_Abort"
            )

            self.rmi.send_packet(packet)

            self.rmi_initialized = False
            self.update_connection_status_label()

            # Refresh from FRC_GetStatus so state is sourced from robot response.
            QTimer.singleShot(
                250,
                self.on_status
            )

            self.update_sequence_label()

        except Exception as e:

            self.log(str(e))

    def on_status(self):
        """
        Request the current robot status, including NextSequenceID.
        """

        try:

            packet = create_packet(
                "FRC_GetStatus"
            )

            self.rmi.send_packet(
                packet
            )

        except Exception as e:

            self.log(
                str(e)
            )

def main(args=None):

    rclpy.init(args=args)

    app = QApplication(sys.argv)

    node = RmiInterfaceNode()

    node.window.show()

    timer = QTimer()

    timer.timeout.connect(
        lambda: rclpy.spin_once(
            node,
            timeout_sec=0.0
        )
    )

    timer.start(10)

    app.exec()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()