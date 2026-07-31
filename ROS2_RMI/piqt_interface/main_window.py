from PyQt5.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QPushButton,
    QComboBox,
    QFormLayout,
    QPlainTextEdit,
    QScrollArea
)


class Ui_MainWindow(object):

    def setupUi(self, MainWindow):

        MainWindow.setWindowTitle("RMI Interface")
        MainWindow.resize(900, 700)

        self.centralwidget = QWidget(MainWindow)
        MainWindow.setCentralWidget(self.centralwidget)

        main_layout = QVBoxLayout(self.centralwidget)

        #
        # CONNECTION SECTION
        #
        self.connectionGroupBox = QGroupBox("Connection")
        connection_layout = QVBoxLayout()

        #
        # First line
        #
        connect_row = QHBoxLayout()

        connect_row.addWidget(QLabel("IP Address"))

        self.robotIpLineEdit = QLineEdit()
        self.robotIpLineEdit.setPlaceholderText(
            "192.168.1.100"
        )

        connect_row.addWidget(self.robotIpLineEdit)

        self.connectButton = QPushButton("Connect")
        self.disconnectButton = QPushButton("Disconnect")

        connect_row.addWidget(self.connectButton)
        connect_row.addWidget(self.disconnectButton)

        connection_layout.addLayout(connect_row)

        #
        # Second line
        #
        control_row = QHBoxLayout()

        self.initializeButton = QPushButton("Initialize")
        self.resetButton = QPushButton("Reset")
        self.abortButton = QPushButton("Abort")

        control_row.addWidget(self.initializeButton)
        control_row.addWidget(self.resetButton)
        control_row.addWidget(self.abortButton)

        connection_layout.addLayout(control_row)

        #
        # Status
        #
        self.statusLabel = QLabel("Disconnected")
        connection_layout.addWidget(self.statusLabel)

        self.connectionGroupBox.setLayout(connection_layout)

        #
        # COMMAND SECTION
        #
        self.commandGroupBox = QGroupBox("Commands")

        command_layout = QVBoxLayout()

        command_row = QHBoxLayout()

        command_row.addWidget(QLabel("Command"))

        self.commandComboBox = QComboBox()

        command_row.addWidget(self.commandComboBox)

        command_layout.addLayout(command_row)

        #
        # Sequence tracking summary
        #
        sequence_status_row = QHBoxLayout()

        self.lastSentSequenceLabel = QLabel(
            "Last Sent Seq: --"
        )

        self.lastReceivedSequenceLabel = QLabel(
            "Last Received Seq: --"
        )

        self.expectedSequenceLabel = QLabel(
            "Next Seq: --"
        )

        sequence_status_row.addWidget(
            self.lastSentSequenceLabel
        )

        sequence_status_row.addWidget(
            self.lastReceivedSequenceLabel
        )

        sequence_status_row.addWidget(
            self.expectedSequenceLabel
        )

        command_layout.addLayout(
            sequence_status_row
        )

        #
        # Dynamic parameter area
        #
        self.parameterWidget = QWidget()

        self.parameterFormLayout = QFormLayout()

        self.parameterWidget.setLayout(
            self.parameterFormLayout
        )

        #
        # Scroll Area
        #
        self.parameterScrollArea = QScrollArea()

        self.parameterScrollArea.setWidgetResizable(
            True
        )

        self.parameterScrollArea.setWidget(
            self.parameterWidget
        )

        #
        # Limit height so it scrolls instead of growing
        #
        self.parameterScrollArea.setMaximumHeight(
            250
        )

        command_layout.addWidget(
            self.parameterScrollArea
        )

        self.sendCommandButton = QPushButton(
            "Send Command"
        )

        command_layout.addWidget(
            self.sendCommandButton
        )

        #
        # Queued packet display
        #
        self.queuedPacketsGroupBox = QGroupBox(
            "Queued Packets"
        )

        queued_packets_layout = QVBoxLayout()

        self.queuedPacketsPlainTextEdit = (
            QPlainTextEdit()
        )

        self.queuedPacketsPlainTextEdit.setReadOnly(
            True
        )

        self.queuedPacketsPlainTextEdit.setMaximumHeight(
            90
        )

        queued_packets_layout.addWidget(
            self.queuedPacketsPlainTextEdit
        )

        self.queuedPacketsGroupBox.setLayout(
            queued_packets_layout
        )

        command_layout.addWidget(
            self.queuedPacketsGroupBox
        )

        self.commandGroupBox.setLayout(
            command_layout
        )

        #
        # RESPONSE SECTION
        #
        self.responseGroupBox = QGroupBox(
            "Responses"
        )

        response_layout = QVBoxLayout()

        self.responsePlainTextEdit = (
            QPlainTextEdit()
        )

        self.responsePlainTextEdit.setReadOnly(
            True
        )

        response_layout.addWidget(
            self.responsePlainTextEdit
        )

        #
        # Response Buttons
        #
        response_button_row = QHBoxLayout()

        self.getStatusButton = QPushButton(
            "Get Status"
        )

        self.clearOutputButton = QPushButton(
            "Clear Output"
        )

        response_button_row.addWidget(
            self.getStatusButton
        )

        response_button_row.addWidget(
            self.clearOutputButton
        )

        response_layout.addLayout(
            response_button_row
        )

        self.responseGroupBox.setLayout(
            response_layout
        )

        #
        # Add sections
        #
        main_layout.addWidget(
            self.connectionGroupBox
        )

        main_layout.addWidget(
            self.commandGroupBox
        )

        main_layout.addWidget(
            self.responseGroupBox
        )