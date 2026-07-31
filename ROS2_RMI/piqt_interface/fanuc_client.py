import socket
import json
import time
import threading


from piqt_interface.rmi_packets import (
    ConnectROS2Packet,
    InitializePacket,
    StatusRequestPacket,
    DisconnectPacket,
)


HANDSHAKE_PORT = 16001
HANDSHAKE_TIMEOUT = 5.0
COMMAND_TIMEOUT = 180.0


class FanucRMIClient:

    def __init__(self, ip):

        self.ip = ip

        self.sock = None
        self.connected = False

        #
        # Sequence tracking
        #
        self.next_sequence_id = 1
        self.pending_sequences = set()

        #
        # Thread management
        #
        self.running = False
        self.receiver_thread = None

        self.lock = threading.Lock()

        self._connect()
        self.rx_buffer = ""


    #
    # ---------------------------------------------------------
    # Connection
    # ---------------------------------------------------------
    #

    def _connect(self):

        #
        # Handshake connection
        #
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        self.sock.settimeout(
            HANDSHAKE_TIMEOUT
        )

        self.sock.connect(
            (
                self.ip,
                HANDSHAKE_PORT
            )
        )

        print(
            "Connected to handshake port"
        )


        #
        # Request RMI session
        #
        packet = ConnectROS2Packet()

        data = (
            packet.to_json()
            + "\r\n"
        )

        print("TX:")
        print(data)


        self.sock.sendall(
            data.encode("utf-8")
        )


        response = self.sock.recv(
            4096
        )


        response = json.loads(
            response.decode("utf-8")
        )


        print("RX:")
        print(response)


        if response["ErrorID"] != 0:

            raise RuntimeError(
                f"Connection failed {response['ErrorID']}"
            )


        session_port = response["PortNumber"]


        print(
            f"Assigned RMI port {session_port}"
        )


        #
        # Close handshake socket
        #
        self.sock.close()


        time.sleep(0.5)


        #
        # Open session socket
        #
        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )


        self.sock.settimeout(
            COMMAND_TIMEOUT
        )


        self.sock.connect(
            (
                self.ip,
                session_port
            )
        )


        self.connected = True


        #
        # Start asynchronous receiver
        #
        self.running = True


        self.receiver_thread = threading.Thread(
            target=self.receiver_loop,
            daemon=True
        )


        self.receiver_thread.start()


        print(
            "Connected to RMI session"
        )


    #
    # ---------------------------------------------------------
    # Sending
    # ---------------------------------------------------------
    #

    def send(self, packet):

        if not self.connected:

            raise RuntimeError(
                "Robot not connected"
            )


        data = (
            packet.to_json()
            + "\r\n"
        )


        print("TX:")
        print(data)


        self.sock.sendall(
            data.encode("utf-8")
        )



    def send_motion(self, packet):

        packet.SequenceID = self.get_next_sequence()

        with self.lock:
            self.pending_sequences.add(packet.SequenceID)

        self.send(packet)

        return packet.SequenceID



    #
    # ---------------------------------------------------------
    # Receiving
    # ---------------------------------------------------------
    #

    def receive_loop_packets(self):

        while "\r\n" in self.rx_buffer:

            message, self.rx_buffer = self.rx_buffer.split(
                "\r\n",
                1
            )

            if not message:
                continue

            try:
                yield json.loads(message)

            except json.JSONDecodeError:
                print(
                    "Bad JSON:",
                    message
                )



    def receiver_loop(self):

        while self.running and self.connected:

            try:

                data = self.sock.recv(4096)

                if not data:
                    break


                self.rx_buffer += data.decode(
                    "utf-8"
                )


                for packet in self.receive_loop_packets():


                    print("RX:")
                    print(packet)


                    if not isinstance(packet, dict):
                        continue



                    #
                    # Motion completion
                    #
                    if (
                        "Instruction" in packet
                        and
                        "SequenceID" in packet
                    ):

                        with self.lock:

                            seq = packet["SequenceID"]

                            if seq in self.pending_sequences:
                                self.pending_sequences.remove(seq)


                        print(
                            "Completed:",
                            packet["SequenceID"]
                        )



                    #
                    # Status response
                    #
                    elif "NextSequenceID" in packet:

                        with self.lock:

                            self.next_sequence_id = (
                                packet["NextSequenceID"]
                            )


                        print(
                            "Robot next sequence:",
                            self.next_sequence_id
                        )


            except Exception as e:

                print(
                    f"Receiver error: {e}"
                )

                break



    #
    # ---------------------------------------------------------
    # Robot commands
    # ---------------------------------------------------------
    #

    def initialize_robot(self):

        self.send(
            InitializePacket(
                GroupMask=1
            )
        )

        time.sleep(0.5)

        print(
            "FANUC RMI ready"
        )



    def get_status(self):

        self.send(
            StatusRequestPacket()
        )



    #
    # ---------------------------------------------------------
    # Sequence helpers
    # ---------------------------------------------------------
    #

    def get_next_sequence(self):

        with self.lock:

            sequence = self.next_sequence_id

            self.next_sequence_id += 1

            return sequence



    @property
    def outstanding_commands(self):

        with self.lock:
            return len(self.pending_sequences)



    #
    # ---------------------------------------------------------
    # Shutdown
    # ---------------------------------------------------------
    #

    def close(self):

        print(
            "Closing FANUC connection"
        )


        try:

            if self.connected:

                self.send(
                    DisconnectPacket()
                )


        except Exception as e:

            print(
                f"Disconnect error: {e}"
            )


        self.running = False


        if self.receiver_thread:

            self.receiver_thread.join(
                timeout=1.0
            )


        if self.sock:

            self.sock.close()


        self.connected = False


        print(
            "Disconnected"
        )