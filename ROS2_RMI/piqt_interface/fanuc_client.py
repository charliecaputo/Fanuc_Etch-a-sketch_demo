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
        self.packet_callbacks = []

        # Receive buffer
        self.rx_buffer = ""
        # Sequence tracking
        self.next_sequence_id = None
        self.sequence_ready = threading.Event()
        # Status tracking
        self.status_event = threading.Event()
        self.last_status = None
        # Initialization tracking
        self.initialized = False
        # Disconnect tracking
        self.disconnect_event = threading.Event()
        # Commands
        self.pending_sequences = set()
        # Thread management
        self.running = False
        self.receiver_thread = None
        
        self.lock = threading.Lock()
        self._connect()
        


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

        print(
            f"Connecting to RMI session port {session_port}"
        )
        self.sock.connect(
            (
                self.ip,
                session_port
            )
        )
        print(
            "RMI session socket connected"
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

    def add_packet_callback(
        self,
        callback
    ):

        self.packet_callbacks.append(
            callback
        )

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

                    for callback in self.packet_callbacks:
                        try:

                            callback(packet)

                        except Exception as e:

                            print(
                                f"Packet callback error: {e}"
                            )


                    if not isinstance(packet, dict):
                        continue
                    #
                    # Motion completion
                    #
                    if "Instruction" in packet and "SequenceID" in packet:

                        seq = packet["SequenceID"]
                        err = packet.get("ErrorID", 0)

                        with self.lock:
                            self.pending_sequences.discard(seq)

                        if err == 0:
                            print(f"Motion {seq} completed")
                        else:
                            print(f"Motion {seq} failed (ErrorID={err})")



                    #
                    # Status response
                    #
                    elif "NextSequenceID" in packet:

                        with self.lock:
                            self.last_status = packet
                            self.next_sequence_id = packet["NextSequenceID"]

                        self.status_event.set()
                        self.sequence_ready.set()

                        print(
                            "Robot next sequence:",
                            self.next_sequence_id
                        )

                        print(
                            "ServoReady:",
                            packet.get("ServoReady")
                        )
                    
                    elif (packet.get("Communication") == "FRC_Disconnect"):
                            print("Disconnect acknowledged")
                            self.disconnect_event.set()


            except OSError:
                break

            except Exception as e:
                print(f"Receiver error: {e}")
                break

    def send_json(
        self,
        command
    ):

        if "Instruction" in command:

            while self.outstanding_commands >= 2:
                time.sleep(0.01)

            command["SequenceID"] = self.get_next_sequence()

            with self.lock:
                self.pending_sequences.add(
                    command["SequenceID"]
                )

        data = (
            json.dumps(command)
            + "\r\n"
        )

        print("TX:")
        print(data)

        with self.lock:
            self.sock.sendall(
                data.encode("utf-8")
            )

    def get_status(self):

        self.status_event.clear()

        self.send(
            StatusRequestPacket()
        )

        if not self.status_event.wait(timeout=5.0):
            raise RuntimeError(
                "No status response from robot"
            )

    #
    # ---------------------------------------------------------
    # Robot commands
    # ---------------------------------------------------------
    #

    def initialize_robot(self):
        #
        # Get current robot state
        #
        self.get_status()

        if self.last_status is None:
            raise RuntimeError(
                "No status information available"
            )

        rmi_motion_status = self.last_status.get(
            "RMIMotionStatus"
        )

        servo_ready = self.last_status.get(
            "ServoReady"
        )

        print(
            "ServoReady:",
            servo_ready
        )

        print(
            "RMIMotionStatus:",
            rmi_motion_status
        )


        #
        # RMI already initialized
        #
        if rmi_motion_status == 1:

            print(
                "RMI already initialized, skipping initialization"
            )

            self.initialized = True
            return


        #
        # RMI needs initialization
        #
        if rmi_motion_status == 0:

            print(
                "RMI motion inactive, initializing"
            )

            self.send(
                InitializePacket(
                    GroupMask=1
                )
            )

            #
            # Allow controller to update state
            #
            time.sleep(0.5)

            #
            # Verify initialization succeeded
            #
            self.get_status()

            if self.last_status.get("RMIMotionStatus") != 1:

                raise RuntimeError(
                    "RMI initialization failed"
                )

            self.initialized = True

            print(
                "RMI initialization successful"
            )

            return


        #
        # Unexpected response
        #
        raise RuntimeError(
            f"Unexpected RMIMotionStatus: {rmi_motion_status}"
        )

    #
    # ---------------------------------------------------------
    # Sequence helpers
    # ---------------------------------------------------------
    #
    def get_next_sequence(self):
        if not self.sequence_ready.wait(timeout=5.0):
            raise RuntimeError(
                "Sequence ID not initialized"
            )

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

    def close_socket_only(self):

        self.running = False

        if self.sock:

            try:
                self.sock.shutdown(socket.SHUT_RDWR)

            except OSError:
                pass

            self.sock.close()

        self.connected = False

    def close(self):

        print("Closing FANUC connection")

        try:

            if self.connected:

                self.disconnect_event.clear()

                self.send(
                    DisconnectPacket()
                )

                #
                # Wait for controller acknowledgement
                #
                if not self.disconnect_event.wait(timeout=5.0):
                    print("Timed out waiting for disconnect acknowledgement")

        except Exception as e:

            print(
                f"Disconnect error: {e}"
            )

        #
        # Stop receiver
        #
        self.running = False

        #
        # Closing the socket unblocks recv()
        #
        if self.sock:
            try:
                self.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

            self.sock.close()

        if self.receiver_thread:
            self.receiver_thread.join(timeout=2.0)

        self.connected = False

        print("Disconnected")