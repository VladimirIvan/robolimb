from robolimb import RoboLimbCAN as RoboLimb


from can.interfaces.pcan.basic import (PCANBasic, PCAN_USBBUS1, PCAN_BAUD_1M,
                                       PCAN_TYPE_ISA, PCAN_ERROR_QRCVEMPTY,
                                       PCAN_ERROR_OK, TPCANMsg,
                                       PCAN_MESSAGE_STANDARD)

class CommsBus:
    def open(self):
        raise RuntimeError('Not implemented')
    def close(self):
        raise RuntimeError('Not implemented')
    def write(self, id, data):
        raise RuntimeError('Not implemented')
    def read(self):
        raise RuntimeError('Not implemented')
    def reset(self):
        raise RuntimeError('Not implemented')

class RoboLimbFingerExample(object):
    def __init__(self):
        bus = PeakCommsBus()
        # bus = SerialCommsBus()
        self.r = RoboLimb(bus)

    def start(self):
        self.r.start()
        self.r.open_all()
        time.sleep(1.5)

class PeakCommsBus(CommsBus):

    def __init__(self,
                 channel=PCAN_USBBUS1,
                 b_rate=PCAN_BAUD_1M,
                 hw_type=PCAN_TYPE_ISA,
                 io_port=0x3BC,
                 interrupt=3):
        self.channel = channel
        self.b_rate = b_rate
        self.hw_type = hw_type
        self.io_port = io_port
        self.interrupt = interrupt

        self.__finger_status = [None] * N_DOF
        self.__finger_current = [None] * N_DOF
        self.__rotator_edge = None

    def open(self):
        """Starts the CAN BUS connection."""
        self.bus = PCANBasic()
        self.bus.Initialize(
            Channel=self.channel,
            Btr0Btr1=self.b_rate,
            HwType=self.hw_type,
            IOPort=self.io_port,
            Interrupt=self.interrupt)

    def close(self):
        """Stops reading incoming CAN messages and shuts down the
        connection."""
        self.bus.Uninitialize(Channel=self.channel)

    def write(self, id, data):
        self.bus.Write(self.channel, self.__can_message(id, data))

    def read(self):
        msg = self.bus.Read(self.channel)
        print(msg)
        return msg

    def reset(self):
        self.bus.Reset(self.channel)

    def __can_message(self, id, data):
        """Creates a CAN message from corresponding CAN ID and data.

        Parameters
        ----------
        message_id : int
            CAN message ID.

        message_data : list of str
            CAN message data. A list of strings of length 4 is expected.

        Returns
        -------
        can_msg : pcan definition
            CAN message.
        """
        can_msg = TPCANMsg()
        can_msg.ID = id
        can_msg.LEN = 4
        can_msg.MSGTYPE = PCAN_MESSAGE_STANDARD
        for i in range(can_msg.LEN):
            can_msg.DATA[i] = int(data[i], 16)

        print(hex(can_msg.ID))
        for i in range(can_msg.LEN):
            print(can_msg.DATA[i])
        return can_msg
    
    def __del__(self):
        """Stops CAN bus connection upon destruction."""
        self.close()



    def __read_messages(self, num_messages=None):
        """Reads either a specified number of messages or all available
        messages from the queue.

        Parameters
        ----------
        num_messages : int, optional (default: None)
            Number of messages to read. If ``None``, read all messages unti
            queue is empty.

        Returns
        -------
        messages : list
            List of incoming CAN messages.

        Notes
        -----
        When the number of messages is specified, the method will block
        execution until the messages become available, i.e., there is no
        timeout implementation. If, for any reason, CAN messages do not arrive
        in the queue, the program may not exit the loop.
        """
        messages = []
        if num_messages:
            while len(messages) < num_messages:
                can_msg = self.bus.read()
                if can_msg[0] == PCAN_ERROR_OK:
                    messages.append(can_msg)
        else:
            res = 0
            while res != PCAN_ERROR_QRCVEMPTY:
                can_msg = self.bus.read()
                res = can_msg[0]
                if res == PCAN_ERROR_OK:
                    messages.append(can_msg)
        print('Read:', messages)
        return messages