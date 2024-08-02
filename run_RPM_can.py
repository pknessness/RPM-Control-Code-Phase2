import can
# import can.interface


# Candlelight firmware on Linux
#bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)

# Stock slcan firmware on Linux
bus = can.interface.Bus(bustype='slcan', channel='/dev/ttyACM0', bitrate=500000)

# Stock slcan firmware on Windows
bus = can.interface.Bus(bustype='slcan', channel='COM0', bitrate=500000)

msg = can.Message(arbitration_id=0x142,
                  data=[0xA2, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00],
                  is_extended_id=True)

try:
    bus.send(msg)
    print("Message sent on {}".format(bus.channel_info))
except can.CanError:
    print("Message NOT sent")