import usb.core
import usb.util

# find our device
dev = usb.core.find(idVendor=0x0fcf, idProduct=0x1009)
# set the active configuration. With no arguments, the first
# configuration will be the active one
dev.set_configuration()
# get an endpoint instance
cfg = dev.get_active_configuration()
intf = cfg[(0, 0)]
ep_out = usb.util.find_descriptor(
    intf,
    # match the first OUT endpoint
    custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
    == usb.util.ENDPOINT_OUT,
)
ep_in = usb.util.find_descriptor(
    intf,
    # match the first IN endpoint
    custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress)
    == usb.util.ENDPOINT_IN,
)
assert ep_out is not None
assert ep_in is not None

MSGS = [
    [0xa4, 0x01, 0x4a, 0x00, 0xef], # reset
    [0xa4, 0x02, 0x4d, 0x00, 0x54, 0xbf], # caps,
    [0xa4, 0x02, 0x4d, 0x00, 0x61, 0x8a, 0x00, 0x00], # serial
]

for m in MSGS:
    ep_out.write(bytes(m))
    raw_rx = ep_in.read(64, 100)
    print("RX: " + str(raw_rx))

del dev
