import usb.core
import usb.util
import math
from dataclasses import dataclass

TOTAL_MASS = 5.6 # kg

@dataclass
class Cylinder():
    radius: float
    height: float

    @property
    def volume(self):
        return math.pi * self.radius ** 2 * self.height

def calculate_kick_moment_inertia(flywheel_r, flywheel_h, inner_h, inner_r, wall_thickness, axle_r):
    # bottom disc
    disc = Cylinder(radius=flywheel_r, height=flywheel_h - inner_h)
    # axle void in disc
    axle_void_disc = Cylinder(radius=axle_r, height=disc.height)

    # thin lump
    flywheel = Cylinder(radius=flywheel_r, height=flywheel_h - disc.height)
    # void
    void = Cylinder(radius=flywheel.radius - wall_thickness, height=flywheel.height)
    # inner cylinder lump
    inner = Cylinder(radius=inner_r, height=flywheel.height)
    # axle void (there is a keyway but hey...)
    axle_void = Cylinder(radius=axle_r, height=flywheel.height)

    # volume of thin wall
    thin_wall_v = flywheel.volume - void.volume
    # volume of inner cylinder
    inner_v = inner.volume - axle_void.volume
    # volume of bottom disc
    disc_v = disc.volume - axle_void_disc.volume

    # actual volume of cast flywheel
    flywheel_v = thin_wall_v + inner_v + disc_v
    # calculate mass of each part using proportion of volume
    thin_wall_mass = (thin_wall_v / flywheel_v) * TOTAL_MASS
    inner_mass = (inner_v / flywheel_v) * TOTAL_MASS
    disc_mass = (disc_v / flywheel_v) * TOTAL_MASS

    # now inertia of each
    disc_i = 0.5 * disc_mass * (disc.radius ** 2 * axle_void_disc.radius ** 2) # hollow cylinder 1/2 M (r1^2 * r2^2)
    thin_wall_i = thin_wall_mass * void.radius ** 2 # thin wall M r1^2
    inner_i = 0.5 * inner_mass * (inner.radius ** 2 * axle_void.radius ** 2)

    return disc_i + thin_wall_i + inner_i

print(f"Kickr 2017 moment inertia {calculate_kick_moment_inertia(flywheel_r=100e-3, flywheel_h=70.5e-3, inner_h=64.5e-3, inner_r=30e-3, wall_thickness=14.8e-3, axle_r=7e-3):.6} kg/m2")

# This was initial testing
def test_usb():
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
