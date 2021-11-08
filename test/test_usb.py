import pytest

from ant.easy.node import Node

from ant.devices.common import DeviceType
from ant.devices.scanner import Scanner
from ant.devices.power_meter import PowerMeter
from ant.devices.fitness_equipment import FitnessEquipment

# standard ANT+ network key
NETWORK_KEY = [0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45]

@pytest.fixture()
def node():
    node = Node()
    node.set_network_key(0x00, NETWORK_KEY)

    yield node

    node.stop()

@pytest.fixture()
def power_meter(node):
    yield PowerMeter(node)

@pytest.fixture()
def fec(node):
    yield FitnessEquipment(node)

def on_found_closure(device_id, device_type, device_trans, callback=None):

    def on_found(device):
        did, dtype, dtrans = device

        print(f"{did}:{dtype}:{dtrans}")
        assert did == device_id
        assert dtype == device_type
        assert dtrans == device_trans

        if callback:
            callback('found')

    return on_found

def on_update_closure(serial_no, software_ver, callback=None):

    def on_update(_, common):
        print(f"{common}")
        if common.serial_no != 0xFFFF:
            assert common.serial_no == serial_no
            assert common.software_ver == software_ver

            if callback:
                callback('update')

    return on_update

class TestScan():
    SCAN_FOUND_PM = False
    SCAN_FOUND_FEC = False

    @pytest.mark.timeout(10)
    def test_scan_power_meter(self, node):
        def stop(e):
            if e == 'update':
                scanner.close_channel()
                node.stop()

        scanner = Scanner(node, device_type=DeviceType.PowerMeter.value)

        scanner.on_found = on_found_closure(1337, DeviceType.PowerMeter.value, 5, stop)
        scanner.on_update = on_update_closure(1337, '0.1', stop)

        try:
            node.start()
        except KeyboardInterrupt:
            print(f"Closing ANT+ node...")
        finally:
            if scanner.channel._ant._running:
                scanner.close_channel()
            node.stop()

    @pytest.mark.timeout(10)
    def test_scan_fec(self, node):
        def stop(e):
            if e == 'update':
                scanner.close_channel()
                node.stop()

        scanner = Scanner(node, device_type=DeviceType.FitnessEquipment.value)

        scanner.on_found = on_found_closure(1337, DeviceType.FitnessEquipment.value, 5, stop)
        scanner.on_update = on_update_closure(1337, '0.1', stop)

        try:
            node.start()
        except KeyboardInterrupt:
            print(f"Closing ANT+ node...")
        finally:
            if scanner.channel._ant._running:
                scanner.close_channel()
            node.stop()

    @pytest.mark.timeout(10)
    def test_scan_all(self, node):
        def found(device):
            global SCAN_FOUND_PM, SCAN_FOUND_FEC

            if device[1] == 11:
                self.SCAN_FOUND_PM = True
            elif device[1] == 17:
                self.SCAN_FOUND_FEC = True

            if self.SCAN_FOUND_PM and self.SCAN_FOUND_FEC:
                scanner.close_channel()
                node.stop()

        def update(*args):
            pass

        scanner = Scanner(node)

        scanner.on_found = found
        scanner.on_update = update

        try:
            node.start()
        except KeyboardInterrupt:
            print(f"Closing ANT+ node...")
        finally:
            if scanner.channel._ant._running:
                scanner.close_channel()
            node.stop()
