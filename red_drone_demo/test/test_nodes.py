import unittest
from red_drone_demo.drone_brain_node import DroneBrainNode
from red_drone_demo.mavlink_iface_node import MavlinkIfaceNode
from red_drone_demo.red_finder_node import RedFinderNode

class TestDroneNodes(unittest.TestCase):

    def setUp(self):
        self.drone_brain = DroneBrainNode()
        self.mavlink_iface = MavlinkIfaceNode()
        self.red_finder = RedFinderNode()

    def test_drone_brain_initialization(self):
        self.assertIsNotNone(self.drone_brain)

    def test_mavlink_iface_initialization(self):
        self.assertIsNotNone(self.mavlink_iface)

    def test_red_finder_initialization(self):
        self.assertIsNotNone(self.red_finder)

    # Add more tests for specific functionalities of each node

if __name__ == '__main__':
    unittest.main()