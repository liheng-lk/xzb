import unittest

from xzb_roarm_driver.ros_node import RoArmRosNode


class Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ''


class JointState:
    def __init__(self):
        self.header = Header()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []


class Position:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class Orientation:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0


class Pose:
    def __init__(self):
        self.position = Position()
        self.orientation = Orientation()


class Bool:
    def __init__(self, data=False):
        self.data = data


class String:
    def __init__(self, data=''):
        self.data = data


class FakePublisher:
    def __init__(self, topic, msg_type, queue_size=10, latch=False):
        self.topic = topic
        self.msg_type = msg_type
        self.queue_size = queue_size
        self.latch = latch
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeTimer:
    def __init__(self, duration, callback):
        self.duration = duration
        self.callback = callback


class FakeTime:
    @staticmethod
    def now():
        return 'STAMP'


class FakeRospy:
    Time = FakeTime

    def __init__(self):
        self.params = {
            '~port': '/dev/ttyUSB0',
            '~baud_rate': 115200,
            '~query_rate': 2.0,
            '~reconnect_interval': 1.0,
            '~serial_timeout': 0.2,
            '~joint_frame_id': 'roarm_base_link',
        }
        self.publishers = {}
        self.timer = None
        self.shutdown_cb = None
        self.logs = []

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def Publisher(self, topic, msg_type, queue_size=10, latch=False):
        pub = FakePublisher(topic, msg_type, queue_size, latch)
        self.publishers[topic] = pub
        return pub

    def Duration(self, value):
        return value

    def Timer(self, duration, callback):
        self.timer = FakeTimer(duration, callback)
        return self.timer

    def on_shutdown(self, cb):
        self.shutdown_cb = cb

    def loginfo(self, *args):
        self.logs.append(('info', args))

    def logwarn(self, *args):
        self.logs.append(('warn', args))

    def logwarn_throttle(self, *args):
        self.logs.append(('warn_throttle', args))

    def logdebug(self, *args):
        self.logs.append(('debug', args))


class FakeTransport:
    instances = []

    def __init__(self, port, baud_rate=115200, timeout=0.2, reconnect_interval=1.0, **kwargs):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.reconnect_interval = reconnect_interval
        self.connected = True
        self.on_line = None
        self.on_connection = None
        self.started = False
        self.stopped = False
        self.writes = []
        self.__class__.instances.append(self)

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True
        self.connected = False

    def write(self, payload):
        self.writes.append(payload)


MESSAGE_TYPES = {
    'Bool': Bool,
    'String': String,
    'JointState': JointState,
    'Pose': Pose,
}


class RosNodeTests(unittest.TestCase):
    def setUp(self):
        FakeTransport.instances = []
        self.ros = FakeRospy()
        self.node = RoArmRosNode(
            rospy_module=self.ros,
            message_types=MESSAGE_TYPES,
            transport_factory=FakeTransport,
        )
        self.transport = FakeTransport.instances[-1]

    def test_initializes_expected_topics_and_transport(self):
        self.assertEqual(set(self.ros.publishers), {
            '/roarm/connected', '/roarm/joint_states', '/roarm/pose', '/roarm/raw_response'
        })
        self.assertTrue(self.ros.publishers['/roarm/connected'].latch)
        self.assertEqual(self.transport.port, '/dev/ttyUSB0')
        self.assertEqual(self.transport.baud_rate, 115200)
        self.assertTrue(self.transport.started)
        self.assertAlmostEqual(self.ros.timer.duration, 0.5)

    def test_state_line_publishes_raw_joint_state_and_pose(self):
        line = '{"T":1051,"x":515.88,"y":-3.95,"z":-49.68,"b":-0.01,"s":1.59,"e":0.03,"t":1.69,"torB":36,"torS":104,"torE":92,"torH":300}'
        self.transport.on_line(line)

        raw = self.ros.publishers['/roarm/raw_response'].messages[-1]
        joint = self.ros.publishers['/roarm/joint_states'].messages[-1]
        pose = self.ros.publishers['/roarm/pose'].messages[-1]

        self.assertEqual(raw.data, line)
        self.assertEqual(joint.header.stamp, 'STAMP')
        self.assertEqual(joint.header.frame_id, 'roarm_base_link')
        self.assertEqual(joint.name, ['base', 'shoulder', 'elbow', 'hand'])
        self.assertEqual(joint.position, [-0.01, 1.59, 0.03, 1.69])
        self.assertEqual(joint.effort, [])
        self.assertAlmostEqual(pose.position.x, 0.51588)
        self.assertAlmostEqual(pose.position.y, -0.00395)
        self.assertAlmostEqual(pose.position.z, -0.04968)
        self.assertEqual(pose.orientation.w, 1.0)

    def test_connection_callback_publishes_latched_bool(self):
        self.transport.on_connection(True)
        msg = self.ros.publishers['/roarm/connected'].messages[-1]
        self.assertTrue(msg.data)

    def test_timer_sends_only_state_query(self):
        self.ros.timer.callback(None)
        self.assertEqual(self.transport.writes, [b'{"T":105}\n'])

    def test_shutdown_stops_transport(self):
        self.node.shutdown()
        self.assertTrue(self.transport.stopped)


if __name__ == '__main__':
    unittest.main()
