import unittest
from xzb_roarm_driver.ros_node import RoArmRosNode

class Header:
    def __init__(self): self.stamp=None; self.frame_id=''
class JointState:
    def __init__(self): self.header=Header(); self.name=[]; self.position=[]; self.velocity=[]; self.effort=[]
class Position:
    def __init__(self): self.x=0.; self.y=0.; self.z=0.
class Orientation:
    def __init__(self): self.x=0.; self.y=0.; self.z=0.; self.w=0.
class Pose:
    def __init__(self): self.position=Position(); self.orientation=Orientation()
class Bool:
    def __init__(self,data=False): self.data=data
class String:
    def __init__(self,data=''): self.data=data
class FakePublisher:
    def __init__(self,topic,msg_type,queue_size=10,latch=False): self.topic=topic; self.msg_type=msg_type; self.latch=latch; self.messages=[]
    def publish(self,msg): self.messages.append(msg)
class FakeTimer:
    def __init__(self,duration,callback): self.duration=duration; self.callback=callback
class FakeTime:
    @staticmethod
    def now(): return 'STAMP'
class FakeRospy:
    Time=FakeTime
    def __init__(self):
        self.params={'~port':'/dev/ttyUSB0','~baud_rate':115200,'~query_rate':2.0,'~reconnect_interval':1.0,'~serial_timeout':0.2,'~joint_frame_id':'roarm_base_link','~command_enable':False}; self.publishers={}; self.timer=None
    def get_param(self,n,d=None): return self.params.get(n,d)
    def Publisher(self,t,m,queue_size=10,latch=False): p=FakePublisher(t,m,queue_size,latch); self.publishers[t]=p; return p
    def Duration(self,v): return v
    def Timer(self,d,c): self.timer=FakeTimer(d,c); return self.timer
    def on_shutdown(self,cb): self.shutdown_cb=cb
    def loginfo(self,*a): pass
    def logwarn(self,*a): pass
    def logwarn_throttle(self,*a): pass
    def logdebug(self,*a): pass
class FakeTransport:
    instances=[]
    def __init__(self,port,baud_rate=115200,timeout=.2,reconnect_interval=1.,**k): self.port=port; self.baud_rate=baud_rate; self.connected=True; self.writes=[]; self.__class__.instances.append(self)
    def start(self): pass
    def stop(self): self.connected=False
    def write(self,p): self.writes.append(p)
MESSAGE_TYPES={'Bool':Bool,'String':String,'JointState':JointState,'Pose':Pose}

class RosNodeTests(unittest.TestCase):
    def setUp(self):
        FakeTransport.instances=[]; self.ros=FakeRospy(); self.node=RoArmRosNode(rospy_module=self.ros,message_types=MESSAGE_TYPES,transport_factory=FakeTransport); self.transport=FakeTransport.instances[-1]
    def test_topics_and_state(self):
        self.assertEqual(set(self.ros.publishers),{'/roarm/connected','/roarm/joint_states','/roarm/controller_joint_states','/roarm/pose','/roarm/raw_response'})
        line='{"T":1051,"x":515.88,"y":-3.95,"z":-49.68,"b":-0.01,"s":1.59,"e":0.03,"t":1.69,"torB":36,"torS":104,"torE":92,"torH":300}'
        self.transport.on_line(line)
        self.assertEqual(self.ros.publishers['/roarm/controller_joint_states'].messages[-1].position,[-0.01,1.59,0.03,1.69])
    def test_timer_remains_state_query(self):
        self.ros.timer.callback(None); self.assertEqual(self.transport.writes,[b'{"T":105}\n'])

if __name__ == '__main__': unittest.main()
