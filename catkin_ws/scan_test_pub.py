#!/usr/bin/env python3
import rospy, struct
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField

def cloud(points):
    msg=PointCloud2(); msg.header.frame_id='world'; msg.height=1; msg.width=len(points)
    msg.fields=[PointField('x',0,PointField.FLOAT32,1),PointField('y',4,PointField.FLOAT32,1),PointField('z',8,PointField.FLOAT32,1)]
    msg.is_bigendian=False; msg.point_step=12; msg.row_step=12*len(points); msg.is_dense=True
    msg.data=b''.join(struct.pack('fff',*p) for p in points); return msg

rospy.init_node('scan_test_pub'); od=rospy.Publisher('/test/odom',Odometry,queue_size=1); pc=rospy.Publisher('/test/cloud',PointCloud2,queue_size=1); goal=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
rate=rospy.Rate(10); t0=rospy.Time.now(); sent=False
pts=[]
for x in [i/10 for i in range(-50,151)]:
  for y in [i/10 for i in range(-30,31)]: pts.append((x,y,0.0))
for x in [i/10 for i in range(25,36)]:
  for y in [i/10 for i in range(-10,11)]:
    for z in [i/10 for i in range(0,11)]: pts.append((x,y,z))
while not rospy.is_shutdown():
  now=rospy.Time.now(); o=Odometry(); o.header.stamp=now; o.header.frame_id='world'; o.child_frame_id='base'; o.pose.pose.position.x=0; o.pose.pose.position.z=0.3; o.pose.pose.orientation.w=1; od.publish(o)
  c=cloud(pts); c.header.stamp=now; pc.publish(c)
  if not sent and (now-t0).to_sec()>2:
    g=PoseStamped(); g.header.stamp=now; g.header.frame_id='world'; g.pose.position.x=8; g.pose.position.z=0.3; g.pose.orientation.w=1; goal.publish(g); sent=True; rospy.loginfo('published test goal and obstacle cloud')
  rate.sleep()
