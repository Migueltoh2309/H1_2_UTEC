from visualization_msgs.msg import Marker

def create_sphere_marker(frame="torso_link",
                         ns="marker",
                         marker_id=0,
                         scale=0.05,
                         color=(1.0,0.0,0.0,1.0)):

    marker = Marker()

    marker.header.frame_id = frame
    marker.ns = ns
    marker.id = marker_id

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD

    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale

    marker.color.r = float(color[0])
    marker.color.g = float(color[1])
    marker.color.b = float(color[2])
    marker.color.a = float(color[3])

    return marker


def set_marker_pose(marker, pose, node):

    marker.header.stamp = node.get_clock().now().to_msg()

    marker.pose.position.x = float(pose[0])
    marker.pose.position.y = float(pose[1])
    marker.pose.position.z = float(pose[2])

    marker.pose.orientation.w = float(pose[3])
    marker.pose.orientation.x = float(pose[4])
    marker.pose.orientation.y = float(pose[5])
    marker.pose.orientation.z = float(pose[6])

    return marker

# 🔵 Marker de trayectoria
def create_line_marker(frame="torso_link"):
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns = "trajectory"
    marker.id = 1
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    marker.scale.x = 0.01

    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    marker.pose.orientation.w = 1.0
    marker.points = []

    return marker