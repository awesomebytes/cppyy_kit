"""RViz drag handle for a bounded virtual force, expressed in the arm base frame."""
import math
import time

from geometry_msgs.msg import Point, Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback, Marker, MarkerArray


class RvizForce:
    def __init__(self, node, frame, origin, max_force=5.0, gain=20.0):
        self.node, self.frame = node, frame
        self.origin = [float(v) for v in origin]
        self.position = self.origin.copy()
        self.max_force, self.gain = max_force, gain
        self.force = [0.0, 0.0, 0.0]
        self.dragging = False
        self.stamp = 0.0
        self.server = InteractiveMarkerServer(node, 'virtual_force')
        self.publisher = node.create_publisher(MarkerArray, '/virtual_force/markers', 1)
        marker = InteractiveMarker()
        marker.header.frame_id = frame
        marker.name = 'force_handle'
        marker.description = 'Virtual force'
        marker.scale = 0.28
        marker.pose = self._pose(self.origin)
        center = InteractiveMarkerControl()
        center.always_visible = True
        center.interaction_mode = InteractiveMarkerControl.MOVE_3D
        ball = Marker(type=Marker.SPHERE)
        ball.pose.orientation.w = 1.0
        ball.scale.x = ball.scale.y = ball.scale.z = 0.035
        ball.color.r, ball.color.g, ball.color.b, ball.color.a = 1.0, 0.65, 0.0, 1.0
        center.markers.append(ball)
        marker.controls.append(center)
        for axis, orientation in [('x', (0., 0., 0., 1.)),
                                  ('y', (0., 0., math.sqrt(.5), math.sqrt(.5))),
                                  ('z', (0., -math.sqrt(.5), 0., math.sqrt(.5)))]:
            control = InteractiveMarkerControl()
            control.name = 'move_' + axis
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            control.orientation.x, control.orientation.y, control.orientation.z, control.orientation.w = orientation
            marker.controls.append(control)
        self.server.insert(marker, feedback_callback=self.feedback)
        self.server.applyChanges()

    @staticmethod
    def _pose(xyz):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        return pose

    def reset(self):
        self.dragging = False
        self.force = [0.0, 0.0, 0.0]
        self.server.setPose('force_handle', self._pose(self.origin))
        self.server.applyChanges()

    def feedback(self, message):
        if message.marker_name != 'force_handle':
            return
        if message.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self.reset()
            return
        if message.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self.dragging = True
        if not self.dragging:
            return
        self.stamp = time.monotonic()
        if message.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            xyz = [message.pose.position.x, message.pose.position.y, message.pose.position.z]
            if not all(math.isfinite(v) for v in xyz):
                self.reset()
                return
            force = [(v - o) * self.gain for v, o in zip(xyz, self.origin)]
            norm = math.sqrt(sum(v*v for v in force))
            scale = min(1.0, self.max_force / norm) if norm else 1.0
            self.force = [v * scale for v in force]

    def get_force(self):
        # RViz sends KEEP_ALIVE feedback while a control is held. Loss of the
        # viewer/feedback stops the force even if MOUSE_UP never arrives.
        if self.dragging and time.monotonic() - self.stamp > 0.6:
            self.reset()
        return self.force.copy()

    def set_state(self, position):
        self.position = position
        force = self.get_force()
        arrow = Marker()
        arrow.header.frame_id = self.frame
        arrow.ns, arrow.id, arrow.type = 'virtual_force', 0, Marker.ARROW
        arrow.pose.orientation.w = 1.0
        arrow.points = [Point(x=self.origin[0], y=self.origin[1], z=self.origin[2]),
                        Point(x=self.origin[0]+force[0]/self.gain,
                              y=self.origin[1]+force[1]/self.gain,
                              z=self.origin[2]+force[2]/self.gain)]
        arrow.scale.x, arrow.scale.y, arrow.scale.z = .012, .025, .035
        arrow.color.r, arrow.color.g, arrow.color.a = 1.0, .4, 1.0
        label = Marker()
        label.header = arrow.header
        label.ns, label.id, label.type = 'virtual_force', 1, Marker.TEXT_VIEW_FACING
        label.pose = self._pose([self.origin[0], self.origin[1], self.origin[2]+.23])
        label.scale.z = .035
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        displacement = math.dist(self.origin, position)*1000
        label.text = f'F={math.sqrt(sum(v*v for v in force)):.1f}N\noffset={displacement:.1f}mm'
        self.publisher.publish(MarkerArray(markers=[arrow, label]))

    def close(self):
        self.reset()
        self.server.clear()
        self.server.applyChanges()
        self.server.shutdown()
        self.node.destroy_publisher(self.publisher)
