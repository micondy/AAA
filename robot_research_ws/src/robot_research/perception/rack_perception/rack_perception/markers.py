"""RViz MarkerArray creation for rack models."""

from __future__ import annotations

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _point(values) -> Point:
    point = Point()
    point.x, point.y, point.z = (float(value) for value in values)
    return point


def model_markers(model: dict, stamp) -> MarkerArray:
    output = MarkerArray()
    delete = Marker()
    delete.action = Marker.DELETEALL
    output.markers.append(delete)
    marker_id = 1
    frame_id = str(model.get("frame_id", "odom"))
    for rack in model.get("racks", []):
        for layer in rack.get("layers", []):
            boundary = layer["boundary"]
            outline = Marker()
            outline.header.frame_id = frame_id
            outline.header.stamp = stamp
            outline.ns = "rack_layers"
            outline.id = marker_id
            marker_id += 1
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.scale.x = 0.018
            outline.color = ColorRGBA(r=0.10, g=0.85, b=0.25, a=0.95)
            outline.points = [_point(item) for item in boundary + [boundary[0]]]
            output.markers.append(outline)

            front = Marker()
            front.header = outline.header
            front.ns = "rack_fronts"
            front.id = marker_id
            marker_id += 1
            front.type = Marker.ARROW
            front.action = Marker.ADD
            front.scale.x, front.scale.y, front.scale.z = 0.025, 0.06, 0.08
            start = layer["front_center"]
            normal = layer["front_normal"]
            front.points = [
                _point(start),
                _point((start[0] + 0.30 * normal[0], start[1] + 0.30 * normal[1], start[2])),
            ]
            front.color = ColorRGBA(r=0.95, g=0.35, b=0.05, a=1.0)
            output.markers.append(front)

            label = Marker()
            label.header = outline.header
            label.ns = "rack_ids"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = _point(layer["center"])
            label.pose.position.z += 0.08
            label.pose.orientation.w = 1.0
            label.scale.z = 0.10
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            state = "obs" if layer["observed"] else "held"
            label.text = f"{rack['stable_id']} L{layer['level_number']} {layer['stable_id']} [{state}]"
            output.markers.append(label)
    return output
