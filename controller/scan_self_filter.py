"""Remove laser endpoints inside the physical chassis; keep raw /scan intact."""
import copy
import math


def mask_ranges(ranges, angle_min, angle_increment, range_min, range_max,
                translation, rotation, bounds):
    """Bounds are base_link xmin,xmax,ymin,ymax in metres; TF is base<-scan.

    NaN removes an occluded measurement without claiming the space behind it
    is free. The scan timestamp, angles and beam ordering are never changed.
    """
    qx, qy, qz, qw = rotation
    # First two rows of the full 3D rotation, including a tilted sensor.
    xx, xy = 1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw)
    yx, yy = 2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz)
    xmin, xmax, ymin, ymax = bounds
    result, count = list(ranges), 0
    for i, distance in enumerate(ranges):
        if not math.isfinite(distance) or not range_min <= distance <= range_max:
            continue
        angle = angle_min + i*angle_increment
        sx, sy = distance*math.cos(angle), distance*math.sin(angle)
        x, y = translation[0]+xx*sx+xy*sy, translation[1]+yx*sx+yy*sy
        if xmin <= x <= xmax and ymin <= y <= ymax:
            result[i] = float('nan')
            count += 1
    return result, count


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformListener, TransformException

    class SelfFilter(Node):
        def __init__(self):
            super().__init__('scan_self_filter')
            for name, value in {'base_frame': 'base_link', 'input_topic': '/scan',
                                'output_topic': '/scan_filtered',
                                'bounds': [-0.21, 0.09, -0.16, 0.16]}.items():
                self.declare_parameter(name, value)
            self.bounds = list(self.get_parameter('bounds').value)
            if (len(self.bounds) != 4 or not all(math.isfinite(x) for x in self.bounds)
                    or self.bounds[0] >= self.bounds[1] or self.bounds[2] >= self.bounds[3]):
                raise ValueError('bounds must be xmin,xmax,ymin,ymax in base_link metres')
            self.base = self.get_parameter('base_frame').value
            source = self.get_parameter('input_topic').value
            target = self.get_parameter('output_topic').value
            if source == target:
                raise ValueError('input_topic and output_topic must differ')
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.publisher = self.create_publisher(LaserScan, target, qos_profile_sensor_data)
            self.subscription = self.create_subscription(LaserScan, source, self.on_scan,
                                                         qos_profile_sensor_data)
            self.last_report = 0.0
            self.get_logger().info(f'{source} -> {target}; chassis bounds={self.bounds}')

        def on_scan(self, msg):
            try:
                # Physical mounting TF only. No map/odom motion or scan retiming.
                tf = self.buffer.lookup_transform(self.base, msg.header.frame_id,
                                                  rclpy.time.Time())
            except TransformException as exc:
                self.get_logger().warning(f'Skipping scan: mounting TF unavailable: {exc}',
                                          throttle_duration_sec=5.0)
                return
            t, q = tf.transform.translation, tf.transform.rotation
            values, count = mask_ranges(msg.ranges, msg.angle_min, msg.angle_increment,
                msg.range_min, msg.range_max, (t.x,t.y,t.z), (q.x,q.y,q.z,q.w), self.bounds)
            filtered = copy.deepcopy(msg)
            filtered.ranges = values
            self.publisher.publish(filtered)
            now = self.get_clock().now().nanoseconds/1e9
            if now-self.last_report >= 5:
                self.get_logger().info(f'masked {count}/{len(values)} chassis returns')
                self.last_report = now

    rclpy.init()
    node = SelfFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
