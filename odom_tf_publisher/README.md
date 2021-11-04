# odom_tf_publisher

## odom_tf_publisher

### Subscribed Topics:
- odom (`nav_msgs/Odometry`)

### Param
- `~child_frame_id` (String)
  if not supplied, odom.header.child_frame_id is used

### Provided TF Transforms
- odom.header.frame_id (e.g. "odom")->`~child_frame_id` (e.g. "BODY")

### Reguired TF Transforms
- `~child_frame_id` (e.g. "BODY")->odom.header.child_frame_id (e.g. "camera_optical_frame")
