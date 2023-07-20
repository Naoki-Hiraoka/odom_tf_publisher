# odom_tf_publisher

## odom_tf_publisher

### Subscribed Topics:
- odom (`nav_msgs/Odometry`)

### Param
- `~child_frame_id` (String)
  if not supplied, odom.child_frame_id is used
- `~frame_id_overwrite` (String)
  if supplied, odom.header.frame_id is overwrited
- `~child_frame_id_overwrite` (String)
  if supplied, odom.child_frame_id is overwrited
- `~queue_size` (Int. default 10)
  queue size

### Provided TF Transforms
- odom.header.frame_id (e.g. "odom")->`~child_frame_id` (e.g. "BODY")

### Reguired TF Transforms
- `~child_frame_id` (e.g. "BODY")->odom.child_frame_id (e.g. "camera_optical_frame")
