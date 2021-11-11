# com_tf_publisher

## com_tf_publisher

### Subscribed Topics:
- joint_states (`sensor_msgs/JointState`)

### Param
- `~model` (String) : choreonoid file name
- `~frame_id` (String)
- `~base_frame_id` (String)
- `~com_frame_id` (String)

### Provided TF Transforms
- `~frame_id` (e.g. "odom")->`~com_frame_id` (e.g. "com")

### Reguired TF Transforms
- `~frame_id` (e.g. "odom")->`~base_frame_id` (e.g. "base_link")
