# base_footprint_publisher

## base_footprint_publisher

### Param
- `~rleg_frame` (String) (default: rleg_end_coords)
- `~lleg_frame` (String) (default: lleg_end_coords)
- `~world_frame` (String) (default: odom)
- `~frame_id` (String) (default: base_footprint)

### Provided TF Transforms
- `~world_frame`->`~frame_id`
  `~rleg_frame`と`~lleg_frame`の中間. ただし`~world_frame`でZ軸は鉛直で、Z座標は`~rleg_frame`と`~lleg_frame`の低い方

### Reguired TF Transforms
- `~world_frame`->`~rleg_frame`
- `~world_frame`->`~lleg_frame`
