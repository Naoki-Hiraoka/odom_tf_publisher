# base_footprint_publisher

## base_footprint_publisher

### Param
- `~parenttree_rootframe` (String)
- `~parenttree_targetframe` (String)
- `~childtree_rootframe` (String)
- `~childtree_targetframe` (String)

### Provided TF Transforms
- `~parenttree_rootframe`->`~childtree_rootframe`
  `~parenttree_targetframe`と`~childtree_targetframe`を一致させる

### Reguired TF Transforms
- `~parenttree_rootframe`->`~parenttree_targetframe`
- `~childtree_rootframe`->`~childtree_targetframe`
