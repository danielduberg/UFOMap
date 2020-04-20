# File version used
string version

# Class id of the contained Octree
string id

# Flag to denote a binary (only free/occupied/unknown) or full occupancy Octree
bool binary

# Resolution (in m) of the octree leaf nodes
float32 resolution

# Number of depth levels
uint8 depth_levels

# Occupancy threshold
float32 occupancy_thres

# Free threshold
float32 free_thres

# If data is compressed
bool compressed

# Size of data uncompressed
int32 uncompressed_data_size

# Bounding volume
ufomap_msgs/BoundingVolume bounding_volume