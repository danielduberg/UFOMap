# Flag to denote a binary (only free/occupied/unknown) or full occupancy Octree
bool binary

# Class id of the contained Octree
string id

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
int32 data_size