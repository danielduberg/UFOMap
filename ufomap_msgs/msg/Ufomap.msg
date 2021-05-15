# A 3D map in binary format, as Octree
std_msgs/Header header

# Flag to denote a binary (only free/occupied/unknown) or full occupancy Octree
bool binary

# Class id of the contained Octree
string id

# Resolution (in m) of the smallest Octree nodes
float32 resolution

# Number of depth levels
uint8 depth_levels # Should be uint8

# Occupancy threshold
float32 occupancy_thres

# Free threshold
float32 free_thres

# If data is compressed
bool compressed

# Size uncompressed
int32 data_size

# Size compressed (0 if not compressed)
int32 compressed_data_size

# Binary serialization of Octree, use conversions.h to read and write octrees
int8[] data