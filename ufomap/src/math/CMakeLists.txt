add_library(ufomap_math  SHARED pose6.cpp quaternion.cpp)

set_target_properties(ufomap_math PROPERTIES
	VERSION ${UFOMAP_VERSION}
	SOVERSION ${UFOMAP_SOVERSION}
)