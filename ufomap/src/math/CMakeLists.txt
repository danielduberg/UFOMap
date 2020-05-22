set(ufomath_srcs
	pose6.cpp
	quaternion.cpp
)

add_library(ufomath SHARED ${ufomath_srcs})

set_target_properties(ufomath PROPERTIES
	VERSION ${UFOMAP_VERSION}
	SOVERSION ${UFOMAP_SOVERSION}
)

add_library(ufomath_static STATIC {ufomath_srcs})

set_target_properties(ufomath_static PROPERTIES
	OUTPUT_NAME "ufomath"
)

export(TARGETS ufomath ufomath_static
	APPEND FILE 
)

install(TARGETS ufomath ufomath_static
	EXPORT ufomap-targets
)