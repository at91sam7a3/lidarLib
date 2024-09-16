project(lidarlib)

aux_source_directory(. LIDAR_SRC_LIST)

add_library(lidarlib STATIC ${LIDAR_SRC_LIST})

