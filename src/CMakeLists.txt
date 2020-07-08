add_library(lzbslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp
        ORBextractor.cpp)

target_link_libraries(lzbslam
        ${THIRD_PARTY_LIBS})
