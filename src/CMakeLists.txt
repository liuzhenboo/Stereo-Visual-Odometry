add_library(robust_vslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        System.cpp
        dataset.cpp
        ORBextractor.cpp)

target_link_libraries(robust_vslam
        ${THIRD_PARTY_LIBS})
