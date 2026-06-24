cmake_minimum_required(VERSION 3.16)
project(IMU_Visualizer CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenGL REQUIRED)
find_package(glfw3  REQUIRED)
find_package(GLUT   REQUIRED)
find_package(Threads REQUIRED)

# GLM — header-only, try pkg-config then manual
find_package(glm QUIET)
if(NOT glm_FOUND)
    find_path(GLM_INCLUDE_DIR glm/glm.hpp
        PATHS /usr/include /usr/local/include)
    if(GLM_INCLUDE_DIR)
        message(STATUS "GLM found at ${GLM_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "GLM not found — install libglm-dev")
    endif()
endif()

add_executable(imu_visualizer
    main.cpp
    gltf_model.cpp
    renderer.cpp
    sensor_manager.cpp
    udp_receiver.cpp
    csv_logger.cpp
    input_handler.cpp
)

target_include_directories(imu_visualizer PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/third_party
    ${GLUT_INCLUDE_DIR}
)
if(GLM_INCLUDE_DIR)
    target_include_directories(imu_visualizer PRIVATE ${GLM_INCLUDE_DIR})
endif()

target_link_libraries(imu_visualizer PRIVATE
    OpenGL::GL
    glfw
    ${GLUT_LIBRARIES}
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

target_compile_options(imu_visualizer PRIVATE -Wall -Wextra -Wno-unused-parameter -O2)

# Copy human.glb next to binary
add_custom_command(TARGET imu_visualizer POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_SOURCE_DIR}/human.glb
        $<TARGET_FILE_DIR:imu_visualizer>/human.glb
    COMMENT "Copying human.glb..."
)
