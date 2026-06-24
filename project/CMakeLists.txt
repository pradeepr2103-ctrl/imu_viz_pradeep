cmake_minimum_required(VERSION 3.16)
project(BharatanatyamVisualizer CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

find_package(OpenGL  REQUIRED)
find_package(glfw3   REQUIRED)
find_package(assimp  REQUIRED)
find_package(Threads REQUIRED)

# ── GLAD (real generated loader from glad.dav1d.de — OpenGL 3.3 Core) ────────
add_library(glad STATIC
    third_party/glad/src/gl.c
)
target_include_directories(glad PUBLIC
    third_party/glad/include
)

# ── Main executable ───────────────────────────────────────────────────────────
add_executable(bharatanatyam_viz
    visualizer/src/main.cpp
    visualizer/src/Model.cpp
    visualizer/src/Renderer.cpp
    visualizer/src/MotionRecorder.cpp
    visualizer/src/Network.cpp
)

target_include_directories(bharatanatyam_viz PRIVATE
    visualizer/include
    third_party/glad/include
)

target_link_libraries(bharatanatyam_viz PRIVATE
    glad
    OpenGL::GL
    glfw
    assimp
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

target_compile_options(bharatanatyam_viz PRIVATE
    -Wall -Wextra -Wno-unused-parameter -O2
)

# ── Copy assets next to binary at build time ──────────────────────────────────
add_custom_command(TARGET bharatanatyam_viz POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/visualizer/assets
        $<TARGET_FILE_DIR:bharatanatyam_viz>/visualizer/assets
    COMMENT "Copying assets..."
)
