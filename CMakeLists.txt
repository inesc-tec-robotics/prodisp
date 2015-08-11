cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_BUILD_TYPE Debug)
set(PROJECT_NAME prodisp)

project(${PROJECT_NAME})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  LoggerSys
  mission_ctrl_msgs
  MathOp
  FileSystemOperations
)

# find and setup Qt4 for this project
set(QT_QMAKE_EXECUTABLE /usr/bin/qmake-qt4) #Force the system to use QT4
find_package(Qt4 REQUIRED)

# OpenGL and GLUT:
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIRS})

# OpenCV:
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Boost:
find_package(Boost REQUIRED COMPONENTS
             thread system filesystem ) # timer
include_directories(${Boost_INCLUDE_DIRS})

# tf:
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# roslib:
find_package(roslib REQUIRED)
include_directories(${roslib_INCLUDE_DIRS})

SET(QT_USE_QTXML true)
SET(QT_USE_QTNETWORK true)
SET(QT_USE_QTGUI true)
SET(QT_USE_QTCORE true)
SET(QT_USE_QDBUS true)

include(${QT_USE_FILE})

INCLUDE_DIRECTORIES(
   ${QT_INCLUDE_DIR}
)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${QT_INCLUDE_DIR})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

# set(qt_srcs
# XMLOperations.cpp
# )
# 
# set(qt_hdrs
# include/XMLOperations.hpp
# )
# 
# qt4_automoc(${qt_srcs})
# 
# QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES XMLOperations MathOp
  CATKIN_DEPENDS
  roscpp
  roslib
  LoggerSys
  DEPENDS Boost
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


add_executable(${PROJECT_NAME}
  prodisp.cfg

  include/application.h
  include/configuration.h
  include/common.h
  include/cursor.h
  include/interface_handler.h
  include/renderer.h
  include/render_system.h
  include/runnable.h
  include/singleton.h
  include/scene.h
  include/texture.h
  include/timer_ms.h
  include/wii_handler.h
  include/objects/cad_model.h
  include/objects/cube.h
  include/objects/fire_extinguisher.h
  include/objects/glm.h
  include/objects/line_circle.h
  include/objects/line_cross.h
  include/objects/line_coordsys.h
  include/objects/mesh_generic.h
  include/objects/model3ds.h
  include/objects/object.h
  include/objects/object_composer.h
  include/objects/plane.h
  include/objects/position.h
  include/objects/point_cloud.h
  include/objects/primitive_surface.h
  include/objects/primitive_line.h
  include/objects/sphere.h
  include/objects/stud.h
  include/objects/tf.h
  include/objects/renderable.h
  include/objects/vertex_buffer_object.h

  src/prodisp.cpp
  src/application.cpp
  src/configuration.cpp
  src/common.cpp
  src/cursor.cpp
  src/interface_handler.cpp
  src/renderer.cpp
  src/render_system.cpp
  src/runnable.cpp
  src/scene.cpp
  src/texture.cpp
  src/timer_ms.cpp
  src/wii_handler.cpp
  src/objects/cad_model.cpp
  src/objects/cube.cpp
  src/objects/fire_extinguisher.cpp
  src/objects/glm.cpp
  src/objects/line_circle.cpp
  src/objects/line_cross.cpp
  src/objects/line_coordsys.cpp
  src/objects/mesh_generic.cpp
  src/objects/model3ds.cpp
  src/objects/object.cpp
  src/objects/object_composer.cpp
  src/objects/plane.cpp
  src/objects/point_cloud.cpp
  src/objects/primitive_surface.cpp
  src/objects/primitive_line.cpp
  src/objects/sphere.cpp
  src/objects/stud.cpp
  src/objects/tf.cpp
  src/objects/vertex_buffer_object.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${LoggerSys_LIBRARIES}
  ${FileSystemOperations_LIBRARIES}
  ${MathOp_LIBRARIES}
  ${roslib_LIBRARIES}
)
                      
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
                      
                      
install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h")
