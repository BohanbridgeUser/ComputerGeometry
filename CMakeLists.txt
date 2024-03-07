CMAKE_MINIMUM_REQUIRED (VERSION 3.1...3.23)

PROJECT (MYCG)

# Instruct CMake to run moc automatically when needed.
SET(CMAKE_AUTOMOC ON)
IF(NOT POLICY CMP0070 AND POLICY CMP0053)
  # Only set CMP0053 to OLD with CMake<3.10, otherwise there is a warning.
  CMAKE_POLICY(SET CMP0053 OLD)
ENDIF()
IF(POLICY CMP0071)
  CMAKE_POLICY(SET CMP0071 NEW)
ENDIF()

# Find CGAL and CGAL Qt5
FIND_PACKAGE(CGAL COMPONENTS Qt5)
# Find Qt5 itself
FIND_PACKAGE(Qt5 QUIET COMPONENTS OpenGL)
# Find OpenGL
FIND_PACKAGE(OpenGL)
# Find Eigen3
FIND_PACKAGE(Eigen3 3.1.0) #(requires 3.1.0 or greater)
INCLUDE(CGAL_Eigen3_support)

FILE(GLOB SRC_FILES "./src/*.cpp")
FILE(GLOB_RECURSE INCS "*.h")

IF(CGAL_FOUND AND Qt5_FOUND AND Eigen3_FOUND AND OpenGL_FOUND)
  # Include this package's headers first
  INCLUDE_DIRECTORIES(BEFORE ./ ./include ./res)
  INCLUDE(${CGAL_USE_FILE})
  INCLUDE(${EIGEN3_USE_FILE})

  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/build)
  # ui file, created with Qt Designer
  qt5_wrap_ui( uis res/UI/MyUI.ui )
  ADD_EXECUTABLE ( ${PROJECT_NAME} ${SRC_FILES} ${MOCS} ${uis} ${INCS})
  qt5_use_modules(${PROJECT_NAME} OpenGL)

  # Link with Qt libraries
  TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${QT_LIBRARIES})
  # Link with OpenGL libraries
  TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${OPENGL_LIBRARIES})
  # Link with CGAL libraries
  TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
else()
  MESSAGE("NOTICE: This demo requires CGAL, Qt5, and Eigen, and will not be compiled.")
endif()
