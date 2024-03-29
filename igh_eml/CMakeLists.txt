cmake_minimum_required(VERSION 2.8.3)
project(igh_eml)

#To avoid problem on early check of existance of this directory
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)

find_package(catkin REQUIRED)

include(ExternalProject)
ExternalProject_Add(upstream_igh_eml    # Name for custom target
# #--Download step--------------
  #HG_REPOSITORY http://hg.code.sf.net/p/etherlabmaster/code         # URL of mercurial repo
  #HG_TAG f5ffd1875da1                # Mercurial branch name, commit id or tag
  #Switch to a UniBremen mirror of the upstream igh code (faster than sourceforge)
  #GIT_REPOSITORY https://gitlab.informatik.uni-bremen.de/amaldo/igh_eml_upstream_clone.git
  #Using the github repositoy of ribalda
  GIT_REPOSITORY https://github.com/ribalda/ethercat.git
  #The following TAG should correspond to the same version as your installed kernel modules
  GIT_TAG 97e7ee64a6b4a2e3847270797cae681b8484a7d0
  #GIT_REPOSITORY https://github.com/amaldo/Etherlab_EtherCAT_Master.git
  #GIT_TAG a3413fd8044fe10723602601417397c8524567ab
  TIMEOUT 30           # Time allowed for file download operations
# #--Update/Patch step----------
  UPDATE_COMMAND touch <SOURCE_DIR>/config.h COMMAND mkdir -p <SOURCE_DIR>/include <SOURCE_DIR>/lib    # Source work-tree update command
# #--Configure step-------------
  CONFIGURE_COMMAND touch <SOURCE_DIR>/ChangeLog COMMAND autoreconf -i <SOURCE_DIR> COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-kernel --enable-hrtimer --enable-rtmutex --enable-tool --disable-eoe # Build tree configuration command
# #--Build step-----------------
  BUILD_IN_SOURCE 1         # Use source dir for build dir
# #--Install step---------------
  INSTALL_DIR ${CATKIN_DEVEL_PREFIX}           # Installation prefix
  BUILD_COMMAND cd <SOURCE_DIR> COMMAND make -j8
  )

add_library(ethercat SHARED IMPORTED)
set_target_properties(ethercat PROPERTIES IMPORTED_LOCATION ${CATKIN_DEVEL_PREFIX}/lib/libethercat.so)
add_dependencies(ethercat upstream_igh_eml)

catkin_package(
  LIBRARIES ethercat
  INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include
)

