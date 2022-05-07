add_executable(move_group_interface_panda src/move_group_interface.cpp)
target_link_libraries(move_group_interface_panda ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface_panda DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
