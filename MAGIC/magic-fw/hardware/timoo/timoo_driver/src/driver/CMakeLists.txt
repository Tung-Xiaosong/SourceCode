# build the driver node
add_executable(timoo_node timoo_node.cc driver.cc)
add_dependencies(timoo_node timoo_driver_gencfg)
target_link_libraries(timoo_node
  timoo_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(timoo_driver_nodelet nodelet.cc driver.cc)
add_dependencies(timoo_driver_nodelet timoo_driver_gencfg)
target_link_libraries(timoo_driver_nodelet
  timoo_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS timoo_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS timoo_driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
