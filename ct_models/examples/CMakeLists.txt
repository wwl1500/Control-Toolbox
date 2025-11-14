
set(CT_MODELS_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/exampleDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/exampleDir.h)

if(CPPADCG)
  add_executable(ex_NLOC_MPC_invertedPendulum mpc/InvertedPendulum/NLOC_MPC.cpp)
  target_include_directories(ex_NLOC_MPC_invertedPendulum PUBLIC ${ct_models_target_include_dirs})
  target_link_libraries(ex_NLOC_MPC_invertedPendulum ct_rbd)
  
  ## install examples
  include(GNUInstallDirs)
  install(
      TARGETS ex_NLOC_MPC_invertedPendulum
      RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_models
      )
  
endif()

add_executable(ex_underwater_double_link UnderwaterDoubleLinkExample.cpp)
target_include_directories(ex_underwater_double_link PUBLIC ${ct_models_target_include_dirs})
target_link_libraries(ex_underwater_double_link ct_core ct_hydro)

include(GNUInstallDirs)
install(
    TARGETS ex_underwater_double_link
    RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/ct_models
    )