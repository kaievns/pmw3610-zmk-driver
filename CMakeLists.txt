if(CONFIG_PMW3610)
  target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/pmw3610.c)
endif()

if(CONFIG_ZMK_INPUT_PROCESSOR_PMW3610_UNPACK_XY)
  target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src/input_processor_pmw3610_unpack_xy.c)
endif()
