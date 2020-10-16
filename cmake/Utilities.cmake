
# Add tests for all target software binaries belonging to a test board
function(add_sw_tests BOARDNAME)
  message("Adding software tests for ${BOARDNAME}")
  set(SWTEST_BUILD_DIR ${PROJECT_SOURCE_DIR}/sw/build/validation)
  file(GLOB_RECURSE TARGET_BINS "${SWTEST_BUILD_DIR}/${BOARDNAME}/*.hex")
  foreach(HEXFILE ${TARGET_BINS})
    get_filename_component(PROGNAME ${HEXFILE} NAME)
    set(TESTNAME "${BOARDNAME}-${PROGNAME}")
    add_test(
      NAME ${TESTNAME}
      COMMAND fused --board ${BOARDNAME} -x ${HEXFILE}
    )
    set_tests_properties(${TESTNAME} PROPERTIES TIMEOUT 10)
    message("Added test ${TESTNAME}")
  endforeach()
endfunction()
