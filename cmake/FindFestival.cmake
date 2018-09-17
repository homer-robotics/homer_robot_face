# - Try to find Festival and EST
# Once done this will define
#
# Festival_FOUND          - set to true if Festival was found
# Festival_LIBRARIES      - link these to use Festival
# Festival_INCLUDE_DIR    - path to Festival header files

if (Festival_LIBRARIES AND Festival_INCLUDE_DIR)
  # in cache already
  set(Festival_FOUND TRUE)
else (Festival_LIBRARIES AND Festival_INCLUDE_DIR)

  set(CANDIDATE_LIB_DIR
    /usr/lib
    /usr/local/lib
    /usr/lib/festival
    /usr/local/lib/festival
    /usr/local/festival/lib
    /opt/festival/lib
  )
  
  set(CANDIDATE_INC_DIR
    /usr/include
    /usr/include/festival
    /usr/local/include
    /usr/local/include/festival
    /usr/local/festival/include
    /opt/festival/include
  )

  find_path(Festival_INCLUDE_DIR festival.h ${CANDIDATE_INC_DIR})
  find_library(Festival_LIBRARIES Festival ${CANDIDATE_LIB_DIR})

  # status output
  include(FindPackageHandleStandardArgs)

  find_package_handle_standard_args(Festival
    DEFAULT_MSG
    Festival_LIBRARIES
    Festival_INCLUDE_DIR
  )
  mark_as_advanced(
    Festival_INCLUDE_DIR
    Festival_LIBRARIES
  )

endif (Festival_LIBRARIES AND Festival_INCLUDE_DIR)
