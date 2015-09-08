# - Try to find EST
# Once done this will define
#
# EST_FOUND          - set to true if EST was found
# EST_LIBRARIES      - link these to use EST
# EST_INCLUDE_DIR    - path to EST header files

if (EST_LIBRARIES AND EST_INCLUDE_DIR)
  # in cache already
  set(EST_FOUND TRUE)
else (EST_LIBRARIES AND EST_INCLUDE_DIR)

  set(CANDIDATE_LIB_DIR
    /usr/lib
    /usr/local/lib
    /usr/lib/pulse
    /usr/local/lib/speech_tools
    /usr/local/speech_tools/lib
    /opt/speech_tools/lib
  )

  set(CANDIDATE_INC_DIR
    /usr/include
    /usr/include/speech_tools
    /usr/local/include
    /usr/local/include/speech_tools
    /usr/local/speech_tools/include
    /opt/speech_tools/include
  )

  find_path(EST_INCLUDE_DIR EST.h ${CANDIDATE_INC_DIR})
  find_library(ESD_LIBRARY esd ${CANDIDATE_LIB_DIR})
  find_library(ESTOOLS_LIBRARY estools ${CANDIDATE_LIB_DIR})
  find_library(ESTSTRING_LIBRARY eststring ${CANDIDATE_LIB_DIR})
  find_library(ESTBASE_LIBRARY estbase ${CANDIDATE_LIB_DIR})

  set(EST_LIBRARIES
    ${ESD_LIBRARY}
    ${ESTOOLS_LIBRARY}
    ${ESTSTRING_LIBRARY}
    ${ESTBASE_LIBRARY}
  )

  # status output
  include(FindPackageHandleStandardArgs)

  find_package_handle_standard_args(EST
    DEFAULT_MSG
    ESD_LIBRARY
    ESTOOLS_LIBRARY
    ESTSTRING_LIBRARY
    ESTBASE_LIBRARY
    EST_INCLUDE_DIR
  )
  mark_as_advanced(
    EST_INCLUDE_DIR
    EST_LIBRARIES
  )

endif (EST_LIBRARIES AND EST_INCLUDE_DIR)
