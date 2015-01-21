# Detect Ubuntu Flavor
# The following variable will be set
# $UBUNTU_DIST (e.g. UBUNTU_DIST=14.04)

if (NOT WINDOWS)	
  if (EXISTS "/etc/os-release")
    execute_process(
      COMMAND grep VERSION_ID=
      COMMAND sed -e "s/VERSION_ID=\"\\(.*\\)\"/\\1/"
      INPUT_FILE "/etc/os-release"
      RESULT_VARIABLE RES
      OUTPUT_VARIABLE UBUNTU_DIST
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_QUIET
      )

    if (RES EQUAL 0)
      message (STATUS "Building for Ubuntu ${UBUNTU_DIST}")
    else()
      message (STATUS "Ubuntu not detected")
      set (UBUNTU_DIST "NA")
    endif()

  endif()

endif()	

