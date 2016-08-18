
macro(generate_parameter_files)
    set(CFG_FILES "${ARGN}")
    set(ROSPARAM_HANDLER_ROOT_DIR "${ROSPARAM_HANDLER_CMAKE_DIR}/..")
    if (${PROJECT_NAME}_CATKIN_PACKAGE)
        message(FATAL_ERROR "generate_parameter_files() must be called before catkin_package() in project '${PROJECT_NAME}'")
    endif ()

    # ensure that package destination variables are defined
    catkin_destinations()
    if(dynamic_reconfigure_FOUND_CATKIN_PROJECT)
        add_definitions(-DDYNAMIC_RECONFIGURE_FOUND)
    endif()

    set(_autogen "")
    foreach (_cfg ${CFG_FILES})
        # Construct the path to the .cfg file
        set(_input ${_cfg})
        if (NOT IS_ABSOLUTE ${_input})
            set(_input ${PROJECT_SOURCE_DIR}/${_input})
        endif ()

        get_filename_component(_cfgext ${_cfg} EXT)
        if( _cfgext STREQUAL ".params" OR _cfgext STREQUAL ".mrtcfg")
            # Define required input files
            set(genparam_build_files
                    ${ROSPARAM_HANDLER_ROOT_DIR}/templates/ConfigType.h.template
                    ${ROSPARAM_HANDLER_ROOT_DIR}/templates/Parameters.h.template
                    )

            # Define output files
            get_filename_component(_cfgonly ${_cfg} NAME_WE)
            set(_output_cfg ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg/${_cfgonly}.cfg)
            set(_output_cpp ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}/${_cfgonly}Parameters.h)

            # Create command
            assert(CATKIN_ENV)
            set(_cmd
                    ${CATKIN_ENV}
                    ${_input}
                    ${ROSPARAM_HANDLER_ROOT_DIR}
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
                    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_INCLUDE_DESTINATION}
                    )

            add_custom_command(OUTPUT
                    ${_output_cpp} ${_output_cfg}
                    COMMAND ${_cmd}
                    DEPENDS ${_input} ${genparam_build_files}
                    COMMENT "Generating parameter files from ${_cfgonly}"
                    )

            list(APPEND ${PROJECT_NAME}_LOCAL_CFG_FILES "${_output_cfg}")
            list(APPEND ${PROJECT_NAME}_params_generated ${_output_cpp} ${_output_cfg})

            install(
                    FILES ${_output_cpp}
                    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
            )
        elseif( _cfgext STREQUAL ".cfg" )
            list(APPEND ${PROJECT_NAME}_LOCAL_CFG_FILES "${_output_cfg}")
        else()
            message(WARNING "Unknown file ending : ${_cfgext}. Skipping")
        endif()

    endforeach (_cfg)

    # genparam target for hard dependency on generate_parameter generation
    add_custom_target(${PROJECT_NAME}_genparam ALL DEPENDS ${${PROJECT_NAME}_params_generated})

    # register target for catkin_package(EXPORTED_TARGETS)
    list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ${PROJECT_NAME}_genparam)

    # make sure we can find generated headers and that they overlay all other includes
    include_directories(BEFORE ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    # pass the include directory to catkin_package()
    list(APPEND ${PROJECT_NAME}_INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})
    # ensure that the folder exists
    file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})


    # generate dynamic reconfigure files
    if(dynamic_reconfigure_FOUND_CATKIN_PROJECT)
        if(${PROJECT_NAME}_LOCAL_CFG_FILES)
            generate_dynamic_reconfigure_options(${${PROJECT_NAME}_LOCAL_CFG_FILES})
        endif()
    else()
        message(WARNING "Dependency to dynamic_reconfigure is missing, or find_package(dynamic_reconfigure) was not called yet. Not building dynamic config files")
    endif()
endmacro()
