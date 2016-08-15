# Generated from: rosparam_handler/cmake/rosparam_handler-extras.cmake.em

if (_ROSPARAM_HANDLER_EXTRAS_INCLUDED_)
    return()
endif ()
set(_ROSPARAM_HANDLER_EXTRAS_INCLUDED_ TRUE)

@[if DEVELSPACE]@
# cmake dir in develspace
set(ROSPARAM_HANDLER_CMAKE_DIR "@(CMAKE_CURRENT_SOURCE_DIR)/cmake")
@[else]@
# cmake dir in installspace
set(ROSPARAM_HANDLER_CMAKE_DIR "@(PKG_CMAKE_DIR)")
@[end if]@

include(${ROSPARAM_HANDLER_CMAKE_DIR}/rosparam_handler-macros.cmake)
