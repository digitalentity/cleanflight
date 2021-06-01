set(MAIN_INCLUDE_DIRS
    "${MAIN_LIB_DIR}"
    "${MAIN_SRC_DIR}"
    "${MAIN_LIB_DIR}/main/MAVLink"
    "${MAIN_LIB_DIR}/main/micro_ros_arduino/src"
)

set(MAIN_DEFINITIONS
    __FORKNAME__=inav
    __REVISION__="${GIT_REV}"
)

set(MAIN_COMPILE_OPTIONS
    -Wall
    -Wextra
    -Wunsafe-loop-optimizations
    -Wdouble-promotion
    -Wstrict-prototypes
    -Werror=switch
)

macro(main_sources var) # list-var src-1...src-n
    set(${var} ${ARGN})
    list(TRANSFORM ${var} PREPEND "${MAIN_SRC_DIR}/")
endmacro()

function(exclude_basenames var excludes)
    set(filtered "")
    foreach(item ${${var}})
        get_filename_component(basename ${item} NAME)
        if (NOT ${basename} IN_LIST excludes)
            list(APPEND filtered ${item})
        endif()
    endforeach()
    set(${var} ${filtered} PARENT_SCOPE)
endfunction()

function(glob_except var pattern excludes)
    file(GLOB results ${pattern})
    list(LENGTH results count)
    if(count EQUAL 0)
        message(FATAL_ERROR "glob with pattern '${pattern}' returned no results")
    endif()
    exclude_basenames(results "${excludes}")
    set(${var} ${results} PARENT_SCOPE)
endfunction()

function(get_generated_files_dir output target_name)
    set(${output} ${CMAKE_CURRENT_BINARY_DIR}/${target_name} PARENT_SCOPE)
endfunction()

function(setup_executable exe name)
    get_generated_files_dir(generated_dir ${name})
    target_compile_options(${exe} PRIVATE ${MAIN_COMPILE_OPTIONS})
    target_include_directories(${exe} PRIVATE ${generated_dir} ${MAIN_INCLUDE_DIRS})
    target_compile_definitions(${exe} PRIVATE ${MAIN_DEFINITIONS} __TARGET__="${name}" ${name})
    # XXX: Don't make SETTINGS_GENERATED_C part of the build,
    # since it's compiled via #include in settings.c. This will
    # change once we move off PGs
    target_sources(${exe} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/${name}/${SETTINGS_GENERATED_H}")
    set_target_properties(${exe} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
    if(IS_RELEASE_BUILD)
        set_target_properties(${exe} PROPERTIES
            INTERPROCEDURAL_OPTIMIZATION ON
        )
    endif()
endfunction()

function(setup_firmware_target exe name)
    setup_executable(${exe} ${name})
    enable_settings(${exe} ${name})
    get_property(targets GLOBAL PROPERTY VALID_TARGETS)
    list(APPEND targets ${name})
    set_property(GLOBAL PROPERTY VALID_TARGETS "${targets}")
    setup_openocd(${exe} ${name})
    setup_svd(${exe} ${name})

    cmake_parse_arguments(args "SKIP_RELEASES" "" "" ${ARGN})
    if(args_SKIP_RELEASES)
        set_target_properties(${exe} ${name} PROPERTIES SKIP_RELEASES ON)
    endif()
endfunction()

function(exclude_from_all target)
    set_property(TARGET ${target} PROPERTY
        TARGET_MESSAGES OFF
        EXCLUDE_FROM_ALL 1
        EXCLUDE_FROM_DEFAULT_BUILD 1)
endfunction()

function(collect_targets)
    get_property(targets GLOBAL PROPERTY VALID_TARGETS)
    list(SORT targets)
    set(release_targets)
    foreach(target ${targets})
        get_target_property(skip_releases ${target} SKIP_RELEASES)
        if(NOT skip_releases)
            list(APPEND release_targets ${target})
        endif()
    endforeach()

    list(JOIN targets " " target_names)
    list(JOIN release_targets " " release_targets_names)
    set_property(GLOBAL PROPERTY RELEASE_TARGETS ${release_targets})

    set(list_target_name "targets")
    add_custom_target(${list_target_name}
        COMMAND ${CMAKE_COMMAND} -E echo "Valid targets: ${target_names}"
        COMMAND ${CMAKE_COMMAND} -E echo "Release targets: ${release_targets_names}"
    )
    exclude_from_all(${list_target_name})
    set(release_target_name "release")
    add_custom_target(${release_target_name}
        ${CMAKE_COMMAND} -E true
        DEPENDS ${release_targets}
    )
    list(LENGTH targets target_count)
    list(LENGTH release_targets release_target_count)
    message("-- ${target_count} targets (${release_target_count} for release) found for toolchain ${TOOLCHAIN}")
endfunction()
