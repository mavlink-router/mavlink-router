project(
        'mavlink-router',
        'cpp', 'c',
        version: '1',
        license: 'Apache 2.0',
        default_options: [
                'cpp_std=gnu++11',
        ],
)

add_project_arguments('-D_GNU_SOURCE', language: 'cpp')
add_project_arguments('-D_GNU_SOURCE', language: 'c')
add_project_arguments('-DPACKAGE_VERSION=' + meson.project_version(), language: 'cpp')
add_project_arguments('-DPACKAGE_VERSION=' + meson.project_version(), language: 'c')
cxx = meson.get_compiler('cpp')
cc = meson.get_compiler('c')
dep_math = cxx.find_library('m')
dep_thread = dependency('threads')

mavgen =  find_program('modules/mavlink/pymavlink/tools/mavgen.py')
gen_mavlink_headers = custom_target('gen_mavlink_headers',
                                    input : ['modules/mavlink/message_definitions/v1.0/ardupilotmega.xml'],
                                    output : ['mavlink'],
                                    command : [mavgen, '--lang', 'C', '--wire-protocol', '2.0', '-o', '@OUTPUT@', '@INPUT@'])

libcommon_private = static_library(
        'common-private',
        [
         'src/common/xtermios.cpp',
         'src/common/conf_file.cpp',
         'src/common/log.cpp',
         'src/common/util.c',
        ],
        install: false,
        pic: true,
        include_directories: include_directories('src'),
)

exe_mavlink_router = executable(
        'mavlink-routerd',
        [
         'src/mavlink-router/autolog.cpp',
         'src/mavlink-router/binlog.cpp',
         'src/mavlink-router/endpoint.cpp',
         'src/mavlink-router/logendpoint.cpp',
         'src/mavlink-router/main.cpp',
         'src/mavlink-router/mainloop.cpp',
         'src/mavlink-router/pollable.cpp',
         'src/mavlink-router/timeout.cpp',
         'src/mavlink-router/ulog.cpp',
        ],
        gen_mavlink_headers,
        link_with: libcommon_private,
        dependencies: [
                dep_math,
                dep_thread,
        ],
        install: true,
        include_directories: [include_directories('mavlink', 'mavlink/ardupilotmega', is_build_only: true),
                              include_directories('src')]
)
