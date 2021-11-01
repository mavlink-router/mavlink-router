subdir('common')

exe_mavlink_router = executable('mavlink-routerd',
        [
         'autolog.cpp',
         'binlog.cpp',
         'endpoint.cpp',
         'git_version.cpp',
         'logendpoint.cpp',
         'main.cpp',
         'mainloop.cpp',
         'pollable.cpp',
         'timeout.cpp',
         'ulog.cpp',
         version_h,
        ],
        include_directories: [src_inc, mavlink_inc],
        dependencies: [
                dep_math,
                dep_rt,
                dep_thread,
        ],
        link_with: libcommon_private,
        install: true,
)
