common_sources = [
  'common/conf_file.cpp',
  'common/log.cpp',
  'common/util.c',
  'common/xtermios.cpp',
]

common_inc = include_directories('common')

libcommon_private = static_library(
  'common-private',
  common_sources,
  include_directories: common_inc,
  install:false,
)

router_sources = [
  'mavlink-router/main.cpp',
  'mavlink-router/autolog.cpp',
  'mavlink-router/timeout.cpp',
  'mavlink-router/binlog.cpp',
  'mavlink-router/mainloop.cpp',
  'mavlink-router/ulog.cpp',
  'mavlink-router/endpoint.cpp',
  'mavlink-router/mainloop_test.cpp',
  'mavlink-router/logendpoint.cpp',
  'mavlink-router/pollable.cpp',
]

router_inc = include_directories('mavlink-router')

#executable('mavlink-routerd',
  #[common_sources, router_sources],
  #include_directories: [config_inc, mavlink_inc, common_inc, router_inc],
  #dependencies: [gtest_dep, realtime_dep, aio_dep],
  #install: true,
#)
executable('mavlink-routerd',
  router_sources,
  include_directories: [config_inc, mavlink_inc, router_inc],
  dependencies: [gtest_dep, realtime_dep, aio_dep],
  link_with: libcommon_private,
  install: true,
)

#util = library('util',
  #'common/util.c',
  #include_directories: [common_inc],
#)
