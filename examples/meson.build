arm_authorizer = executable('arm-authorizer',
  ['arm-authorizer.cpp'],
  include_directories: [mavlink_inc, src_inc],
  install: true,
)

heartbeat_print = executable('heartbeat-print',
  ['heartbeat-print.cpp'],
  include_directories: [mavlink_inc, src_inc],
  install: true,
)

px4_offboard_mode = executable('px4-offboard-mode',
  ['px4-offboard-mode.cpp'],
  include_directories: [mavlink_inc, src_inc],
  link_with: libcommon_private,
  install: true,
)
