src_platform = []
if host_machine.system() != 'darwin'
        src_platform = [
         'xtermios.cpp', 
         'serial_linux.cpp'
        ]
else
        src_platform = [
         'serial_macos.cpp'
        ]
endif 

libcommon_private = static_library('common-private',
        src_platform + [
         'conf_file.cpp',
         'log.cpp',
         'util.cpp',
         
        ],
        pic: true,
        install: false,
)
