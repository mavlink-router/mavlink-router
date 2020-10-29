project('mavlink-router', 'c', 'cpp')

r = run_command('git', 'rev-parse', '--short', 'HEAD', '2>/dev/null')
version_str = r.stdout().strip()

conf_data = configuration_data()
conf_data.set('version', version_str)
conf_data.set('package', 'mavlink-router')

configure_file(
  input: 'config.h.meson.in',
  output: 'config.h',
  configuration: conf_data,
)
config_inc = include_directories('.')

python3 = find_program('python3')
mavgen = 'modules/mavlink/pymavlink/tools/mavgen.py'
infile = 'modules/mavlink/message_definitions/v1.0/ardupilotmega.xml'
run_command([python3, mavgen, '-o', 'include/mavlink', '--lang', 'C',
  '--wire-protocol', '2.0', infile])

mavlink_inc = include_directories('include/mavlink/ardupilotmega/')

cc = meson.get_compiler('c')
gtest_dep = dependency('gtest')
aio_dep = cc.find_library('aio')
realtime_dep = cc.find_library('rt')

subdir('src')
src_inc = include_directories('src') # for use in examples
subdir('examples')
