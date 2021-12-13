#!/bin/bash

# Wrapper script to fix the header-filter path output by meson to be a regex
# expression (replace '.' in a path with it's escaped value '\.').

arguments=$@

# replace .. with \.\.
fixed_args=${arguments//../\\.\\.}


# disable glob expansion and run original script
set -f
run-clang-tidy $fixed_args
