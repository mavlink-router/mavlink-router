/*
 * This file is part of the MAVLink Router project
 *
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

class ConfFile {
public:
    ConfFile(const char *filename)
        : _filename{filename} {};

    ~ConfFile();

    int parse_file();
    const char *next_from_section(const char *section_name, const char *key);
    const char *first_section();
    const char *next_section();

private:
    const char *_filename;
    struct section *_sections = nullptr;
    struct section *_current_section = nullptr;

    int _add_section(char *entry, int line);
    int _add_config(char *entry, int line);
    void _trim(char *str);
};
