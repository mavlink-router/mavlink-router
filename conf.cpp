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

#include "conf.h"

#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "log.h"

struct config {
    char *key;
    char *value;
    struct config *next;
};

struct section {
    char *name;
    struct config *configs;
    struct config *current_config;
    struct section *next;
};

ConfFile::~ConfFile()
{
    struct section *section;
    struct config *config;

    section = _sections;
    while (section) {
        struct section *tmp = section;

        config = section->configs;
        while (config) {
            struct config *tmp_config = config;
            free(config->key);
            // config->value is on the same allocation as config->key, no need to free it
            config = config->next;
            free(tmp_config);
        }

        free(section->name);
        section = section->next;
        free(tmp);
    }
}

int ConfFile::parse_file()
{
    FILE *file;
    char *entry = NULL;
    int line = 0, ret = 0;
    size_t n = 0;
    ssize_t read;

    assert(_filename);

    file = fopen(_filename, "re");
    if (!file) {
        log_error_errno(errno, "Could not open conf file '%s' (%m)", _filename);
        return -EIO;
    }

    while ((read = getline(&entry, &n, file)) >= 0) {
        line++;

        _trim(&entry);

        switch (entry[0]) {
        case ';':
        case '\0':
            // Discards comment or blank line
            free(entry);
            break;
        case '[':
            ret = _add_section(entry, line);
            if (ret < 0) {
                goto end;
            }
            break;
        default:
            ret = _add_config(entry, line);
            if (ret < 0) {
                goto end;
            }
        }

        entry = NULL;
    }

    _current_section = nullptr;

end:
    free(entry);
    fclose(file);
    return ret;
}

const char* ConfFile::next_section(bool reset)
{
    if (reset) {
        _current_section = _sections;
        return _current_section->name;
    }

    if (!_current_section->next)
        return NULL;

    _current_section = _current_section->next;
    return _current_section->name;
}

int ConfFile::_add_section(char *entry, int line)
{
    struct section *section;

    char *end = strchr(entry, ']');
    if (!end) {
        log_error("On file %s: Line %d: Unfinished section name. Expected ']'", _filename, line);
        return -EINVAL;
    }

    // So section name is the string between '[]'
    *entry = ' ';
    *end = '\0';
    _trim(&entry);

    // Ensure section is not duplicated. If it is,
    // just make parser add on known section
    section = _sections;
    while (section) {
        if (!strcasecmp(section->name, entry)) {
            _current_section = section;
            free(entry);
            return 0;
        }
        section = section->next;
    }

    // Section is new.
    section = (struct section *)calloc(1, sizeof(struct section));
    null_check(section, -ENOMEM);

    section->name = entry;
    section->next = _sections;
    _sections = section;

    _current_section = section;

    return 0;
}

int ConfFile::_add_config(char *entry, int line)
{
    char *equal_pos;
    struct config *config;

    if (!(equal_pos = strchr(entry, '='))) {
        log_error("On file %s: Line %d: Missing '=' on config", _filename, line);
        return -EINVAL;
    }

    if (equal_pos == entry) {
        log_error("On file %s: Line %d: Missing name on config", _filename, line);
        return -EINVAL;
    }

    if (equal_pos == (entry + strlen(entry) - 1)) {
        log_error("On file %s: Line %d: Missing value on config", _filename, line);
        return -EINVAL;
    }

    config = (struct config *)malloc(sizeof(struct config));
    null_check(config, -ENOMEM);

    config->key = entry;
    config->value = equal_pos + 1;
    *equal_pos = '\0';

    _trim(&config->key);
    _trim(&config->value);

    config->next = _current_section->configs;
    _current_section->configs = config;

    return 0;
}

const char *ConfFile::next_from_section(const char *section_name, const char *key)
{
    struct section *section;
    struct config *config;

    if (!_current_section)
        _current_section = _sections;

    if (!_current_section) {
        return NULL;
    }

    if (strcasecmp(section_name, _current_section->name)) {
        section = _sections;

        while (section) {
            if (!strcasecmp(section->name, section_name)) {
                _current_section = section;
                _current_section->current_config = NULL;
                break;
            }
            section = section->next;
        }

        if (!section) {
            return NULL;
        }
    }

    if (!_current_section->current_config || strcasecmp(_current_section->current_config->key, key)) {
        config = _current_section->configs;
    } else {
        config = _current_section->current_config->next;
    }

    while (config) {
        if (!strcasecmp(config->key, key)) {
            _current_section->current_config = config;
            return config->value;
        }
        config = config->next;
    }

    return NULL;
}

void ConfFile::_trim(char **str)
{
    char *s = *str;
    char *end;

    while (isspace(*s))
        s++;

    end = s + strlen(s);
    while (end != s && isspace(*(end - 1)))
        end--;

    *end = '\0';

    memmove(*str, s, end - s + 1);
}
