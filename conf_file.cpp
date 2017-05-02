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

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <fnmatch.h>
#include <limits.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "conf_file.h"
#include "log.h"
#include "util.h"

#define MAX_SECTION_NAME 100

struct config {
    const char *key;
    const char *value;
    size_t key_len, value_len;
    const char *filename;
    int line;
    struct config *next;
};

struct section {
    const char *name;
    size_t len;
    struct config *configs;
    struct section *next;
};

struct conffile {
    void *addr;
    size_t len;
    char *filename;
    struct conffile *next;
};

ConfFile::~ConfFile()
{
    release_all();
}

int ConfFile::parse(const char *filename)
{
    int fd, ret = 0;
    void *addr;
    struct stat fstat;

    assert(filename);

    fd = open(filename, O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        log_error("Could not open conf file '%s' (%m)", filename);
        return -errno;
    }

    if (stat(filename, &fstat) < 0) {
        ret = -errno;
        goto error;
    }

    addr = mmap(0, (size_t)fstat.st_size, PROT_READ, MAP_PRIVATE, fd, 0);
    if (addr == MAP_FAILED) {
        ret = -errno;
        goto error;
    }

    close(fd);

    _files = new conffile{addr, (size_t)fstat.st_size, strdup(filename), _files};

    ret = _parse_file((char *)addr, (size_t)fstat.st_size, _files->filename);
    if (ret < 0) {
        log_error("Conf file parcially parsed. Configuration keys defined before the error were "
                  "included");
        return ret;
    }

    return 0;
error:
    close(fd);
    return ret;
}

int ConfFile::_parse_file(const char *addr, size_t len, const char *filename)
{
    int line = 0, ret = 0;
    char *end;
    size_t line_len, full_line_len;

    struct section *s = nullptr;
    while ((end = (char *)memchr((void *)addr, '\n', len))) {
        line++;
        full_line_len = line_len = end - addr;

        // Make sure all casts to int are safe. No need to support options or sections
        // larger than INT_MAX
        if (line_len > INT_MAX)
            return -EINVAL;
        _trim(&addr, &line_len);
        if (!line_len)
            goto next;

        switch (addr[0]) {
        case ';':
        case '#':
            goto next;
        case '[':
            s = _add_section(addr, line_len, line, filename);
            if (!s)
                return -EINVAL;
            break;
        default:
            if (!s) {
                log_error(
                    "On file %s: Line %d: Expected section before the definition of first option.",
                    filename, line);
                return -EINVAL;
            }

            ret = _add_config(s, addr, line_len, filename, line);
            if (ret < 0)
                return ret;
            break;
        }

    next:
        len = len - full_line_len - 1;
        addr = end + 1;
    }

    return ret;
}

struct section *ConfFile::_add_section(const char *addr, size_t len, int line, const char *filename)
{
    const char *end, *p;
    struct section *s;
    bool spaces;

    end = (char *)memchr(addr, ']', len);
    if (!end) {
        log_error("On file %s: Line %d: Unfinished section name. Expected ']'", filename, line);
        return nullptr;
    }

    if ((size_t)(end - addr) < len - 1) {
        log_error("On file %s: Line %d: Unexpected characters after session name", filename, line);
        return nullptr;
    }

    if (isspace(*(end - 1)) || isspace(*(addr + 1))) {
        log_error(
            "On file %s: Line %d: Trailing or leading spaces are not allowed in section name.",
            filename, line);
        return nullptr;
    }

    spaces = false;
    for (p = addr; p < end; p++) {
        if (isspace(*p)) {
            if (*p != ' ') {
                log_error("On file %s: Line %d: Whitespaces different from single space are not "
                          "allowed in section names.",
                          filename, line);
                return nullptr;
            }
            if (spaces) {
                log_error("On file %s: Line %d: Invalid section name. No spaces in subsection name "
                          "are allowed.",
                          filename, line);
                return nullptr;
            }
            spaces = true;
        }
    }

    s = _find_section(addr + 1, len - 2);
    if (s)
        return s;

    if (len - 2 > MAX_SECTION_NAME) {
        log_error("Max supported section name is %d", MAX_SECTION_NAME);
        return nullptr;
    }

    log_debug("ConfFile: Adding section '%.*s'", (int)(len - 2), addr + 1);
    _sections = new section{addr + 1, len - 2, nullptr, _sections};
    return _sections;
}

int ConfFile::_add_config(struct section *s, const char *entry, size_t entry_len,
                          const char *filename, int line)
{
    const char *equal_pos;
    struct config *c;
    const char *key, *value;
    size_t key_len, value_len;

    if (!(equal_pos = (char *)memchr(entry, '=', entry_len))) {
        log_error("On file %s: Line %d: Missing '=' on config", filename, line);
        return -EINVAL;
    }

    if (equal_pos == entry) {
        log_error("On file %s: Line %d: Missing name on config", filename, line);
        return -EINVAL;
    }

    if (equal_pos == (entry + entry_len - 1)) {
        log_error("On file %s: Line %d: Missing value on config", filename, line);
        return -EINVAL;
    }

    key = entry;
    key_len = equal_pos - entry;
    value = equal_pos + 1;
    value_len = entry_len - key_len - 1;

    _trim(&key, &key_len);
    _trim(&value, &value_len);

    c = _find_config(s, key, key_len);
    if (c) {
        c->value = value;
        c->value_len = value_len;
        c->filename = filename;
        c->line = line;
    } else {
        s->configs = c = new config{key, value, key_len, value_len, filename, line, s->configs};
    }

    return 0;
}

void ConfFile::_trim(const char **str, size_t *len)
{
    const char *s = *str;
    const char *end = s + *len;

    while (isspace(*s) && s < end)
        s++;

    while (end > s && isspace(*(end - 1)))
        end--;

    *len = end - s;
    *str = s;
}

void ConfFile::release_all()
{
    struct conffile *f;
    struct section *s;
    struct config *c;

    while (_files) {
        f = _files;
        _files = _files->next;

        munmap(f->addr, f->len);
        free(f->filename);
        delete f;
    }

    while (_sections) {
        s = _sections;
        _sections = _sections->next;
        while (s->configs) {
            c = s->configs;
            s->configs = s->configs->next;
            delete c;
        }

        delete s;
    }
}

static void print_filenames(struct section *s)
{
    const char *files[128] = {};
    int files_size = 0;
    struct config *c;

    if (Log::get_max_level() < Log::Level::ERROR)
        return;

    for (c = s->configs; c; c = c->next) {
        bool found = false;
        for (int i = 0; i < files_size; i++) {
            if (streq(files[i], c->filename)) {
                found = true;
                break;
            }
        }
        if (!found) {
            if (files_size == (sizeof(files) / sizeof(*files))) {
                log_error("\tand others.");
                break;
            }
            log_error("\t%s", c->filename);
            files[files_size++] = c->filename;
        }
    }
}

int ConfFile::_extract_options_from_section(struct section *s, const OptionsTable table[],
                                            size_t table_len, void *data)
{
    struct config *c;
    int ret;
    size_t i;
    void *storage;

    for (i = 0; i < table_len; i++) {
        c = _find_config(s, table[i].key, strlen(table[i].key));
        if (!c) {
            if (table[i].required) {
                log_error("Required field '%s' not found in section '%.*s', defined in:",
                          table[i].key, (int)s->len, s->name);
                print_filenames(s);
                return -ENOENT;
            }
            continue;
        }
        storage = (void *)((char *)data + table[i].storage.offset);
        ret = table[i].parser_func(c->value, c->value_len, storage, table[i].storage.len);
        if (ret < 0) {
            log_error("On file %s: Line %d: Invalid value '%.*s' for field '%s'", c->filename,
                      c->line, (int)c->value_len, c->value, table[i].key);
            return ret;
        }
    }

    return 0;
}

int ConfFile::extract_options(const char *section_name, const OptionsTable table[],
                              size_t table_len, void *data)
{
    struct section *s;

    assert(section_name);
    assert(table);

    s = _find_section(section_name, strlen(section_name));
    if (!s) {
        // It is only a problem when there is are required fields
        for (size_t i = 0; i < table_len; i++) {
            if (table[i].required) {
                log_error("Section '%s' not found and field '%s' is required.", section_name,
                          table[i].key);
                return -ENOENT;
            }
        }

        return 0;
    }

    return _extract_options_from_section(s, table, table_len, data);
}

int ConfFile::extract_options(struct section_iter *iter, const OptionsTable table[],
                              size_t table_len, void *data)
{
    assert(iter);
    assert(table);

    return _extract_options_from_section((struct section *)iter->ptr, table, table_len, data);
}

struct config *ConfFile::_find_config(struct section *s, const char *key, size_t key_len)
{
    struct config *c;

    for (c = s->configs; c; c = c->next) {
        if (c->key_len == key_len && strncasecmp(key, (char *)c->key, key_len) == 0)
            return c;
    }

    return nullptr;
}

struct section *ConfFile::_find_section(const char *section_name, size_t len)
{
    struct section *s;

    for (s = _sections; s; s = s->next) {
        if (memcaseeq(section_name, len, s->name, s->len))
            return s;
    }

    return nullptr;
}

int ConfFile::get_sections(const char *pattern, struct section_iter *iter)
{
    struct section *s;
    char section_name[MAX_SECTION_NAME];

    assert(pattern);
    assert(iter);

    if (iter->ptr == nullptr)
        s = _sections;
    else
        s = ((section *)iter->ptr)->next;

    for (; s; s = s->next) {
        memcpy(section_name, s->name, s->len);
        section_name[s->len] = 0;
        if (fnmatch(pattern, section_name, FNM_CASEFOLD) == 0) {
            iter->name = s->name;
            iter->name_len = s->len;
            iter->ptr = s;
            return 0;
        }
    }

    iter->ptr = nullptr;
    iter->name = nullptr;
    iter->name_len = 0;
    return -ENOENT;
}

int ConfFile::parse_str_dup(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    char **ptr = (char **)storage;

    assert(val);
    assert(storage);
    assert(val_len);

    if (storage_len < sizeof(char *))
        return -ENOBUFS;

    *ptr = strndup(val, val_len);
    if (!*ptr)
        return -errno;
    return 0;
}

int ConfFile::parse_str_buf(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    assert(val);
    assert(storage);
    assert(storage_len);
    assert(val_len);

    if (storage_len - 1 < val_len)
        return -ENOBUFS;

    memcpy(storage, val, val_len);
    ((char *)storage)[val_len] = '\0';

    return 0;
}

int ConfFile::parse_bool(const char *val, size_t val_len, void *storage, size_t storage_len)
{
    bool *b = (bool *)storage;
    int ival, ret;

    assert(val);
    assert(storage);
    assert(val_len);

    if (storage_len < sizeof(bool))
        return -ENOBUFS;

    if (memcaseeq("true", 4, val, val_len))
        *b = true;
    else if (memcaseeq("false", 5, val, val_len))
        *b = false;
    else {
        ret = parse_i(val, val_len, &ival, sizeof(ival));
        if (ret < 0)
            return ret;
        *b = !!ival;
    }

    return 0;
}

#define DEFINE_PARSE_INT(_name, _type, _func)                                   \
    int ConfFile::parse_##_name(const char *val, size_t val_len, void *storage, \
                                size_t storage_len)                             \
    {                                                                           \
        char *str;                                                              \
                                                                                \
        assert(val);                                                            \
        assert(storage);                                                        \
        assert(val_len);                                                        \
        if (storage_len < sizeof(_type))                                        \
            return -ENOBUFS;                                                    \
                                                                                \
        str = strndupa(val, val_len);                                           \
        return _func(str, (_type *)storage);                                    \
    }

DEFINE_PARSE_INT(i, int, safe_atoi)
DEFINE_PARSE_INT(ul, unsigned long, safe_atoul)
DEFINE_PARSE_INT(ull, unsigned long long, safe_atoull)
