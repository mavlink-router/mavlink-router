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

/**
 * Load and parse multiple conf files, offering methods to extract the configuration options to user
 * structs.
 */
class ConfFile {
public:
    ConfFile()
        : _files(nullptr)
        , _sections(nullptr){};
    ~ConfFile();

    struct section_iter {
        const char *name;
        size_t name_len;
        void *ptr;
    };

    /**
     * Table with information on how to parse options from conf file.
     *
     * @param key The key to the option field
     * @param required If the field is required, extract_options() will fail if it is not found
     * @param parser_func Function to parse the value of the desired option, where val is the
     * pointer to value as string, val_len is size of the value, storage is a pointer to the area to
     * store the parsed value and storage_len is the length of the storage area.
     * @param storage.offset The offset from the beggining of the struct used in extract_options()
     * to store the values.
     * @param storage.len The length in bytes of the storage poited by @a storage_offset.
     */
    struct OptionsTable {
        const char *key;
        bool required;
        int (*parser_func)(const char *val, size_t val_len, void *storage, size_t storage_len);
        struct {
            off_t offset;
            size_t len;
        } storage;
    };

    /**
     * Load and parse a configuration file.
     *
     * Open the file using mmap and parse it, keeping an internal structure with pointers to conf
     * sections and options, in order to do a quick extraction using extract_options().
     * When an option in a section is already present before parse is called, the option is
     * overrided by the option from the new file. Other options from sections are not changed.
     * Files are kept open until release_all() is called or the ConfFile object is destructed.
     *
     * @param filename The conf filename
     * @return errno on IO or parsing errors or @c 0 if successful
     */
    int parse(const char *filename);

    /**
     * Release all opened files and internal structures from this ConfFile.
     */
    void release_all();

    /**
     * Extract options set in @a table from section @a section_name
     *
     * @param section_name Name of the section to extract options.
     * @param table Array of TableOption with information on which fields are going to be
     * extracted and how to extract them.
     * @param table_len The number of elements in @a table.
     * @param data A pointer to the struct that will be used to hold the extracted data.
     */
    int extract_options(const char *section_name, const OptionsTable table[], size_t table_len,
                        void *data);

    /**
     * Extract options set in @a table from section @a iter
     *
     * @param iter An section_iter filled by get_sections()
     * @param table Array of TableOption with information on which fields are going to be
     * extracted and how to extract them.
     * @param table_len The number of elements in @a table.
     * @param data A pointer to the struct that will be used to hold the extracted data.
     *
     * @see get_sections()
     */
    int extract_options(struct section_iter *iter, const OptionsTable table[], size_t table_len,
                        void *data);

    /**
     * Get next section name from iterator that matches the shell wildcard @a pattern.
     *
     * If iter.ptr is @c nullptr get_sections() will look for first occurence of @a pattern in
     * section list and fill iter.name with a pointer to section name, iter.name_len with the
     * size of section name and iter.ptr with an iterator to be used in consecutive calls.
     * If iter.ptr is filled with value from previous calls, get_section() will look for consective
     * occurences of @a pattern.
     *
     * @param pattern A shell wilcard pattern.
     * @param iter A instance of struct section_iter to be filled section data that matches @a
     * pattern.
     *
     * @return 0 if a section is found or -ENOENT if no section matches the pattern.
     */
    int get_sections(const char *pattern, struct section_iter *iter);

    // Helpers
    static int parse_bool(const char *val, size_t val_len, void *storage, size_t storage_len);
    static int parse_str_dup(const char *val, size_t val_len, void *storage, size_t storage_len);
    static int parse_log_mode(const char *val, size_t val_len, void *storage, size_t storage_len);
    static int parse_str_buf(const char *val, size_t val_len, void *storage, size_t storage_len);

#define DECLARE_PARSE_INT(_type) \
    static int parse_##_type(const char *val, size_t val_len, void *storage, size_t storage_len)
    DECLARE_PARSE_INT(i);
    DECLARE_PARSE_INT(ul);
    DECLARE_PARSE_INT(ull);
#undef DECLARE_PARSE_INT

private:
    struct conffile *_files;
    struct section *_sections;

    int _parse_file(const char *addr, size_t len, const char *filename);
    struct section *_find_section(const char *section_name, size_t len);
    struct config *_find_config(struct section *s, const char *key_name, size_t key_len);

    struct section *_add_section(const char *addr, size_t len, int line, const char *filename);
    int _add_config(struct section *s, const char *entry, size_t entry_len, const char *filename,
                    int line);
    void _trim(const char **str, size_t *len);
    int _extract_options_from_section(struct section *s, const OptionsTable table[],
                                      size_t table_len, void *data);
};

/*
 * Helper to create the parameter storage from OptionsTable, using _field  from _struct.
 */
#define OPTIONS_TABLE_STRUCT_FIELD(_struct, _field)        \
    {                                                      \
        offsetof(_struct, _field), sizeof(_struct::_field) \
    }
