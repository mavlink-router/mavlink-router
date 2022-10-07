#pragma once

#include <vector>
#include <string>
#include <stdint.h>
#include <fcntl.h>
#include <fstream>
#include <regex>


class InformationMessage {
private:
    struct _packed_ Header {
        uint16_t msg_size;
        char msg_type;
        uint8_t key_size;
    };

public:
    static std::vector<char> encode(const std::string key, const std::string value) {
        const std::string prepended_key = "char[" + std::to_string(value.size()) + "] " + key;
        const std::string full_payload = prepended_key + value;
        Header h;
        h.msg_size = full_payload.size() + 1;
        h.msg_type = 'I';
        h.key_size = prepended_key.size();
        std::vector<char> result;
        std::copy((char*)(&h), (char*)(&h)+sizeof(h), std::back_inserter(result));
        std::copy(full_payload.begin(), full_payload.end(), std::back_inserter(result));
        return result;
    }
};


bool get_aos_version(std::string &version) {
    std::ifstream infile("/etc/release.conf");
    if (!infile.good()) {
        infile.close();
        return false;
    }

    const std::regex regex{"^Version: (.*)"};
    std::string line;
    
    while (std::getline(infile, line)) {
        std::smatch match;
        if (std::regex_search(line, match, regex)) {
            version = match[1];
            infile.close();
            return true;
        }
    }
    infile.close();
    return false;
};




bool write_meta_information(int file, bool &meta_written) {

    std::string version;
    if (!get_aos_version(version)) {
        log_warning("Could not read AuterionOS version file!");
        meta_written = true;
        return true;
    }
    const auto messages = InformationMessage::encode("aos_version", version);


    ssize_t total_length = messages.size();
    ssize_t written = 0;
    while (written < total_length) {
        ssize_t r = write(file, messages.data() + written, total_length - written);
        if (r == 0 || (r == -1 && errno == EAGAIN))
            return true;
        if (r < 0) {
            log_error("Unable to write to ULog file: (%m)");
            return false;
        }
        written += r;
    }
    meta_written = true;
    return true;
}
