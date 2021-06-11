//
// Created by amanda on 6/10/21.
//

#ifndef AUTODIFF_GP_FILE_IO_UTILS_H
#define AUTODIFF_GP_FILE_IO_UTILS_H

#include <vector>
#include <string>
#include <fstream>

namespace file_io {
    void writeCommaSeparatedStringsLineToFile(const std::vector<std::string> &strings, std::ofstream &file_stream) {
        for (size_t i = 0; i < strings.size(); i++) {
            file_stream << strings[i];
            if (i == (strings.size() - 1)) {
                file_stream << "\n";
            } else {
                file_stream << ", ";
            }
        }
    }
}

#endif //AUTODIFF_GP_FILE_IO_UTILS_H
