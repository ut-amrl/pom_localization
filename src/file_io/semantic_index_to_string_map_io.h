//
// Created by amanda on 2/5/22.
//

#ifndef AUTODIFF_GP_SEMANTIC_INDEX_TO_STRING_MAP_IO_H
#define AUTODIFF_GP_SEMANTIC_INDEX_TO_STRING_MAP_IO_H

#include <stdint.h>
#include <fstream>
#include <unordered_map>
#include <sstream>
#include <file_io/file_io_utils.h>

namespace file_io {

    void writeSemanticIndexToStringMapHeaderToFile(std::ofstream &file_stream) {
        writeCommaSeparatedStringsLineToFile({"semantic_index", "semantic_class_string"}, file_stream);
    }


    void writeSemanticIndexToStringEntryLineToFile(
            const std::pair<unsigned short, std::string> &semantic_index_to_string_map,
            std::ofstream &file_stream) {
        file_stream << semantic_index_to_string_map.first << ", " << semantic_index_to_string_map.second << "\n";
    }

    void writeSemanticIndexToStringMapToFile(const std::string &file_name,
                                             const std::unordered_map<unsigned short, std::string> &semantic_index_to_string_map) {
        std::ofstream csv_file(file_name, std::ios::trunc);
        writeSemanticIndexToStringMapHeaderToFile(csv_file);
        for (const auto &semantic_index_to_string_entry : semantic_index_to_string_map) {
            writeSemanticIndexToStringEntryLineToFile(semantic_index_to_string_entry, csv_file);
        }

        csv_file.close();
    }

    void readSemanticIndexToStringEntryLine(const std::string &line_in_file,
                                            std::pair<unsigned short, std::string> &semantic_index_to_string_entry) {
        std::stringstream ss(line_in_file);
        std::string substr;

        getline(ss, substr, ',');
        std::istringstream stream_index(substr);
        unsigned short semantic_index;
        stream_index >> semantic_index;

        std::string semantic_string;
        getline(ss, semantic_string, ',');

        semantic_index_to_string_entry = std::make_pair(semantic_index, semantic_string);
    }

    void readSemanticIndexToStringMapFromFile(const std::string &file_name,
                                              std::unordered_map<unsigned short, std::string> &semantic_index_to_string_map) {
        LOG(INFO) << "Reading semantic index map from file " << file_name;
        std::ifstream file_obj(file_name);
        std::string line;
        bool first_line = true;
        while (std::getline(file_obj, line)) {
            if (first_line) {
                first_line = false;
                continue;
            }
            std::pair<unsigned short, std::string> semantic_index_to_string_entry;
            readSemanticIndexToStringEntryLine(line, semantic_index_to_string_entry);
            semantic_index_to_string_map[semantic_index_to_string_entry.first] = semantic_index_to_string_entry.second;
        }
        if (first_line) {
            LOG(ERROR) << "File " << file_name << " was empty. Exiting";
            exit(1);
        }
    }
}

#endif //AUTODIFF_GP_SEMANTIC_INDEX_TO_STRING_MAP_IO_H
