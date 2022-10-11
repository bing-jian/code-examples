#ifndef PROTO_UTILS_H_
#define PROTO_UTILS_H_

#include <google/protobuf/text_format.h>

#include <fstream>
#include <iostream>

namespace utils {

template <class T>
bool LoadProtoFromBinFile(const std::string& pb_file, T& obj) {
  std::ifstream infile(pb_file, std::ios::in | std::ios::binary);
  if (!infile) {
    std::cout << pb_file << ": File not found.  Creating a new file."
              << std::endl;
    return false;
  } else if (!obj.ParseFromIstream(&infile)) {
    infile.close();
    std::cerr << "Failed to parse pb file." << std::endl;
    return false;
  }
  infile.close();
  return true;
}

template <class T>
bool LoadProtoFromTxtFile(const std::string& txt_file, T& obj) {
  std::ifstream infile(txt_file);
  if (infile.is_open()) {
    std::string content((std::istreambuf_iterator<char>(infile)),
                        (std::istreambuf_iterator<char>()));
    infile.close();
    google::protobuf::TextFormat::ParseFromString(content, &obj);
    return true;
  }
  return false;
}

template <class T>
bool LoadProtoFromFile(const std::string& fname, T& obj) {
  std::string::size_type idx = fname.rfind('.');
  if (idx != std::string::npos) {
    std::string extension = fname.substr(idx + 1);
    if ("txt" == extension) {
      return LoadProtoFromTxtFile(fname, obj);
    }
  }
  if (!LoadProtoFromTxtFile(fname + ".txt", obj)) {
    return LoadProtoFromBinFile(fname, obj);
  }
  return false;
}

template <class T>
bool SaveProtoToBinFile(const std::string& pb_file, T& obj) {
  std::ofstream ofs(pb_file,
                    std::ios::out | std::ios::trunc | std::ios::binary);
  obj.SerializeToOstream(&ofs);
  ofs.close();
  std::ofstream(pb_file + ".txt") << obj.Utf8DebugString() << std::endl;
  return true;
}

}  // namespace utils

#endif  // PROTO_UTILS_H_
