#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>

namespace kukadu {

    std::vector<std::string> getFilesInDir(std::string directory);
    std::vector<std::string> filterByPrefix(std::vector<std::string>& vec, std::string prefix);

}

#endif // UTILS_H
