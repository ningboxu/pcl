#include "point_cloud_processor.h"

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_file> <output_file>" << std::endl;
        return -1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];

    removeInvalidPoints(input_file, output_file);

    return 0;
}

