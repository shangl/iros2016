#include "perceptclip.hpp"

using namespace std;

namespace kukadu {

    PerceptClip::PerceptClip(int perceptId, std::string label, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string clipDimensionValues, int immunity) : Clip(0, generator, clipDimensionValues, immunity) {
        construct(perceptId, label);
    }

    PerceptClip::PerceptClip(int perceptId, std::string label, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipDimensionValues, int immunity) : Clip(0, generator, clipDimensionValues, immunity) {
        construct(perceptId, label);
    }

    void PerceptClip::construct(int perceptId, std::string label) {
        this->perceptId = perceptId;
        this->label = label;
    }

    int PerceptClip::getPerceptId() {
        return perceptId;
    }

    std::string PerceptClip::getLabel() {
        return label;
    }

    std::string PerceptClip::toString() const {
        return label;
    }

}
