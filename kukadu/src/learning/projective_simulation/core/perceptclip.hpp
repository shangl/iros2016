#ifndef KUKADU_PERCEPTCLIP_H
#define KUKADU_PERCEPTCLIP_H

#include <string>

#include "clip.hpp"
#include "../../../types/kukadutypes.hpp"

namespace kukadu {

    class PerceptClip : public Clip {

    private:

        int perceptId;
        std::string label;

        void construct(int perceptId, std::string label);

    public:

        PerceptClip(int perceptId, std::string label, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string clipDimensionValues, int immunity);
        PerceptClip(int perceptId, std::string label, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, KUKADU_SHARED_PTR<std::vector<int> > clipDimensionValues, int immunity);

        int getPerceptId();

        std::string getLabel();
        std::string toString() const;

    };

}

#endif
