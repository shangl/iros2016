#ifndef KUKADU_PROJECTIVESIMULATOR_H
#define KUKADU_PROJECTIVESIMULATOR_H

#include <set>
#include <map>
#include <limits>
#include <vector>
#include <utility>
#include <algorithm>

#include "clip.hpp"
#include "reward.hpp"
#include "actionclip.hpp"
#include "perceptclip.hpp"
#include "../../../types/kukadutypes.hpp"

#define PS_PRINT_DEBUG_INFO 0
#define PS_PRINT_RANKING_DEBUG_INFO 0
#define PS_USE_ORIGINAL 1
#define PS_USE_GEN 2

#define PS_MAX_NUMBER_OF_CLIPS 1000

namespace kukadu {

    struct clip_compare {

        bool operator() (KUKADU_SHARED_PTR<Clip> const& lhs, KUKADU_SHARED_PTR<Clip> const& rhs) {

            /*
             *This object determines the order of the elements in the container:
             *it is a function pointer or a function object that takes two arguments of the same
             *type as the container elements, and returns true if the first argument is considered
             *to go before the second in the strict weak ordering it defines, and false otherwise.
             *
             *Two elements of a set are considered equivalent if key_comp returns false reflexively
             *(i.e., no matter the order in which the elements are passed as arguments)
            */

            // if same reference --> always the same
            if(lhs == rhs || *lhs == *rhs)
                return false;

            if(*lhs < *rhs)
                return true;

            return false;

        }

    };

    class ProjectiveSimulator {

    private:

        bool useRanking;
        bool useBoredom;
        bool doTraining;

        int levels;
        int operationMode;
        int immunityThresh;
        int maxNumberOfClips;

        double gamma;
        double boredom;

        std::vector<std::pair<double, KUKADU_SHARED_PTR<Clip> > > rankVec;

        KUKADU_SHARED_PTR<Reward> reward;
        KUKADU_SHARED_PTR<Clip> lastClipBeforeAction;
        KUKADU_SHARED_PTR<ActionClip> lastActionClip;
        KUKADU_SHARED_PTR<PerceptClip> lastPerceptClip;
        KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator;
        KUKADU_SHARED_PTR<PerceptClip> lastGeneralizedPercept;
        KUKADU_SHARED_PTR<std::vector<int> > intermediateHops;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > actionClips;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > perceptClips;
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > > clipLayers;

        kukadu_uniform_distribution intDist;

        void cleanByRank();
        void printRankVec();
        void computeRankVec();
        void construct(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking);

        bool fileExists(const std::string filePath);

        int getIdVecLevel(KUKADU_SHARED_PTR<std::vector<int> > idVec);

        double computeBoredem(KUKADU_SHARED_PTR<Clip> clip);

        KUKADU_SHARED_PTR<Clip> findClipByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec);
        KUKADU_SHARED_PTR<Clip> findClipInLevelByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec, int level);

    public:

        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file);
        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking);
        ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                            KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > network,
                            double gamma, int operationMode, bool useRanking);
        ~ProjectiveSimulator();

        void printWeights();
        void setBoredom(double boredom);
        void setTrainingMode(bool doTraining);
        void setStandardImmunity(int immunity);
        void setMaxNumberOfClips(int maxNumberOfClips);
        void connectNewClip(KUKADU_SHARED_PTR<Clip> conClip);
        void eliminateClip(KUKADU_SHARED_PTR<Clip> currClip);
        void generalize(KUKADU_SHARED_PTR<PerceptClip> nextClip);
        void fillClipLayersFromNetwork(KUKADU_SHARED_PTR<Clip> cl);

        int getClipCount();
        int getStandardImmunity();

        std::pair<bool, double> performRewarding();

        KUKADU_SHARED_PTR<std::vector<int> > getIntermediateHopIdx();

        KUKADU_SHARED_PTR<ActionClip> performRandomWalk();

        // returns list of clips that have to be created
        KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > createNewClips(KUKADU_SHARED_PTR<PerceptClip> nextClip);
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > getPerceptClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > getActionClips();
        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > > getClipLayers();

        void storePS(std::string targetFile);

    };

}

#endif
