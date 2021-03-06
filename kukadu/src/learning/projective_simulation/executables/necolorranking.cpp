#include <ctime>
#include <vector>
#include <time.h>
#include <fstream>
#include <numeric>
#include <cstdlib>
#include <sstream>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <stdexcept>
#include <execinfo.h>

#include "../core/clip.hpp"
#include "../core/actionclip.hpp"
#include "../utils/tokenizer.hpp"
#include "../core/perceptclip.hpp"
#include "../core/psevaluator.hpp"
#include "../../../types/kukadutypes.hpp"
#include "../core/projectivesimulator.hpp"
#include "../visualization/treedrawer.hpp"
#include "../application/neverendingcolorreward.hpp"

#define ASY_WR_CONVERGENCE_TOL 0.01
#define ASY_WR_NUMBER_OF_AGENTS 2000
#define ASY_WR_NUMBER_OF_WALKS 100000
#define ASY_WR_NUMBER_OF_ACTIONS 2
#define ASY_WR_GAMMA 0.0
#define ASY_WR_MAX_NUMBER_OF_CLIPS 300
#define ASY_WR_CLIP_IMMUNITY 60
#define ASY_WR_NUMBER_OF_CATS 10
#define ASY_WR_START_NUMER_OF_CATS 2

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    int seed = time(NULL);
    int numberOfWalks = ASY_WR_NUMBER_OF_WALKS;
    int startNumberOfCats = ASY_WR_START_NUMER_OF_CATS;
    int maxNumberOfCats = ASY_WR_NUMBER_OF_CATS;
    int clipImmunity = ASY_WR_CLIP_IMMUNITY;
    int maxClipNumber = ASY_WR_MAX_NUMBER_OF_CLIPS;

    string outFilePath = "./necolornoranking.txt";

    char c = 0;
    opterr = 1;
    while((c = getopt(argc, args, "c:C:W:o:S:I:M:")) != -1) {
        switch(c) {
        case 'c':
            startNumberOfCats = atoi(optarg);
            break;
        case 'C':
            maxNumberOfCats = atoi(optarg);
            break;
        case 'W':
            numberOfWalks = atoi(optarg);
            break;
        case 'o':
            outFilePath = string(optarg);
            break;
        case 'S':
            seed = atoi(optarg);
            break;
        case 'I':
            clipImmunity = atoi(optarg);
            break;
        case 'M':
            maxClipNumber = atoi(optarg);
            break;
        }
    }

    cout << "options parsed" << endl;

    cout << "summary" << endl << "======================================" << endl;
    cout << "startNumberOfCats: " << startNumberOfCats << endl;
    cout << "maxNumberOfCats: " << maxNumberOfCats << endl;
    cout << "numberOfWalks: " << numberOfWalks << endl;
    cout << "outFilePath: " << outFilePath << endl;
    cout << "randomSeed: " << seed << endl;
    cout << "clipImmunity: " << clipImmunity << endl;
    cout << "maxClipNumber: " << maxClipNumber << endl;
    cout << "======================================" << endl << endl;

    KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator = KUKADU_SHARED_PTR<kukadu_mersenne_twister>(new kukadu_mersenne_twister(seed));

    ofstream outFile;
    outFile.open(outFilePath.c_str());

    outFile << "program mode is neverending color scenario without ranking" << endl;

    for(int numberOfCats = startNumberOfCats; numberOfCats <= maxNumberOfCats; ++numberOfCats) {

        outFile << endl << "number of cats: " << numberOfCats << endl;

        vector<double> asymptoticRewards;
        for(int i = 0; i < numberOfWalks; ++i)
            asymptoticRewards.push_back(0.0);

        KUKADU_SHARED_PTR<NeverendingColorReward> trafficReward;
        KUKADU_SHARED_PTR<ProjectiveSimulator> currentProjSim;

        outFile << "agent number " << 0 << " with " << numberOfCats << " cats" << endl;

        trafficReward = KUKADU_SHARED_PTR<NeverendingColorReward>(new NeverendingColorReward(generator, ASY_WR_NUMBER_OF_ACTIONS, numberOfCats, true));
        currentProjSim = KUKADU_SHARED_PTR<ProjectiveSimulator>(new ProjectiveSimulator(trafficReward, generator, ASY_WR_GAMMA, PS_USE_GEN, true));

        currentProjSim->setStandardImmunity(clipImmunity);
        currentProjSim->setMaxNumberOfClips(maxClipNumber);

        PSEvaluator::produceStatistics(currentProjSim, trafficReward, numberOfWalks, clipImmunity, NEVERENDINGCOLORREWARD_SUCCESSFUL_REWARD, outFile);

        currentProjSim.reset();
        trafficReward.reset();

    }

    return EXIT_SUCCESS;

}
