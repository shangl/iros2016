#include "psevaluator.hpp"

#define PSEVAL_BUFFER_SIZE 500

using namespace std;
using namespace kukadu;

PSEvaluator::PSEvaluator() {
}

std::pair<std::vector<int>, std::vector<double> > PSEvaluator::evaluateStatistics(std::vector<string> inputFiles, std::vector<int> inputPos) {

    vector<double> retSuccess;
    vector<double> walkSuccess;
    int usedFilesCount = 0;
    int lowestMaxWalkIdx = INT32_MAX;
    char memblock[PSEVAL_BUFFER_SIZE + 1];

    for(int streamNum = 0; streamNum < inputFiles.size(); ++streamNum) {
        string lastLine = "";
        string dataLine = "";
        string currentStreamFile = inputFiles.at(streamNum);
        ifstream currentStream;
        currentStream.open(currentStreamFile.c_str());
        currentStream.seekg(inputPos.at(streamNum));
        memblock[0] = '\0';
        size_t endPos = 0;

        while(currentStream && (endPos = (lastLine + string(memblock)).find("number of cats:")) == string::npos) {

            lastLine = string(memblock);
            dataLine += lastLine;

            currentStream.read(memblock, PSEVAL_BUFFER_SIZE);
            memblock[PSEVAL_BUFFER_SIZE] = '\0';

        }

        dataLine += string(memblock).substr(0, endPos);
        currentStream.seekg((endPos - lastLine.size()) - PSEVAL_BUFFER_SIZE, currentStream.cur);

        int walkIdx = 0;
        for(int currentSignIdx = 0; currentSignIdx < dataLine.size(); currentSignIdx += 2) {

            int currentSign = dataLine[currentSignIdx];
            for(int i = 0; i < 8; ++i) {

                int currentWalk = ((currentSign & 128) == 128) ? 1 : 0;
                currentSign = currentSign << 1;

                if(streamNum == 0)
                    walkSuccess.push_back(currentWalk);
                else if(walkIdx < walkSuccess.size()) {
                    walkSuccess.at(walkIdx) += currentWalk;
                }

                ++walkIdx;

            }

        }

        if(walkIdx > 0)
            ++usedFilesCount;
        else
            walkIdx = INT32_MAX;

        if(walkIdx < lowestMaxWalkIdx)
            lowestMaxWalkIdx = walkIdx;

        inputPos.at(streamNum) = currentStream.tellg();
        currentStream.close();

    }

    if(lowestMaxWalkIdx < INT32_MAX)
        for(int i = 0; i < lowestMaxWalkIdx; ++i) {
            retSuccess.push_back(walkSuccess.at(i) / usedFilesCount);
        }

    return std::pair<std::vector<int>, std::vector<double> >(inputPos, retSuccess);

}

vector<double> PSEvaluator::evaluateStatistics(std::vector<KUKADU_SHARED_PTR<std::ifstream> >& inputStreams) {

    vector<double> retSuccess;
    vector<double> walkSuccess;
    int usedFilesCount = 0;
    int lowestMaxWalkIdx = INT32_MAX;
    char memblock[PSEVAL_BUFFER_SIZE + 1];

    for(int streamNum = 0; streamNum < inputStreams.size(); ++streamNum) {
        string lastLine = "";
        string dataLine = "";
        KUKADU_SHARED_PTR<ifstream> currentStream = inputStreams.at(streamNum);
        memblock[0] = '\0';
        size_t endPos = 0;

        while(*currentStream && (endPos = (lastLine + string(memblock)).find("number of cats:")) == string::npos) {
            lastLine = string(memblock);
            dataLine += lastLine;
            currentStream->read(memblock, PSEVAL_BUFFER_SIZE);
            memblock[PSEVAL_BUFFER_SIZE] = '\0';
        }

        dataLine += string(memblock).substr(0, endPos);
        currentStream->seekg((endPos - lastLine.size()) - PSEVAL_BUFFER_SIZE, currentStream->cur);

        int walkIdx = 0;
        for(int currentSignIdx = 0; currentSignIdx < dataLine.size(); currentSignIdx += 2) {

            int currentSign = dataLine[currentSignIdx];
            for(int i = 0; i < 8; ++i) {

                int currentWalk = ((currentSign & 128) == 128) ? 1 : 0;
                currentSign = currentSign << 1;

                if(streamNum == 0)
                    walkSuccess.push_back(currentWalk);
                else if(walkIdx < walkSuccess.size()) {
                    walkSuccess.at(walkIdx) += currentWalk;
                }

                ++walkIdx;

            }

        }

        if(walkIdx > 0)
            ++usedFilesCount;
        else
            walkIdx = INT32_MAX;

        if(walkIdx < lowestMaxWalkIdx)
            lowestMaxWalkIdx = walkIdx;

    }

    for(int i = 0; i < lowestMaxWalkIdx; ++i) {
        retSuccess.push_back(walkSuccess.at(i) / usedFilesCount);
    }

    return retSuccess;

}

void PSEvaluator::produceStatistics(KUKADU_SHARED_PTR<ProjectiveSimulator> ps, KUKADU_SHARED_PTR<Reward> reward, int numberOfWalks, int clipImmunity, int rewardValue, std::ostream& outStream) {

    char currentOutput = 0;
    int fieldsInCurrentOutput = 0;
    for(int j = 0; j < numberOfWalks; ++j, ++fieldsInCurrentOutput) {

        KUKADU_SHARED_PTR<PerceptClip> nextClip = reward->generateNextPerceptClip(clipImmunity);
        ps->generalize(nextClip);
        ps->performRandomWalk();

        pair<bool, double> rewRes = ps->performRewarding();
        int lastResult = rewRes.second / rewardValue;
        if(!rewRes.first) {
            if(fieldsInCurrentOutput < 8) {
                currentOutput = currentOutput << 1;
            } else {
                outStream << currentOutput << ",";
                fieldsInCurrentOutput = 0;
                currentOutput = 0;
            }
            currentOutput = currentOutput | lastResult;
        } else {
            --fieldsInCurrentOutput;
        }

    }

}
