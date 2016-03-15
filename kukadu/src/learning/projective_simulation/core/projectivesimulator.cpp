#include "projectivesimulator.hpp"

#include <utility>
#include <fstream>
#include <fstream>
#include <iostream>

#include "../utils/tokenizer.hpp"
#include "../../../types/kukadutypes.hpp"

using namespace std;

namespace kukadu {

    ProjectiveSimulator::ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, std::string file) {

        this->reward = reward;
        this->generator = generator;

        this->lastActionClip;
        this->lastPerceptClip;

        actionClips = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > >(new std::vector<KUKADU_SHARED_PTR<ActionClip> >());
        perceptClips = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > >(new std::vector<KUKADU_SHARED_PTR<PerceptClip> >());
        clipLayers = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > >(new std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >());

        intermediateHops = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());

        string line = "";
        ifstream inputFile;
        inputFile.open(file.c_str());

        // check version
        getline(inputFile, line);
        if(!line.compare("V1.0")) {

            // ignore "general properties" line
            getline(inputFile, line);

            // operation mode
            getline(inputFile, line);
            Tokenizer tok(line, "=");
            tok.next(); operationMode = atoi(tok.next().c_str());

            // use ranking?
            getline(inputFile, line);
            tok = Tokenizer(line, "=");
            tok.next(); useRanking = (atoi(tok.next().c_str()))?true:false;

            // gamma
            getline(inputFile, line);
            tok = Tokenizer(line, "=");
            tok.next(); gamma = atof(tok.next().c_str());

            // max number of clips (ignored without ranking)
            getline(inputFile, line);
            tok = Tokenizer(line, "=");
            tok.next(); maxNumberOfClips = atoi(tok.next().c_str());

            // immunity threshhold (ignored without ranking)
            getline(inputFile, line);
            tok = Tokenizer(line, "=");
            tok.next(); immunityThresh = atoi(tok.next().c_str());

            // levels
            getline(inputFile, line);
            tok = Tokenizer(line, "=");
            tok.next(); levels = atoi(tok.next().c_str());

            for(int i = 0; i < levels; ++i)
                clipLayers->push_back(KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> >(new set<KUKADU_SHARED_PTR<Clip>, clip_compare>()));

            getline(inputFile, line);
            getline(inputFile, line);

            // load the clips
            int currentLayer = 0;
            bool isFirstPercept = true;
            int perceptDimensionality = 0;
            while(getline(inputFile, line) && line.compare("")) {

                // check if its a layer line
                tok = Tokenizer(line, "=");
                string nextToken = tok.next();
                if(!nextToken.compare("layer")) {

                    string layerString = tok.next();
                    currentLayer = atoi(layerString.c_str());

                } else {

                    // it is no t a layer line (must be a clip line
                    tok = Tokenizer(line, ";");
                    KUKADU_SHARED_PTR<Clip> nextClip;

                    // first line is the id vector
                    if(currentLayer == 0) {

                        string idVec = tok.next();
                        string label = tok.next();
                        int immunity = atoi(tok.next().c_str());
                        KUKADU_SHARED_PTR<PerceptClip> pc = KUKADU_SHARED_PTR<PerceptClip>(new PerceptClip(atoi(tok.next().c_str()), label, generator, idVec, immunity));
                        nextClip = pc;

                        if(isFirstPercept) {

                            isFirstPercept = false;
                            perceptDimensionality = pc->getDimensionality();

                        }

                        perceptClips->push_back(pc);

                    } else if(currentLayer == CLIP_H_LEVEL_FINAL) {

                        // id vec is not useful
                        tok.next();
                        string label = tok.next();
                        int immunity = atoi(tok.next().c_str());
                        KUKADU_SHARED_PTR<ActionClip> ac = KUKADU_SHARED_PTR<ActionClip>(new ActionClip(atoi(tok.next().c_str()), perceptDimensionality, label, generator));
                        nextClip = ac;
                        actionClips->push_back(ac);

                    } else {

                        string idVec = tok.next();
                        int immunity = atoi(tok.next().c_str());
                        KUKADU_SHARED_PTR<Clip> c = nextClip = KUKADU_SHARED_PTR<Clip>(new Clip(currentLayer, generator, idVec, immunity));

                    }

                    int clipLevel = currentLayer;
                    if(clipLevel != CLIP_H_LEVEL_FINAL)
                        clipLayers->at(clipLevel)->insert(nextClip);
                    else
                        clipLayers->at(clipLayers->size() - 1)->insert(nextClip);

                }

            }

            getline(inputFile, line);

            // load the clips
            currentLayer = 0;
            KUKADU_SHARED_PTR<Clip> currentParent;
            std::vector<double> newChildrenWeights;
            KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > > newChildren = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > >(new std::vector<KUKADU_SHARED_PTR<Clip> >());
            while(getline(inputFile, line)) {

                // check if its a layer line
                tok = Tokenizer(line, "=");
                string nextToken = tok.next();
                if(!nextToken.compare("layer")) {

                    string layerString = tok.next();
                    currentLayer = atoi(layerString.c_str());

                } else {

                    // a new parent will be provided next
                    if(!line.compare("")) {

                        currentParent->setChildren(newChildren, newChildrenWeights);
                        currentParent.reset();
                        newChildrenWeights.clear();
                        newChildren = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > >(new std::vector<KUKADU_SHARED_PTR<Clip> >());

                    }
                    // its a new parent clip
                    else if(line.find(':') != string::npos) {

                        currentParent = findClipByIdVec(Clip::getIdVectorFromString(line.substr(0, line.size() - 1)));

                    } else if(line.find(';') != string::npos) {

                        // it must be a new child clip
                        tok = Tokenizer(line, ";");
                        string idVecString = tok.next();
                        double connectionWeight = atof(tok.next().c_str());
                        KUKADU_SHARED_PTR<Clip> currentChild = findClipByIdVec(Clip::getIdVectorFromString(idVecString));
                        newChildren->push_back(currentChild);
                        newChildrenWeights.push_back(connectionWeight);

                    }

                }

            }

        } else {
            throw KukaduException("PS file version cannot be handled");
        }

    }

    int ProjectiveSimulator::getIdVecLevel(KUKADU_SHARED_PTR<std::vector<int> > idVec) {

        int retCount = 0;

        for(int i = 0; i < idVec->size(); ++i) {
            int val = idVec->at(i);
            if(val == CLIP_H_HASH_VAL)
                retCount++;
        }

        return retCount;

    }

    KUKADU_SHARED_PTR<Clip> ProjectiveSimulator::findClipInLevelByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec, int level) {
        KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currentLayer = clipLayers->at(level);

        std::set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator it;
        for(it = currentLayer->begin(); it != currentLayer->end(); ++it) {
            KUKADU_SHARED_PTR<Clip> c = *it;
            KUKADU_SHARED_PTR<vector<int> > clipDim = c->getClipDimensions();
            if(Clip::compareIdVecs(clipDim, idVec)) {
                return c;
            }
        }

        return KUKADU_SHARED_PTR<Clip>();
    }

    KUKADU_SHARED_PTR<Clip> ProjectiveSimulator::findClipByIdVec(KUKADU_SHARED_PTR<std::vector<int> > idVec) {

        if(operationMode == PS_USE_ORIGINAL) {

            for(int level = 0; level < clipLayers->size() - 1; ++level) {
                KUKADU_SHARED_PTR<Clip> retVal = findClipInLevelByIdVec(idVec, level);
                if(!retVal)
                    return retVal;
            }

        } else if(operationMode == PS_USE_GEN) {
            // this works only for generalization where the number of wildcards says something about the level
            int level = getIdVecLevel(idVec);
            return findClipInLevelByIdVec(idVec, level);
        }

        for(int i = 0; i < actionClips->size(); ++i) {
            KUKADU_SHARED_PTR<Clip> ac = actionClips->at(i);
            KUKADU_SHARED_PTR<vector<int> > clipDim = ac->getClipDimensions();
            if(Clip::compareIdVecs(clipDim, idVec)) {
                return ac;
            }
        }

        return KUKADU_SHARED_PTR<Clip>();

    }

    void ProjectiveSimulator::setBoredom(double boredom) {
        if(boredom <= 0.0) {
            this->boredom = 0.0;
            this->useBoredom = false;
        } else {
            this->boredom = boredom;
            this->useBoredom = true;
        }
    }

    void ProjectiveSimulator::setTrainingMode(bool doTraining) {
        this->doTraining = doTraining;
    }

    void ProjectiveSimulator::construct(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking) {

        this->boredom = 0.0;
        this->doTraining = true;
        this->useBoredom = false;

        this->useRanking = useRanking;
        this->operationMode = operationMode;
        this->gamma = gamma;
        intermediateHops = KUKADU_SHARED_PTR<vector<int> >(new vector<int>());

        this->lastActionClip.reset();
        this->lastPerceptClip.reset();

        this->reward = reward;
        this->maxNumberOfClips = PS_MAX_NUMBER_OF_CLIPS;

        this->generator = generator;
        intDist = kukadu_uniform_distribution(0, perceptClips->size() - 1);

        clipLayers = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > >(new std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >());

        for(int i = 0; i < levels + 1; ++i)
            clipLayers->push_back(KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> >(new set<KUKADU_SHARED_PTR<Clip>, clip_compare>()));

        clipLayers->at(0)->insert(perceptClips->begin(), perceptClips->end());
        clipLayers->at(clipLayers->size() - 1)->insert(actionClips->begin(), actionClips->end());

        lastGeneralizedPercept.reset();

    }

    ProjectiveSimulator::ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator, double gamma, int operationMode, bool useRanking) {

        this->perceptClips = reward->generatePerceptClips();
        this->actionClips = reward->generateActionClips();

        levels = 0;
        if(operationMode == PS_USE_ORIGINAL)
            levels = 1;
        else if(operationMode == PS_USE_GEN)
            // + 1 for the (#, #, #, ...) layer
            levels = reward->getDimensionality() + 1;

        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > > clipActionClips = KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<Clip> > >(new std::vector<KUKADU_SHARED_PTR<Clip> >());
        for(int i = 0; i < actionClips->size(); ++i) {
            KUKADU_SHARED_PTR<Clip> t = actionClips->at(i);
            clipActionClips->push_back(KUKADU_DYNAMIC_POINTER_CAST<Clip>(t));
        }

        for(int i = 0; i < perceptClips->size(); ++i) {
            KUKADU_SHARED_PTR<Clip> currentClip = perceptClips->at(i);
            currentClip->setChildren(clipActionClips);
        }

        construct(reward, generator, gamma, operationMode, useRanking);

    }

    ProjectiveSimulator::ProjectiveSimulator(KUKADU_SHARED_PTR<Reward> reward, KUKADU_SHARED_PTR<kukadu_mersenne_twister> generator,
                        KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > network,
                        double gamma, int operationMode, bool useRanking) {

        this->perceptClips = network;

        // set levels and action clips
        // walk down to last percept
        levels = 0;
        KUKADU_SHARED_PTR<Clip> lastClip;
        lastClip.reset();
        KUKADU_SHARED_PTR<Clip> currClip = perceptClips->at(0);
        while(currClip->getSubClipCount()) {
            lastClip = currClip;
            currClip = currClip->getSubClipByIdx(0);
            ++levels;
        }

        actionClips = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<ActionClip> > >(new vector<KUKADU_SHARED_PTR<ActionClip> >());
        for(int i = 0; i < lastClip->getSubClipCount(); ++i) {
            KUKADU_SHARED_PTR<ActionClip> nextActClip = KUKADU_DYNAMIC_POINTER_CAST<ActionClip>(lastClip->getSubClipByIdx(i));
            actionClips->push_back(nextActClip);
        }

        construct(reward, generator, gamma, operationMode, useRanking);

        for(int i = 0; i < perceptClips->size(); ++i) {
            KUKADU_SHARED_PTR<PerceptClip> pc = perceptClips->at(i);
            fillClipLayersFromNetwork(pc);
        }

    }

    void ProjectiveSimulator::fillClipLayersFromNetwork(KUKADU_SHARED_PTR<Clip> cl) {

        int level = cl->getLevel();

        if(level != CLIP_H_LEVEL_FINAL) {
            clipLayers->at(level)->insert(cl);
            for(int i = 0; i < cl->getSubClipCount(); ++i)
                fillClipLayersFromNetwork(cl->getSubClipByIdx(i));
        } else {
            clipLayers->at(clipLayers->size() - 1)->insert(cl);
        }


    }

    KUKADU_SHARED_PTR<std::vector<int> > ProjectiveSimulator::getIntermediateHopIdx() {
        return intermediateHops;
    }

    ProjectiveSimulator::~ProjectiveSimulator() {

    }

    void ProjectiveSimulator::eliminateClip(KUKADU_SHARED_PTR<Clip> currClip) {

        int level = currClip->getLevel();
        KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLayer = clipLayers->at(level);
        currLayer->erase(currClip);
        set<KUKADU_SHARED_PTR<Clip> > parents = std::set<KUKADU_SHARED_PTR<Clip> >(currClip->getParents()->begin(), currClip->getParents()->end());

        set<KUKADU_SHARED_PTR<Clip> >::iterator it;
        for(it = parents.begin(); it != parents.end(); ++it) {
            KUKADU_SHARED_PTR<Clip> parent = *it;
            parent->removeSubClip(currClip);
        }

        if(level == 0 && operationMode != PS_USE_ORIGINAL)
            perceptClips->erase(std::find(perceptClips->begin(), perceptClips->end() + 1, currClip));

        currClip->removeAllSubClips();

    }

    int ProjectiveSimulator::getStandardImmunity() {
        return immunityThresh;
    }

    void ProjectiveSimulator::cleanByRank() {

        int clipNumber = rankVec.size();

        int alreadyDeleted = 0;
        int toDelete = clipNumber - maxNumberOfClips;

        if(clipNumber > maxNumberOfClips) {

            for(int i = 0; i < clipNumber; ++i) {

                KUKADU_SHARED_PTR<Clip> currClip = rankVec.at(i).second;
                if(!currClip->isImmune()) {

                    ++alreadyDeleted;
                    eliminateClip(currClip);
                    rankVec.erase(rankVec.begin() + i);
                    --i;
                    --clipNumber;

                    if(alreadyDeleted >= toDelete)
                        return;

                }
            }
        }

        // if too many clips were immune, then start deleting the rest
        if(alreadyDeleted < toDelete) {

            for(int i = 0; alreadyDeleted < toDelete; ++i, ++alreadyDeleted) {

                KUKADU_SHARED_PTR<Clip> currClip = rankVec.at(i).second;
                eliminateClip(currClip);

                rankVec.erase(rankVec.begin() + i);
                --i;

            }

        }

    }

    void ProjectiveSimulator::setStandardImmunity(int immunity) {
        this->immunityThresh = immunity;
    }

    void ProjectiveSimulator::setMaxNumberOfClips(int maxNumberOfClips) {
        this->maxNumberOfClips = maxNumberOfClips;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<ActionClip> > > ProjectiveSimulator::getActionClips() {
        return actionClips;
    }

    KUKADU_SHARED_PTR<ActionClip> ProjectiveSimulator::performRandomWalk() {

        lastClipBeforeAction.reset();
        intermediateHops->clear();
        KUKADU_SHARED_PTR<Clip> previousClip;
        KUKADU_SHARED_PTR<Clip> currentClip;

        if(operationMode == PS_USE_GEN) {
            if(!lastGeneralizedPercept) {
                cerr << "(ProjectiveSimulator) you have to generalize before you walk" << endl;
                throw KukaduException("(ProjectiveSimulator) you have to generalize before you walk");
            } else {
                currentClip = lastGeneralizedPercept;
            }
        } else if(operationMode == PS_USE_ORIGINAL) {
            currentClip = reward->generateNextPerceptClip(immunityThresh);
        }

        std::vector<KUKADU_SHARED_PTR<PerceptClip> >::iterator it = std::find(perceptClips->begin(), perceptClips->end() + 1, currentClip);
        int previousIdx = it - perceptClips->begin();
        lastPerceptClip = KUKADU_DYNAMIC_POINTER_CAST<PerceptClip>(currentClip);

        while(previousClip != currentClip) {
            intermediateHops->push_back(previousIdx);
            pair<int, KUKADU_SHARED_PTR<Clip> > nextHop;
            lastClipBeforeAction = previousClip;
            previousClip = currentClip;
            nextHop = currentClip->jumpNextRandom();
            previousIdx = nextHop.first;
            currentClip = nextHop.second;

        }

        lastActionClip = KUKADU_DYNAMIC_POINTER_CAST<ActionClip>(currentClip);
        return lastActionClip;

    }

    double ProjectiveSimulator::computeBoredem(KUKADU_SHARED_PTR<Clip> clip) {

        double entropy = clip->computeSubEntropy();
        double numberOfSubclips = clip->getSubClipCount();

        // cout << "entropy: " << entropy << "; log2: " << log2(numberOfSubclips) << "; " << "; boredomConst: " << boredom << "; ";

        // b * (1 - H / H_max) = b * (1 - H / log2(N))
        double clipBoredom = boredom * (1.0 - entropy / log2(numberOfSubclips));

        return clipBoredom;

    }

    pair<bool, double> ProjectiveSimulator::performRewarding() {

        int beingBored = 0;
        if(useBoredom) {

            double boredomScore = computeBoredem(lastClipBeforeAction);
            vector<double> boredomDistWeights;
            boredomDistWeights.push_back(boredomScore);
            boredomDistWeights.push_back(1 - boredomScore);
            KUKADU_DISCRETE_DISTRIBUTION<int> boredomDist = KUKADU_DISCRETE_DISTRIBUTION<int>(boredomDistWeights.begin(), boredomDistWeights.end());
            beingBored =  1 - boredomDist(*generator);

        }

        double computedReward = 0.0;
        if(!beingBored) {

            computedReward = reward->computeReward(lastPerceptClip, lastActionClip);

            if(doTraining) {

                KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel;
                for(int i = 0; i < clipLayers->size(); ++i) {

                    currLevel = clipLayers->at(i);

                    KUKADU_SHARED_PTR<Clip> currClip;
                    set<KUKADU_SHARED_PTR<Clip> >::iterator currIt;
                    for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {
                        KUKADU_SHARED_PTR<Clip> currClip = *currIt;
                        currClip->updateWeights(computedReward, gamma);

                        // decrease immunity
                        if(useRanking)
                            currClip->decreaseImmunity();
                    }

                }

                if(useRanking) {

                    computeRankVec();
                    cleanByRank();

                }
            }

        }

        return make_pair(beingBored, computedReward);

    }

    void ProjectiveSimulator::generalize(KUKADU_SHARED_PTR<PerceptClip> nextClip) {

        lastGeneralizedPercept = nextClip;

        KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > toConnect = createNewClips(nextClip);

        // first connect everything...
        set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator it;
        for(it = toConnect->begin(); it != toConnect->end(); ++it) {
            KUKADU_SHARED_PTR<Clip> con = *it;
            if(PS_PRINT_DEBUG_INFO)
                cout << "calling connect function for " << *con << endl;

            if(useRanking)
                con->setPreviousRank();

            connectNewClip(con);
        }

        toConnect.reset();

    }

    void ProjectiveSimulator::printWeights() {

        int level = 0;

        std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >::iterator it;
        for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {

            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel = *it;
            cout << "clips on layer " << level << endl << "=========================" << endl;

            set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
            for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {
                KUKADU_SHARED_PTR<Clip> currClip = *currIt;
                int subClipCount = currClip->getSubClipCount();
                for(int i = 0; i < subClipCount; ++i) {
                    cout << *currClip << " --> " << *currClip->getSubClipByIdx(i) << " with weight " << currClip->getWeightByIdx(i) << endl;
                }
            }

            ++level;

        }

    }

    void ProjectiveSimulator::connectNewClip(KUKADU_SHARED_PTR<Clip> conClip) {

        int currentLevel = 0;

        std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >::iterator it;
        for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {
            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel = *it;
            if(currLevel->size() > 0 && conClip->getLevel() != currentLevel) {

                set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
                for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {

                    KUKADU_SHARED_PTR<Clip> currClip = *currIt;

                    // if they are compatible
                    if(currClip->isCompatibleSubclip(conClip)) {
                        currClip->addSubClip(conClip, CLIP_H_STD_WEIGHT);
                        conClip->addParent(currClip);
                    } else if(conClip->isCompatibleSubclip(currClip)) {
                        conClip->addSubClip(currClip, CLIP_H_STD_WEIGHT);
                        currClip->addParent(conClip);
                    } else {
                        // dont connect, they are not compatible
                    }

                }

            }

            ++currentLevel;
        }

    }

    KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > ProjectiveSimulator::createNewClips(KUKADU_SHARED_PTR<PerceptClip> newClip) {

        KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > conClips = KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> >(new set<KUKADU_SHARED_PTR<Clip>, clip_compare>());

        // insert new percept clip
        pair<set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator, bool> inserted = clipLayers->at(0)->insert(newClip);

        // check if new clip was already there
        if(inserted.second) {

            // if it wasnt there yet, add it to the new clips that have to be freshly connected
            conClips->insert(newClip);
            perceptClips->push_back(KUKADU_DYNAMIC_POINTER_CAST<PerceptClip>(newClip));

            // check in each level, if there will be new clips
            std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >::iterator it;
            for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {

                KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel = *it;
                KUKADU_SHARED_PTR<Clip> firstClipOnLevel;
                firstClipOnLevel.reset();
                if(currLevel->size())
                    firstClipOnLevel = *(currLevel->begin());
                if(currLevel->size() && firstClipOnLevel->getLevel() != CLIP_H_LEVEL_FINAL) {

                    set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
                    for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {

                        KUKADU_SHARED_PTR<Clip> currClip = *currIt;

                        // create new clip that gets generated as a cascade
                        KUKADU_SHARED_PTR<Clip> nextClip = currClip->compareClip(newClip);

                        // check if already there and insert it if not
                        pair<set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator, bool> inserted2 = clipLayers->at(nextClip->getLevel())->insert(nextClip);

                        // if new && not already there
                        if(inserted2.second && nextClip != currClip) {

                            nextClip = *inserted2.first;

                            // insert to the new set of clips that should be connected
                            pair<set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator, bool> inserted3 = conClips->insert(nextClip);


                        }
                        // if same clip is there already, but new instance, then delete the instance
                        else if(newClip != nextClip) {
                            nextClip.reset();
                        }

                    }

                }
                else break;
            }

        }

        return conClips;

    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > > > ProjectiveSimulator::getClipLayers() {
        return clipLayers;
    }

    KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<PerceptClip> > > ProjectiveSimulator::getPerceptClips() {
        return perceptClips;
    }

    bool compareRanks(std::pair<double, KUKADU_SHARED_PTR<Clip> > p1, std::pair<double, KUKADU_SHARED_PTR<Clip> > p2) {
        return (p1.first < p2.first);
    }

    void ProjectiveSimulator::computeRankVec() {

        rankVec.clear();
        for(int i = 0; i < clipLayers->size() - 1; i++) {

            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > currLevel = clipLayers->at(i);

            set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
            for(currIt = currLevel->begin(); currIt != currLevel->end(); ++currIt) {

                KUKADU_SHARED_PTR<Clip> currClip = *currIt;
                rankVec.push_back(pair<double, KUKADU_SHARED_PTR<Clip> >(currClip->computeRank(), currClip));

            }
        }
        std::sort(rankVec.begin(), rankVec.end(), compareRanks);

        if(PS_PRINT_RANKING_DEBUG_INFO)
            printRankVec();

    }

    int ProjectiveSimulator::getClipCount() {
        return rankVec.size();
    }

    void ProjectiveSimulator::printRankVec() {

        cout << rankVec.size() << endl;

    }

    bool ProjectiveSimulator::fileExists(const std::string filePath) {
        ifstream f(filePath.c_str());
        if (f.good()) {
            f.close();
            return true;
        } else {
            f.close();
            return false;
        }
    }

    void ProjectiveSimulator::storePS(std::string targetFile) {

        int deleteFile = 0;
        if(fileExists(targetFile)) {
            cout << "(ProjectiveSimulator) file already exists. do you want to overwrite it? (0 = no / 1 = yes)" << endl;
            cin >> deleteFile;
            if(deleteFile == 1) {
                // do nothing and overwrite it later
            } else {
                cout << "(ProjectiveSimulator) no overwriting selected. skip storing" << endl;
                return;
            }
        }

        ofstream outFile;
        outFile.open(targetFile.c_str(), ios::trunc);

        outFile << "V1.0" << endl;
        outFile << "general properties" << endl;
        outFile << "operationMode=" << operationMode << endl;
        outFile << "ranking=" << useRanking << endl;
        outFile << "gamma=" << gamma << endl;
        outFile << "maxNumberOfClips=" << maxNumberOfClips << endl;
        outFile << "immunityThresh=" << immunityThresh << endl;
        outFile << "levels=" << clipLayers->size() << endl;
        outFile << endl;

        outFile << "clips" << endl;
        std::vector<KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > >::iterator it;
        for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {

            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > layer = *it;
            if(layer->size() > 0) {
                int currentLevel = (*layer->begin())->getLevel();
                outFile << "layer=" << currentLevel << endl;

                set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
                for(currIt = layer->begin(); currIt != layer->end(); ++currIt) {

                    KUKADU_SHARED_PTR<Clip> cClip = *currIt;
                    outFile << cClip->getIdVecString() << ";" << *cClip << ";" << cClip->getCurrentImmunity();
                    if(currentLevel == 0) {
                        KUKADU_SHARED_PTR<PerceptClip> cpc = KUKADU_DYNAMIC_POINTER_CAST<PerceptClip>(cClip);
                        outFile << ";" << cpc->getPerceptId();
                    } else if(currentLevel == CLIP_H_LEVEL_FINAL) {
                        KUKADU_SHARED_PTR<ActionClip> cpc = KUKADU_DYNAMIC_POINTER_CAST<ActionClip>(cClip);
                        outFile << ";" << cpc->getActionId();
                    }
                    outFile << endl;
                }
            }
        }
        outFile << endl;

        outFile << "connections" << endl;
        for(it = clipLayers->begin(); it != clipLayers->end(); ++it) {

            KUKADU_SHARED_PTR<set<KUKADU_SHARED_PTR<Clip>, clip_compare> > layer = *it;
            if(layer->size() > 0) {
                int currentLevel = (*layer->begin())->getLevel();
                if(currentLevel != CLIP_H_LEVEL_FINAL) {
                    outFile << "layer=" << currentLevel << endl;

                    set<KUKADU_SHARED_PTR<Clip>, clip_compare>::iterator currIt;
                    for(currIt = layer->begin(); currIt != layer->end(); ++currIt) {

                        KUKADU_SHARED_PTR<Clip> cClip = *currIt;
                        int subClipCount = cClip->getSubClipCount();
                        outFile << cClip->getIdVecString() << ":" << endl;
                        for(int j = 0;  j < subClipCount; ++j) {
                            outFile << cClip->getSubClipByIdx(j)->getIdVecString() << ";" << cClip->getWeightByIdx(j) << endl;
                        }
                        outFile << endl;
                    }
                }
            }
        }

    }

}
