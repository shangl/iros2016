#include "sensordata.hpp"

using namespace std;
using namespace arma;

namespace kukadu {

    SensorData::SensorData(std::string timeLabel, std::vector<std::string> jointPosLabels, std::vector<std::string> jointFrcLabels, std::vector<std::string> cartPosLabels,
               std::vector<std::string> cartForceAbsLabel, std::vector<std::string> cartFrcTrqLabels, arma::vec time, arma::mat jointPos, arma::mat jointFrc, arma::mat cartPos, arma::mat cartForceAbs, arma::mat cartFrcTrq) {

        this->jointPosLabels = jointPosLabels;
        this->jointFrcLabels = jointPosLabels;
        this->cartPosLabels = cartPosLabels;
        this->cartFrcTrqLabels = cartFrcTrqLabels;
        this->cartForceAbsLabel = cartForceAbsLabel;

        labels.push_back(timeLabel);
        if(jointPos.n_cols > 1) {
            for(int i = 0; i < jointPosLabels.size(); ++i)
                labels.push_back(jointPosLabels.at(i));
        } else
            this->jointPosLabels.clear();

        if(jointFrc.n_cols > 1) {
            for(int i = 0; i < jointFrcLabels.size(); ++i)
                labels.push_back(jointFrcLabels.at(i));
        } else
            this->jointFrcLabels.clear();

        if(cartPos.n_cols > 1) {
            for(int i = 0; i < cartPosLabels.size(); ++i)
                labels.push_back(cartPosLabels.at(i));
        } else
            this->cartPosLabels.clear();

        if(cartFrcTrq.n_cols > 1) {
            for(int i = 0; i < cartFrcTrqLabels.size(); ++i)
                labels.push_back(cartFrcTrqLabels.at(i));
        } else
            this->cartFrcTrqLabels.clear();

        if(cartForceAbs.n_cols > 1) {
            for(int i = 0; i < cartForceAbsLabel.size(); ++i)
                labels.push_back(cartForceAbsLabel.at(i));
        } else
            this->cartForceAbsLabel.clear();

        if((time.n_elem > 1)&&(jointPos.n_elem > 1))
            values = armaJoinRows(time, jointPos);
        else values = time;


        if(jointFrc.n_cols > 1)
            values = armaJoinRows(values, jointFrc);

        if(cartPos.n_cols > 1){
            values = armaJoinRows(values, cartPos);}

        if(cartFrcTrq.n_cols > 1)
            values = armaJoinRows(values, cartFrcTrq);

        if(cartForceAbs.n_cols > 1)
            values = armaJoinRows(values, cartForceAbs);

    }

    void SensorData::removeDuplicateTimes() {

        double currentTime = DBL_MAX;
        int vecSize = values.n_rows;
        for(int i = 0; i < vecSize; ++i) {
            double nextTime = getTime(i);
            if(currentTime == nextTime) {
                values.shed_row(i);
            }
        }

    }

    int SensorData::labelExists(std::string label) {

        for(int i = 0; i < labels.size(); ++i) {
            if(!label.compare(labels.at(i)))
                return i;
        }

        return -1;

    }

    arma::vec SensorData::getDataByIdx(int idx) {

        return vec(values.col(idx));

    }

    arma::vec SensorData::getTime() {

        return getDataByLabel("time");

    }

    arma::vec SensorData::getDataByLabel(std::string label) {

        return getDataByIdx(labelExists(label));

    }

    arma::mat SensorData::getRange(std::vector<std::string> indexes) {

        vector<int> intIndexes;
        for(int i = 0; i < indexes.size(); ++i) {

            int idx = labelExists(indexes.at(i));
            if(idx >= 0)
                intIndexes.push_back(idx);
            else
                return mat(1, 1);

        }

        return getRange(intIndexes);

    }

    arma::mat SensorData::getRange(std::vector<int> indexes) {

        mat retMat = getDataByIdx(indexes.at(0));
        for(int i = 1; i < indexes.size(); ++i) {
            vec currVec = getDataByIdx(indexes.at(i));
            retMat = join_rows(retMat, currVec);
        }

        return retMat;

    }

    arma::mat SensorData::getRange(int startIdx, int endIdx) {

        vector<int> rangeVec;
        for(; startIdx < endIdx; ++startIdx)
            rangeVec.push_back(startIdx);

        return getRange(rangeVec);

    }

    arma::vec SensorData::getTimes() {
        return values.col(0);
    }

    arma::mat SensorData::getJointPos() {
        return values.cols(1, 1 + jointPosLabels.size() - 1);
    }

    arma::mat SensorData::getJointForces() {
        return values.cols(1 + jointPosLabels.size(), 1 + jointPosLabels.size() + jointFrcLabels.size() - 1);
    }

    arma::mat SensorData::getCartPos() {
        return values.cols(1 + jointPosLabels.size() + jointFrcLabels.size(), 1 + jointPosLabels.size() + jointFrcLabels.size() + cartPosLabels.size() - 1);
    }

    arma::mat SensorData::getCartFrcTrqs() {
        return values.cols(1 + jointPosLabels.size() + jointFrcLabels.size() + cartPosLabels.size(), 1 + jointPosLabels.size() + jointFrcLabels.size() + cartPosLabels.size() + cartFrcTrqLabels.size() - 1);
    }

    double SensorData::getTime(int rowIdx) {
        return values(rowIdx, 0);
    }

    arma::vec SensorData::getJointPosRow(int rowIdx) {
        vec retVal(jointPosLabels.size());
        for(int i = 0; i < jointPosLabels.size(); ++i)
            retVal(i) = values(rowIdx, i + 1);
        return retVal;
    }

    arma::vec SensorData::getJointForcesRow(int rowIdx) {
        vec retVal(jointFrcLabels.size());
        for(int i = 0; i < jointFrcLabels.size(); ++i)
            retVal(i) = values(rowIdx, i + 1 + jointPosLabels.size());
        return retVal;
    }

    arma::vec SensorData::getCartPosRow(int rowIdx) {
        vec retVal(cartPosLabels.size());
        for(int i = 0; i < cartPosLabels.size(); ++i)
            retVal(i) = values(rowIdx, i + 1 + jointPosLabels.size() + jointFrcLabels.size());
        return retVal;
    }

    arma::vec SensorData::getCartFrcTrqsRow(int rowIdx) {
        vec retVal(cartFrcTrqLabels.size());
        for(int i = 0; i < cartFrcTrqLabels.size(); ++i)
            retVal(i) = values(rowIdx, i + 1 + jointPosLabels.size() + jointFrcLabels.size() + cartPosLabels.size());
        return retVal;
    }

}
