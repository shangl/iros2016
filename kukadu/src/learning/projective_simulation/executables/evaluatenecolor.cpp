#include <ctime>
#include <vector>
#include <cstdlib>
#include <numeric>
#include <sstream>
#include <fstream>
#include <sstream>
#include <iostream>
#include <getopt.h>
#include <stdexcept>
#include <execinfo.h>

#include "../core/clip.hpp"
#include "../utils/utils.hpp"
#include "../core/actionclip.hpp"
#include "../utils/tokenizer.hpp"
#include "../core/perceptclip.hpp"
#include "../core/psevaluator.hpp"
#include "../core/projectivesimulator.hpp"
#include "../visualization/treedrawer.hpp"
#include "../application/neverendingcolorreward.hpp"

using namespace std;
using namespace kukadu;

int main(int argc, char** args) {

    string filePrefix = "noRank";
    string inputFolder = "/home/c7031109/data/studium/physik/semester_10/bak_thesis/results/really_final_results/rankNA1500/projsimnew3/bin/";
    string outputFolder = "/home/c7031109/data/studium/physik/semester_10/bak_thesis/results/really_final_results/rankNA1500/projsimnew3/bin/evalrank/";

    char c = 0;
    opterr = 1;
    while((c = getopt(argc, args, "i:o:p:")) != -1) {
        switch(c) {
        case 'i':
            inputFolder = string(optarg);
            break;
        case 'o':
            outputFolder = string(optarg);
            break;
        case 'p':
            filePrefix = string(optarg);
        }
    }

    vector<string> folderFiles = getFilesInDir(inputFolder);
    vector<string> selectedFilesList = filterByPrefix(folderFiles, filePrefix);
    vector<string> selectedFiles;
    vector<int> selectedFilesPos;
    for(int i = 0; i < selectedFilesList.size(); ++i) {
        string s = selectedFilesList.at(i);
        selectedFiles.push_back(inputFolder + s);
        ifstream ifs;
        ifs.open((inputFolder + s).c_str());
        string tmp;
        getline(ifs, tmp); getline(ifs, tmp); getline(ifs, tmp);
        selectedFilesPos.push_back(ifs.tellg());
        ifs.close();
    }


    int numOfCats = 0;
    vector<double> stat;
    stat.push_back(0);
    ifstream firstStream;
    firstStream.open(selectedFiles.at(0).c_str());

    while(selectedFiles.size() && stat.size()) {

        cout << "current number of cats: " << (numOfCats + 2) << endl;
        pair<vector<int>, vector<double> > retStat = PSEvaluator::evaluateStatistics(selectedFiles, selectedFilesPos);
        selectedFilesPos = retStat.first;
        stat = retStat.second;

        stringstream s;
        s << outputFolder << "eval" << filePrefix << "Cat" << (numOfCats + 2);
        cout << "writing to file: " << s.str() << endl;

        ofstream outFile;
        outFile.open(s.str().c_str());

        for(int i = 0; i < stat.size(); ++i) {
            outFile << i << "\t" << stat.at(i) << endl;
        }

        string line = "";
        for(int i = 0; i < selectedFiles.size(); ++i) {
            ifstream currentS;
            string currentFile = selectedFiles.at(i);
            currentS.open(currentFile.c_str());
            currentS.seekg(selectedFilesPos.at(i));
            getline(currentS, line); getline(currentS, line);
            selectedFilesPos.at(i) = currentS.tellg();
            currentS.close();
        }

        outFile.close();
        ++numOfCats;

    }

    /* eval results (convergence)
     * cat 3: 0.66499 (at about 100)
     * cat 4: 0.69779 (at about 500)
     * cat 5: 0.72269 (at about 1000)
     * cat 6: 0.73560 (at about 3000)
     * cat 7: 0.74152 (at about 5000)
     */


    /* with ranking (convergence)
     * gamma = 0; immunity = 50; max clips = 400
     * cat 3: 0.66661 (at about 120)
     * cat 4: 0.70076 (at about 500)
     * cat 5: 0.72211 (at about 1000)
     * cat 6: 0.72446 (at about 9000) --> why is it here so much later?
     * cat 7: 0.52198 --> here the clip number was not enough
     *
     * with 1500 clips
     * gamma = 0; immunity = 50; max clips = 1500
     * cat 3: 0.66130
     * cat 4: 0.69997
     * cat 5: 0.72224
     * cat 6: 0.73523
     * cat 7: 0.73995
     * cat 8: 0.51213
     */

    return EXIT_SUCCESS;

}
