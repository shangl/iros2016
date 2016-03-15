#include "inftheometriclearner.hpp"

using namespace arma;
using namespace std;

namespace kukadu {

    InfTheoMetricLearner::InfTheoMetricLearner(std::vector<arma::vec> x1s, std::vector<arma::vec> x2s, std::vector<double> distances, double simBorder, double disSimBorder, double gamma,
                           double divTol, int checkForConCount) : MahalanobisLearner(x1s, x2s, distances) {

        this->simBorder = simBorder;
        this->disSimBorder = disSimBorder;
        this->gamma = gamma;
        this->divTol = divTol;
        this->checkForConCount = checkForConCount;

        int dim = getVectorDim();

        vec dia(dim);
        dia.fill(1.0);

        // euclidean distance is standard matrix
        M0 = mat(dim, dim);
        M0.fill(0.0);
        M0.diag() = dia;

    }

    Mahalanobis InfTheoMetricLearner::learnMetric() {

        computeConstraints();
        int dim = getVectorDim();

        Mahalanobis metric = getMetric();
        metric.setM(M0);

        int simConCount = simConstraints.getConstraintCount();
        int disSimConCount = disSimConstraints.getConstraintCount();

        vec x1;
        vec x2;
        double slack = 0.0;
        double lambda = 0.0;

        InfTheoConstraints* currentConstSet = NULL;
        int currentIdx = 0;
        int currentSimIdx = 0;
        int currentDisSimIdx = 0;

        double p = 0.0;
        double delta = 0.0;
        double alpha = 0.0;
        double beta = 0.0;

        int i = 0;

        for(i = 0; !checkConvergence(metric.getM()) && (simConCount || disSimConCount); ++i) {

            // choose sim constraint
            if(i % 2 && simConCount) {

                currentConstSet = &simConstraints;
                currentIdx = currentSimIdx = (currentSimIdx + 1) % simConCount;

                // line 3.3
                delta = 1;

            }
            // choose dissim constraint
            else if(!(i %2) && disSimConCount) {
                currentConstSet = &disSimConstraints;
                currentIdx = currentDisSimIdx = (currentDisSimIdx + 1) % disSimConCount;

                // line 3.3
                delta = -1;

            }

            // pick some constraint (line 3.1)
            x1 = currentConstSet->getX1(currentIdx);
            x2 = currentConstSet->getX2(currentIdx);
            slack = currentConstSet->getSlack(currentIdx);
            lambda = currentConstSet->getLambda(currentIdx);

            // line 3.2
            p = metric.computeSquaredDistance(x1, x2);

            // ignore pairs with x1 == x2
            if(p > 0) {

                // line 3.4
                alpha = min(lambda, delta / 2.0 * (1.0 / p - gamma / slack));

                // line 3.5
                beta = delta * alpha / (1.0 - delta * alpha * p);

                /*
                // prints debug information
                cout << "p: " << p << endl;
                cout << "alpha: " << alpha << endl;
                cout << "1.0 / p: " << (1.0 / p) << endl;
                cout << "gamma / slack: " << (gamma / slack) << endl;
                cout << x1.t() << endl << x2.t() << endl;
                cout << "delta / 2.0 * (1.0 / p - gamma / slack): " << (delta / 2.0 * (1.0 / p - gamma / slack)) << endl;
                cout << "beta: " << beta << endl;
                */

                // line 3.6
                double nextSlack = gamma * slack / (gamma + delta * alpha * slack);
                currentConstSet->setSlack(currentIdx, nextSlack);

                // line 3.7
                double nextLambda = lambda - alpha;
                currentConstSet->setLambda(currentIdx, nextLambda);

                // line 3.8
                mat currentM = metric.getM();
                currentM = currentM + beta * currentM * (x1 - x2) * (x1 - x2).t() * currentM;
                metric.setM(currentM);

            }

        }

        cout << "(InfTheoMetricLearner) metric learning converged in " << i << " iterations" << endl;

        return metric;

    }

    int InfTheoMetricLearner::checkConvergence(arma::mat currentM) {

        int simConCount = simConstraints.getConstraintCount();
        int disSimConCount = disSimConstraints.getConstraintCount();

        if(oldMetrics.size() < (simConCount + disSimConCount))
            oldMetrics.push_back(currentM);

        lastMetrics.push_back(currentM);

        int metricsCount = oldMetrics.size();
        int lastMetricsCount = lastMetrics.size();

        // use whole date before checking convergence
        if(metricsCount < (simConCount + disSimConCount))
            return 0;

        // use at least checkForConCount metrics to check for convergence
        if(lastMetricsCount <= checkForConCount)
            return 0;

        lastMetrics.erase(lastMetrics.begin());
        for(int i = 0; (i + 1) < checkForConCount; ++i) {

            mat M0 = lastMetrics.at(i);
            mat M1 = lastMetrics.at(i + 1);

            mat diffM = M0 - M1;
            double convDiv = sum(sum(diffM % diffM));

            if(convDiv > divTol)
                return 0;

        }

        return 1;

    }

    void InfTheoMetricLearner::computeConstraints() {

        simConstraints.flush();
        disSimConstraints.flush();

        int sampleCount = getSampleCount();

        for(int i = 0; i < sampleCount; ++i) {

            vec x1 = getX1(i);
            vec x2 = getX2(i);

            double dist = getSampleDistance(i);
            if(dist <= simBorder) {
                simConstraints.addConstraint(x1, x2, simBorder);
            } else if(dist >= disSimBorder) {
                disSimConstraints.addConstraint(x1, x2, disSimBorder);
            } else {
                // no additional constraint
            }

        }

    }

}
