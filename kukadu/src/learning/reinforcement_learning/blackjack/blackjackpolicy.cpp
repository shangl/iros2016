#include "blackjackpolicy.hpp"
#include "blackjackstate.hpp"
#include "blackjackaction.hpp"
#include <vector>
#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

namespace kukadu {
    BlackJackPolicy::BlackJackPolicy() : Policy(KUKADU_SHARED_PTR<vector<pair<KUKADU_SHARED_PTR<StateAction>, double> > >(new vector<pair<KUKADU_SHARED_PTR<StateAction>, double>  >()), KUKADU_SHARED_PTR<vector<pair<KUKADU_SHARED_PTR<State>,bool> > >(new vector<pair<KUKADU_SHARED_PTR<State>,bool> >()))
    {
        KUKADU_SHARED_PTR<BlackJackAction> stay =  KUKADU_SHARED_PTR<BlackJackAction>(new BlackJackAction(BlackJackAction::STAY));
        KUKADU_SHARED_PTR<BlackJackAction> hit =  KUKADU_SHARED_PTR<BlackJackAction>(new BlackJackAction(BlackJackAction::HIT));
        this->visit = KUKADU_SHARED_PTR<std::vector<std::pair<KUKADU_SHARED_PTR<StateAction>, int> > >(new vector<std::pair<KUKADU_SHARED_PTR<StateAction>, int> >());
        for(int i=11;i<22;i++){
            for(int j=2;j<12;j++){
                KUKADU_SHARED_PTR<BlackJackState> sAce = KUKADU_SHARED_PTR<BlackJackState>(new BlackJackState(i,j,true));
                KUKADU_SHARED_PTR<BlackJackState> sNoAce = KUKADU_SHARED_PTR<BlackJackState>(new BlackJackState(i,j,false));
                KUKADU_SHARED_PTR<StateAction> sa1= KUKADU_SHARED_PTR<StateAction>(new StateAction(sAce,stay));
                KUKADU_SHARED_PTR<StateAction> sa2= KUKADU_SHARED_PTR<StateAction>(new StateAction(sAce,hit));
                KUKADU_SHARED_PTR<StateAction> sa3= KUKADU_SHARED_PTR<StateAction>(new StateAction(sNoAce,stay));
                KUKADU_SHARED_PTR<StateAction> sa4= KUKADU_SHARED_PTR<StateAction>(new StateAction(sNoAce,hit));
                this->qVal->push_back(pair<KUKADU_SHARED_PTR<StateAction>,double>(sa1, 2));
                this->qVal->push_back(make_pair(sa2,2));
                this->qVal->push_back(make_pair(sa3,2));
                this->qVal->push_back(make_pair(sa4,2));
                this->visit->push_back(make_pair(sa1,0));
                this->visit->push_back(make_pair(sa2,0));
                this->visit->push_back(make_pair(sa3,0));
                this->visit->push_back(make_pair(sa4,0));
                this->policy->push_back(make_pair(sAce,(i < 17)));
                this->policy->push_back(make_pair(sNoAce,(i < 17)));
            }
        }
    }

    void BlackJackPolicy::dumpPolicy()
    {
        string file = "data.dat";
        ofstream datfile;
        datfile.open (file.c_str());

        file = "data.csv";
        ofstream csvfile;
        csvfile.open (file.c_str());

        file = "chart-no-ace.csv";
        ofstream csvfilechartnoace;
        csvfilechartnoace.open (file.c_str());

        file = "chart-ace.csv";
        ofstream csvfilechartace;
        csvfilechartace.open (file.c_str());

        file = "chart-Q-ace.csv";
        ofstream csvfilechartQ;
        csvfilechartQ.open (file.c_str());

        file = "chart-Q-no-ace.csv";
        ofstream csvfilechartQn;
        csvfilechartQn.open (file.c_str());

        csvfile<<"\"My Value\",\"Dealer Value\",\"Usable Ace\",\"Action\",\"Q-Value\",\"Policy\""<<endl;
        csvfilechartace<<"\"My Value\",\"Dealer Value\",\"Policy\""<<endl;
        csvfilechartnoace<<"\"My Value\",\"Dealer Value\",\"Policy\""<<endl;
        csvfilechartQ<<"\"My Value\",\"Dealer Value\",\"Q-Value\""<<endl;
        csvfilechartQn<<"\"My Value\",\"Dealer Value\",\"Q-Value\""<<endl;
        for(int i=0;i<this->qVal->size();i++){
            std::pair<KUKADU_SHARED_PTR<StateAction>, double> qv= qVal->at(i);
            KUKADU_SHARED_PTR<StateAction> sa=qv.first;
            KUKADU_SHARED_PTR<BlackJackState> s  = KUKADU_STATIC_POINTER_CAST<BlackJackState>(sa->getState());
            KUKADU_SHARED_PTR<BlackJackAction> a  = KUKADU_STATIC_POINTER_CAST<BlackJackAction>(sa->getAction());
            double val=qv.second;
            bool poli = true;
            for(vector<pair<KUKADU_SHARED_PTR<State>, bool> >::iterator p = policy->begin(); p != policy->end(); ++p){
                if((*p).first->equals(s))
                    poli=(*p).second;
            }

            datfile<< "My Value: " <<  s->getMyValue() << "| Dealer Value: " << s->getDealerValue() << "| Usable Ace: " << ((s->getUsableAce()) ? "Yes" : "No") << "| Action: "<<((a->getAction()) ? "HIT" : "STAY")<< "| Q-Value: "<<val << "| Policy: "<< ((poli) ? "HIT" : "STAY") <<endl;
            csvfile<<s->getMyValue() << "," << s->getDealerValue() << "," << ((s->getUsableAce()) ? "\"Yes\"" : "\"No\"") << ","<<((a->getAction()) ? "\"HIT\"" : "\"STAY\"")<< ","<<val << ","<< ((poli) ? "\"HIT\"" : "\"STAY\"") <<endl;

            if(a->getAction()){
                if(s->getUsableAce())
                {
                    csvfilechartace<<s->getMyValue() << "," << s->getDealerValue() << "," << poli <<endl;
                    csvfilechartQ<<s->getMyValue() << "," << s->getDealerValue() << "," <<val <<endl;

                } else
                {
                    csvfilechartnoace<<s->getMyValue() << "," << s->getDealerValue() << "," << poli <<endl;
                    csvfilechartQn<<s->getMyValue() << "," << s->getDealerValue() << "," << val <<endl;

                }
            }
        }
        datfile.close();
        csvfile.close();
        csvfilechartace.close();
        csvfilechartnoace.close();
        csvfilechartQ.close();
        csvfilechartQn.close();
    }

    void BlackJackPolicy::updatePolicy(KUKADU_SHARED_PTR<std::vector<KUKADU_SHARED_PTR<StateAction> > > sa, int result){
        for (vector<KUKADU_SHARED_PTR<StateAction> >::iterator stateaction = sa->begin(); stateaction != sa->end(); ++stateaction)
        {
            for(vector<pair<KUKADU_SHARED_PTR<StateAction>, int> >::iterator v = visit->begin(); v != visit->end(); ++v)
            {
                if((*v).first->equals(*stateaction))
                {
                    for(vector<pair<KUKADU_SHARED_PTR<StateAction>, double> >::iterator q = qVal->begin(); q != qVal->end(); ++q)
                    {
                        if((*q).first->equals(*stateaction))
                        {
                            (*q).second = ((*v).second == 0 ? (*q).second : (*q).second * (double) (*v).second) + (double) result;
                            (*v).second = (*v).second+1;
                            (*q).second =  (*q).second/((double) (*v).second);
                            KUKADU_SHARED_PTR<BlackJackState> state = KUKADU_STATIC_POINTER_CAST<BlackJackState>((*stateaction)->getState());
                            KUKADU_SHARED_PTR<BlackJackAction> action  = KUKADU_STATIC_POINTER_CAST<BlackJackAction>((*stateaction)->getAction());
                            KUKADU_SHARED_PTR<BlackJackAction> actionNeg = KUKADU_SHARED_PTR<BlackJackAction>(new BlackJackAction(!(action->getAction())));
                            KUKADU_SHARED_PTR<StateAction> negation = KUKADU_SHARED_PTR<StateAction>(new StateAction(state,actionNeg));
                            for(vector<pair<KUKADU_SHARED_PTR<StateAction>, double> >::iterator q1 = qVal->begin(); q1 != qVal->end(); ++q1)
                            {
                                if((*q1).first->equals(negation))
                                {
                                    for(vector<pair<KUKADU_SHARED_PTR<State>, bool> >::iterator p = policy->begin(); p != policy->end(); ++p)
                                    {
                                        if((*p).first->equals(state))
                                        {
                                            if((*q).second > (*q1).second)
                                            {
                                                (*p).second=action->getAction();
                                            } else
                                            {
                                                (*p).second=actionNeg->getAction();
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }



}
