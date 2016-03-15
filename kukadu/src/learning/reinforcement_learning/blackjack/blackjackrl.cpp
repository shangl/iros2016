#include "blackjackrl.hpp"
#include "blackjackpolicy.hpp"
#include "blackjackaction.hpp"
#include "blackjackstate.hpp"
#include <time.h>
#include <stdio.h>
#include <string>
#include <algorithm>

using namespace std;

namespace kukadu {

BlackJackRL::BlackJackRL(int gameCounter) : RLInterface(KUKADU_SHARED_PTR<Policy>(new BlackJackPolicy()))
{
    this->gameCounter=gameCounter;
    this->moveList = KUKADU_SHARED_PTR<vector<KUKADU_SHARED_PTR<StateAction> > >(new vector<KUKADU_SHARED_PTR<StateAction> >());
    this->deck = KUKADU_SHARED_PTR<vector<Card> >(new vector<Card>());

}

void  BlackJackRL::performRollout(){
    this->reset();
    performPolicy();
    while(dealerSum < 17) {
              deal(false);
    };

    requestReward();
}

void BlackJackRL::performPolicy(){
    while(myStatus){
        KUKADU_SHARED_PTR<BlackJackState> currentState=KUKADU_SHARED_PTR<BlackJackState>(new BlackJackState(playerSum,dealerSum,usableAce));
        bool action=policy->getPolicy(currentState);
        moveList->push_back(KUKADU_SHARED_PTR<StateAction>(new StateAction(currentState,KUKADU_SHARED_PTR<BlackJackAction>(new BlackJackAction(action)))));
        switch(action){
            case BlackJackAction::STAY:
                myStatus=false;
                break;
            case BlackJackAction::HIT:
                deal(true);
                break;
        }
        if(playerSum > 21 ) {
            myStatus = false;
        }
    }
}

void BlackJackRL::reset(){
  //  cout<<"Starting round"<<endl;
    myStatus = true;
    moveList->clear();
    usableAce = false;
    dealerUsableAce = false;
    playerCount = 0;
    dealerCount = 0;
    playerSum = 0;
    dealerSum = 0;
    Card arr[52] = {ACE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, FACE, FACE, FACE, FACE,
            ACE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, FACE, FACE, FACE, FACE,
            ACE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, FACE, FACE, FACE, FACE,
            ACE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT, NINE, FACE, FACE, FACE, FACE };
    deck->clear();
    for(int i = 0; i < 52; i++)
        deck->push_back(arr[i]);
    random_shuffle(deck->begin(), deck->end());

    deal(true);
    deal(false);
    deal(true);
    while(playerSum < 12)
        deal(true);

}

void BlackJackRL::loop(){
    for(int i=0;i<this->gameCounter;i++)
    {
        this->performRollout();
    }
    this->policy->dumpPolicy();
}

void BlackJackRL::deal(bool player){
    Card top = deck->at(0);
    deck->erase(deck->begin());
    if(player) {
        playerSum += top;
        if(usableAce) {
            if(dealerSum > 21) {
                usableAce = false;
                dealerSum -= 10;
            }
        } else if(top == ACE && playerSum < 12 ){
            usableAce = true;
            playerSum += 10;
        }
        playerCount++;
    } else {
        if(dealerCount == 0) revealed = top;
        dealerSum += top;
        if(dealerUsableAce) {
            if(dealerSum > 21) {
                dealerUsableAce = false;
                dealerSum -= 10;
            }
        } else if(top == ACE && dealerSum < 12 ){
            dealerUsableAce = true;
            dealerSum += 10;
        }
        dealerCount++;
    }
  //  cout<<"Dealt: "<<top<<" to "<<(player ? "Player" : "Dealer")<<endl;
}

int BlackJackRL::result() {

  /*  cout<<"Dealer has: "<<revealed<<endl;
    cout<<"Player sum: "<<playerSum<<endl;
    cout<<"\tUsable Ace: "<<(usableAce ? "Yes" : "No")<<endl;
    cout<<"Dealer sum: "<<dealerSum<<endl;
    cout<<"\tUsable Ace: "<<(dealerUsableAce ? "Yes" : "No")<<endl;*/
    if(playerSum > 21) return -1;
    else if (dealerSum > 21) return 1;
    else if (playerSum > dealerSum) return 1;
    else if (playerSum < dealerSum) return -1;
    else return 0;
}

void BlackJackRL::requestReward(){
    policy->updatePolicy(moveList,result());
}


}


void usage_err(string key){
     if(key.compare("-h") != 0)
     cout<<"Improper usage of "<<key<<endl;
     cout<<"Usage: ./blackjack [-g episode] [-debug]"<<endl;
     cout<<".blackjack -h displays usage"<<endl;
     exit(0);
}

int main(int argc, char* argv[]){
    using namespace kukadu;

    srand(time(NULL));
    int gameCounter=1;
    for(int i=1;i<argc;i++){
        string temp=argv[i];
        if(temp.compare("-g") == 0){
                    if(++i<argc)
                        gameCounter = atoi(argv[i]);

        } else { usage_err(temp); }

    }
    KUKADU_SHARED_PTR<BlackJackRL> blackjack(new BlackJackRL(gameCounter));

    blackjack->loop();

}
