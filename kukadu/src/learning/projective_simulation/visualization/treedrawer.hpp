#ifndef KUKADU_TREEDRAWER_H
#define KUKADU_TREEDRAWER_H

#include <vector>
#include <iostream>

#if VISUALIZATION == 1
    #include <allegro5/allegro.h>
    #include <allegro5/allegro_font.h>
#endif

#include "../core/perceptclip.hpp"
#include "../core/projectivesimulator.hpp"

#define TREEDRAWER_H_NODE_X_OFFS 50
#define TREEDRAWER_H_NODE_Y_OFFS 70
#define TREEDRAWER_H_NODE_X_DIST 100
#define TREEDRAWER_H_NODE_Y_DIST 150
#define TREEDRAWER_H_EDGE_Y_OFFS 20
#define TREEDRAWER_H_WINDOW_X_SIZE 1024
#define TREEDRAWER_H_WINDOW_Y_SIZE 1500
#define TREEDRAWER_H_NODE_RADIUS 45

namespace kukadu {

    class TreeDrawer {

    private:

    #if VISUALIZATION == 1
        ALLEGRO_FONT* font;
        ALLEGRO_DISPLAY* display;
        ALLEGRO_EVENT_QUEUE* event_queue;

        ALLEGRO_COLOR textColor;
        ALLEGRO_COLOR circleColor;
    #endif

        int windowXSize;
        int windowYSize;

        void construct();
        void drawNode(int x, int y, std::string text, int level);

        int compteXOffset(KUKADU_SHARED_PTR<std::set<KUKADU_SHARED_PTR<Clip>, clip_compare> > level);

    public:

        TreeDrawer();
        TreeDrawer(int windowXSize, int windowYSize);
        ~TreeDrawer();

        void waitForEnter();
        void drawTree(KUKADU_SHARED_PTR<ProjectiveSimulator> projSim);

    };

}

#endif
