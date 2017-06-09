/***************************************************************************

    file                 : driver.h
    created              : Thu Dec 20 01:20:19 CET 2002
    copyright            : (C) 2002 Bernhard Wymann

 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

#ifndef _TUTORIALDRIVER_H_
#define _TUTORIALDRIVER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <tgf.h>
#include <track.h>
#include <car.h>
#include <raceman.h>
#include <robottools.h>
#include <robot.h>

class TutorialDriver {
    public:
        TutorialDriver(int index);

        /* callback functions called from TORCS */
        void initTrack(tTrack* t, void *carHandle,
                       void **carParmHandle, tSituation *s);
        void newRace(tCarElt* car, tSituation *s);
        void drive(tSituation *s);
        int pitCommand(tSituation *s);
        void endRace(tSituation *s);

    private:
        /* utility functions */
        void initCA();
        void initCW();
        bool isStuck();
        void update(tSituation *s);
        float getAllowedSpeed(tTrackSeg* segment);
        float getDistToSegEnd();
        float getAccel();
        float getBrake();
        float getGear();

        /* per robot global data */
        int stuckCounter;
        float trackangle;
        float trackRelativeYaw;
        float mass;

        /* data that should stay constant after first initialization */
        int MAX_UNSTUCK_COUNT;
        int INDEX;
        float CA;
        float CW;
        float CARMASS;
        tCarElt* car;

        /* class constants */
        static const float G;
        static const float MAX_UNSTUCK_ANGLE;
        static const float UNSTUCK_TIME_LIMIT;
        static const float MAX_UNSTUCK_SPEED;
        static const float MIN_UNSTUCK_DIST;
        static const float FULL_ACCEL_MARGIN;
        static const float SHIFT;
        static const float SHIFT_MARGIN;

        /* track variables */
        tTrack* track;
};

#endif //TUTORIALDRIVER_H_
