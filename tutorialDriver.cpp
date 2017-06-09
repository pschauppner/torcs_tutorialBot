/***************************************************************************

    file                 : driver.cpp
    created              : Thu Dec 20 01:21:49 CET 2002
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

#include "tutorialDriver.h"

const float TutorialDriver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI;  /* [radians] */
const float TutorialDriver::UNSTUCK_TIME_LIMIT = 2.0;           /* [s] */

TutorialDriver::TutorialDriver(int index)
{
    INDEX = index;
}

/* Called for every track change or new race. */
void TutorialDriver::initTrack(tTrack* t, void *carHandle,
                       void **carParmHandle, tSituation *s)
{
    track = t;
    *carParmHandle = NULL;
}

/* Start a new race. */
void TutorialDriver::newRace(tCarElt* car, tSituation *s)
{
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    stuck = 0;
}

void TutorialDriver::drive(tCarElt* car, tSituation *s)
{
    update(car, s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck(car)) {
        car->ctrl.steer = -angle / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } else {
        float steerangle = angle - car->_trkPos.toMiddle/car->_trkPos.seg->width;

        car->ctrl.steer = steerangle / car->_steerLock;
        car->ctrl.gear = 1; // first gear
        car->ctrl.accelCmd = 0.3; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    }
}

/* Set pitstop commands. */
int TutorialDriver::pitCommand(tCarElt* car, tSituation *s)
{
    return ROB_PIT_IM; /* return immediately */
}

/* End of the current race */
void TutorialDriver::endRace(tCarElt *car, tSituation *s)
{
}

/* Update my private data every timestep */
void TutorialDriver::update(tCarElt* car, tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    angle = trackangle - car->_yaw;
    NORM_PI_PI(angle);
}

/* Check if I'm stuck */
bool TutorialDriver::isStuck(tCarElt* car)
{
    if (fabs(angle) < MAX_UNSTUCK_ANGLE) {
        stuck = 0;
        return false;
    }
    if (stuck < MAX_UNSTUCK_COUNT) {
        stuck++;
        return false;
    } else {
        return true;
    }
}
