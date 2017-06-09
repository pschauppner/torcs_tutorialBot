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
const float TutorialDriver::MAX_UNSTUCK_SPEED = 5.0;
const float TutorialDriver::MIN_UNSTUCK_DIST = 3.0;

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
    stuckCounter = 0;
}

void TutorialDriver::drive(tCarElt* car, tSituation *s)
{
    update(car, s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck(car)) {
        car->ctrl.steer = -trackRelativeYaw / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.5; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } else {
        float steerangle = trackRelativeYaw - car->_trkPos.toMiddle/car->_trkPos.seg->width;

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
    trackRelativeYaw = trackangle - car->_yaw;
    NORM_PI_PI(trackRelativeYaw);
}

/* Check if I'm stuck */
bool TutorialDriver::isStuck(tCarElt* car)
{
    if (fabs(trackRelativeYaw) > MAX_UNSTUCK_ANGLE &&
        car->_speed_x < MAX_UNSTUCK_SPEED &&
        fabs(car->_trkPos.toMiddle) > MIN_UNSTUCK_DIST) {
        if(stuckCounter > MAX_UNSTUCK_COUNT && car->_trkPos.toMiddle * trackRelativeYaw < 0)
            return true;
        else{
            stuckCounter++;
            return false;
        }
    }
    else {
        stuckCounter = 0;
        return false;
    }
}
