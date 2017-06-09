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

const float TutorialDriver::G = 9.81;

const float TutorialDriver::MAX_UNSTUCK_ANGLE = 30.0/180.0*PI;  /* [radians] */
const float TutorialDriver::UNSTUCK_TIME_LIMIT = 2.0;           /* [s] */
const float TutorialDriver::MAX_UNSTUCK_SPEED = 5.0;
const float TutorialDriver::MIN_UNSTUCK_DIST = 3.0;
const float TutorialDriver::FULL_ACCEL_MARGIN = 1.0;
const float TutorialDriver::SHIFT = 0.9;
const float TutorialDriver::SHIFT_MARGIN = 4.0;
const float TutorialDriver::ABS_MINSPEED = 3.0;
const float TutorialDriver::ABS_SLIP = .9;
const float TutorialDriver::TCL_MINSPEED = 3.0;
const float TutorialDriver::TCL_SLIP = .9;
const float TutorialDriver::LOOKAHEAD_CONST = 17.0;
const float TutorialDriver::LOOKAHEAD_FACTOR = 0.33;

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

void TutorialDriver::initCA()
{
    char *WheelSect[4] = {SECT_FRNTRGTWHEEL, SECT_FRNTLFTWHEEL,
                          SECT_REARRGTWHEEL, SECT_REARLFTWHEEL};
    float rearwingarea = GfParmGetNum(car->_carHandle, SECT_REARWING,
                                       PRM_WINGAREA, (char*) NULL, 0.0);
    float rearwingangle = GfParmGetNum(car->_carHandle, SECT_REARWING,
                                        PRM_WINGANGLE, (char*) NULL, 0.0);
    float wingca = 1.23*rearwingarea*sin(rearwingangle);
    float cl = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                             PRM_FCL, (char*) NULL, 0.0) +
                GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                             PRM_RCL, (char*) NULL, 0.0);
    float h = 0.0;
    int i;
    for (i = 0; i < 4; i++)
        h += GfParmGetNum(car->_carHandle, WheelSect[i],
                          PRM_RIDEHEIGHT, (char*) NULL, 0.20);
    h*= 1.5; h = h*h; h = h*h; h = 2.0 * exp(-3.0*h);
    CA = h*cl + 4.0*wingca;
}

void TutorialDriver::initCW()
{
    float cx = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                            PRM_CX, (char*) NULL, 0.0);
    float frontarea = GfParmGetNum(car->_carHandle, SECT_AERODYNAMICS,
                                   PRM_FRNTAREA, (char*) NULL, 0.0);
    CW = 0.645*cx*frontarea;
}

/* Start a new race. */
void TutorialDriver::newRace(tCarElt* car, tSituation *s)
{
    this->car = car;
    CARMASS = GfParmGetNum(car->_carHandle, SECT_CAR, PRM_MASS, NULL, 1000);
    initCA();
    initCW();
    MAX_UNSTUCK_COUNT = int(UNSTUCK_TIME_LIMIT/RCM_MAX_DT_ROBOTS);
    stuckCounter = 0;
}

void TutorialDriver::drive(tSituation *s)
{
    update(s);

    memset(&car->ctrl, 0, sizeof(tCarCtrl));

    if (isStuck()) {
        car->ctrl.steer = -trackRelativeYaw / car->_steerLock;
        car->ctrl.gear = -1; // reverse gear
        car->ctrl.accelCmd = 0.5; // 30% accelerator pedal
        car->ctrl.brakeCmd = 0.0; // no brakes
    } else {
        car->ctrl.steer = getSteer();
        car->ctrl.gear = getGear();
        car->ctrl.brakeCmd = filterABS(getBrake());
        car->ctrl.accelCmd = car->ctrl.brakeCmd == 0.0 ? filterTCL(getAccel()) : 0;
    }
}

/* Set pitstop commands. */
int TutorialDriver::pitCommand(tSituation *s)
{
    return ROB_PIT_IM; /* return immediately */
}

/* End of the current race */
void TutorialDriver::endRace(tSituation *s)
{
}

/* Update my private data every timestep */
void TutorialDriver::update(tSituation *s)
{
    trackangle = RtTrackSideTgAngleL(&(car->_trkPos));
    trackRelativeYaw = trackangle - car->_yaw;
    NORM_PI_PI(trackRelativeYaw);
    mass = CARMASS + car->_fuel;
}

/* Check if I'm stuck */
bool TutorialDriver::isStuck()
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

float TutorialDriver::getAllowedSpeed(tTrackSeg* segment)
{
    if(segment->type == TR_STR)
        return 1000000;
    float arc = 0.0;
    tTrackSeg* s = segment;
    while(s->type == segment->type && arc < PI / 2.0)
    {
        arc += s->arc;
        s = s->next;
    }
    arc /= PI/2.0;
    float mu = segment->surface->kFriction;
    float r_eff = (segment->radius + segment->width/2.0) / sqrt(arc);
    return sqrt((mu * G * r_eff) /  (1.0 - MIN(1.0,r_eff * CA * mu / mass)));
}

float TutorialDriver::getDistToSegEnd()
{
    if(car->_trkPos.seg->type == TR_STR)
        return car->_trkPos.seg->length - car->_trkPos.toStart;
    return (car->_trkPos.seg->arc - car->_trkPos.toStart) * car->_trkPos.seg->radius;
}

float TutorialDriver::getAccel()
{
    float allowedspeed = getAllowedSpeed(car->_trkPos.seg);
    float gr = car->_gearRatio[car->_gear + car->_gearOffset];
    float rm = car->_enginerpmRedLine;
    if (allowedspeed > car->_speed_x + FULL_ACCEL_MARGIN) {
        return 1.0;
    } else {
        return allowedspeed/car->_wheelRadius(REAR_RGT)*gr /rm;
    }
}

float TutorialDriver::getBrake()
{
    tTrackSeg *segptr = car->_trkPos.seg;
    float currentspeedsqr = car->_speed_x*car->_speed_x;
    float mu = segptr->surface->kFriction;
    float maxlookaheaddist = currentspeedsqr/(2.0*mu*G);

    float lookaheaddist = getDistToSegEnd();
    float allowedspeed = getAllowedSpeed(segptr);
    if (allowedspeed < car->_speed_x)
        return 1.0;
    segptr = segptr->next;
    while (lookaheaddist < maxlookaheaddist) {
        allowedspeed = getAllowedSpeed(segptr);
        if (allowedspeed < car->_speed_x)
        {
            float allowedspeedsqr = allowedspeed*allowedspeed;
            float brakedist = mass * (currentspeedsqr - allowedspeedsqr) / (2.0*(mu*G*mass + allowedspeedsqr*(CA*mu + CW)));
            if(brakedist > lookaheaddist)
                return 1;
        }
        lookaheaddist += segptr->length;
        segptr = segptr->next;
    }
    return 0;
}

float TutorialDriver::getGear()
{
    if(car->_gear <= 0)
        return 1;
    float gr_up = car->_gearRatio[car->_gear + car->_gearOffset];
    float omega = car->_enginerpmRedLine / gr_up;
    float wr = car->_wheelRadius(2);
    if(omega * wr * SHIFT < car->_speed_x)
        return car->_gear + 1;
    else
    {
        float gr_down = car->_gearRatio[car->_gear + car->_gearOffset -1];
        omega = car->_enginerpmRedLine / gr_down;
        if(car->_gear > 1 && omega * wr * SHIFT > car->_speed_x + SHIFT_MARGIN)
            return car->_gear - 1;
    }
    return car->_gear;
}

float TutorialDriver::filterABS(float brake)
{
    if(car->_speed_x < ABS_MINSPEED)
        return brake;
    float slip = 0.0;
    int i;
    for(i=0;i<4;i++)
        slip += car->_wheelSpinVel(i) * car->_wheelRadius(i) / car->_speed_x;
    slip = slip / 4;
    if(slip < ABS_SLIP)
        return brake * slip;
    return brake;
}

float TutorialDriver::filterTCL(float accel)
{
    if(car->_speed_x < TCL_MINSPEED)
        return accel;
    float slip = car->_speed_x / filterTCL_RWD();
    if(slip < TCL_SLIP)
        return 0;
    return accel;
}

float TutorialDriver::filterTCL_RWD()
{
    return(car->_wheelSpinVel(REAR_RGT) + car->_wheelSpinVel(REAR_LFT)) * car->_wheelRadius(REAR_LFT) / 2.0;
}

v2d TutorialDriver::getTargetPoint()
{
    tTrackSeg *seg = car->_trkPos.seg;
    float lookahead = LOOKAHEAD_CONST + car->_speed_x*LOOKAHEAD_FACTOR;

    float length = getDistToSegEnd();

    while (length < lookahead) {
        seg = seg->next;
        length += seg->length;
    }

    length = lookahead - length + seg->length;
    v2d s;
    s.x = (seg->vertex[TR_SL].x + seg->vertex[TR_SR].x)/2.0;
    s.y = (seg->vertex[TR_SL].y + seg->vertex[TR_SR].y)/2.0;

    if ( seg->type == TR_STR)
    {
        v2d d;
        d.x = (seg->vertex[TR_EL].x - seg->vertex[TR_SL].x)/seg->length;
        d.y = (seg->vertex[TR_EL].y - seg->vertex[TR_SL].y)/seg->length;
        return s + d*length;
    }
    else
    {
        v2d c;
        c.x = seg->center.x;
        c.y = seg->center.y;
        float arc = length/seg->radius;
        float arcsign = (seg->type == TR_RGT) ? -1 : 1;
        arc = arc*arcsign;
        return s.rotate(c, arc);
    }
}

float TutorialDriver::getSteer()
{
    float targetAngle;
    v2d target = getTargetPoint();

    targetAngle = atan2(target.y - car->_pos_Y, target.x - car->_pos_X);
    targetAngle -= car->_yaw;
    NORM_PI_PI(targetAngle);
    return targetAngle / car->_steerLock;
}
