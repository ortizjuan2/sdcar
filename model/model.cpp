/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "model.h"
#include "cmath"

const double Lf = 2.67;

model::model(){
    this->state.x = 0.;
    this->state.y = 0.;
    this->state.psi = 0.;
    this->state.v = 0.;
    this->state.steer = 0.;
    this->state.throttle = 0.;
}

model::~model(){};

void model::move(double steer, double throttle, double dt){
    STATE newstate;
    STATE current_state = this->get_state();
    newstate.x = current_state.x + (current_state.v * cos(current_state.psi) * dt);
    newstate.y = current_state.y + (current_state.v * sin(current_state.psi) * dt);
    newstate.psi = current_state.psi + ((current_state.v / Lf) * steer * dt);
    newstate.v = current_state.v + (throttle * dt);
    newstate.steer = current_state.steer + steer;
    if (newstate.steer > 0.43)
        newstate.steer = 0.43;
    else if(newstate.steer < -0.43)
        newstate.steer = -0.43;

    newstate.throttle = (current_state.throttle + throttle);
    if (newstate.throttle > 1.0)
        newstate.throttle = 1.0;
    else if (newstate.throttle < -1.0)
        newstate.throttle = -1.0;
    
    this->set_state(newstate);
}

STATE model::get_state(){
    return this->state;
}

void model::set_state(STATE newstate){
    this->state = newstate;
}