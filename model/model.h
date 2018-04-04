/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   model.h
 * Author: jom
 *
 * Created on March 28, 2018, 8:42 AM
 */

#ifndef MODEL_H
#define MODEL_H

const double Lf = 2.67;
const double vref = 15.0;

typedef struct {
    double x, y, psi, v, steer, throttle;
    } STATE;

    class model {
    public:
        STATE state;
        model(){
            this->state.x = 0.;
            this->state.y = 0.;
            this->state.psi = 0.;
            this->state.v = 0.;
            this->state.steer = 0.;
            this->state.throttle = 0.;
        };
        ~model(){};
        void move(double steer, double throttle, double dt){
            STATE newstate;
            STATE current_state = this->get_state();
            // set new v based on received throttle
            newstate.throttle = throttle;
            newstate.v = current_state.v + (throttle * dt);
            if(newstate.v > vref)
                newstate.v = vref;
            else if(newstate.v < 0.0)
                newstate.v = 0.0;
            // set new steer based on received steer
            newstate.steer = steer;
            if (newstate.steer > 0.43)
                newstate.steer = 0.43;
            else if(newstate.steer < -0.43)
                newstate.steer = -0.43;
            // execute actuators command
            newstate.psi = current_state.psi + ((newstate.v / Lf) * newstate.steer * dt);
            newstate.x = current_state.x + (newstate.v * cos(newstate.psi) * dt);
            newstate.y = current_state.y + (newstate.v * sin(newstate.psi) * dt);
            
            this->set_state(newstate);
        };
        STATE get_state(){
            return this->state;
        };
        
    private:
        void set_state(STATE newstate){
            this->state = newstate;
        };
    };

#endif /* MODEL_H */

