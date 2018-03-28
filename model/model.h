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

typedef struct {
    double x, y, psi, v, steer, throttle;
    } STATE;

    class model {
    public:
        STATE state;
        model();
        ~model();
        void move(double steer, double throttle, double dt);
        STATE get_state();
        
    private:
        void set_state(STATE newstate);
    };

#endif /* MODEL_H */

