/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   DepthHypo.h
 * Author: yiluo
 *
 * Created on November 10, 2016, 6:30 PM
 */

#ifndef DEPTHHYPO_H
#define DEPTHHYPO_H

class DepthHypo {
public:
    
    DepthHypo();
    
    bool isValid;
    float depth;
    float var;
    float depth_smoothed;
    float var_smoothed;
};



#endif /* DEPTHHYPO_H */

