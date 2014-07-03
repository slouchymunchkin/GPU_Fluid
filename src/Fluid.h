//
//  Fluid.h
//  gpu_fluid
//
//  Created by M on 30/04/14.
//
//

#ifndef __gpu_fluid__Fluid__
#define __gpu_fluid__Fluid__

#include <iostream>
#include "ofMain.h"


class Fluid
{
    
    struct fluidObj {
        ofFbo src;
        ofFbo dst;
        
        void allocate(int width, int height, int internalFormat)
        {
            
            src.allocate(width, height, internalFormat);
            src.begin();
            ofClear(0);
            src.end();
            
            dst.allocate(width, height, internalFormat);
            dst.begin();
            ofClear(0);
            dst.end();
            
        };
        
        void swap()
        {
            ofFbo temp;
            temp = src;
            src = dst;
            dst = temp;
            
        };
        
        void clear()
        {
            src.begin();
            ofClear(0);
            src.end();
            
            dst.begin();
            ofClear(0);
            dst.end();
        }
    };
    
    typedef struct  {
        ofVec3f color;
        ofVec2f pos;
        ofVec2f vel;
        float   rad;
        float   temp;
        float   den;
    } punctualForce;
    
    
private:
    

    void computeAdvection(fluidObj &fObj, float dissipation);
    void computeDivergence();
    void computeJacobi();
    void subtractGradient();
    void computeImpulse(fluidObj &fObj, ofVec2f &pos, ofVec3f &color, float radius);
    void computeForceBouyant();
    
    void computeBoundary();
    void computeForces(ofFbo &dest, ofVec2f pos, float value);
   
    void quad();



public:
    
    Fluid();
    ~Fluid();
    void init(int width, int height);
    void updateFluid();
    void drawFluid();
    void addTemporalForce(ofPoint _pos, ofPoint _dir, ofFloatColor _col, float _rad = 1.0f, float _temp = 10.f, float _den = 1.f );
    void addConstantForce(ofPoint _pos, ofPoint _dir, ofFloatColor _col, float _rad = 1.0f, float _temp = 10.f, float _den = 1.f );


    ofFbo getVelocity() { return velocity.src; };
    ofFbo getDensity() { return density.src; };
    ofFbo getPressure() { return pressure.src; };

    
    ofFbo obstacle, divergence, scalarfield, bouyant;
    
    ofShader advectionS, diffusionS, jacobiS, forceS, pressureS, gradientS, simpleS, splatS, divergenceS, bouyancyS, addMassS;
        
    fluidObj velocity, density, pressure, temperature, jacobi;
    
    int w, h;
    int cellsize;

    float alpha, beta;
    float timestep, currTime, lastTime, bouyBeta, bouyAlpha, temp, dissipation;
    float radius;
    //int dx;
    
    ofVec2f p;
    ofVec3f c;
    ofVec3f v;
    ofVec3f t;
    
    vector<punctualForce> temporalForces;
    vector<punctualForce> constantForces;



};

#endif /* defined(__gpu_fluid__Fluid__) */
