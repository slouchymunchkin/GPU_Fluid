//
//  Fluid.cpp
//  gpu_fluid
//
//  Created by M on 30/04/14.
//
//

#include "Fluid.h"

Fluid::Fluid()
{}

Fluid::~Fluid()
{}

void Fluid::init(int width, int height)
{

    
    w = width;
    h = height;
    
    advectionS.load("shaders/simple.vert", "shaders/advection.frag");
    //diffusionS.load("simple.vert", "shaders/advection.frag");
    jacobiS.load("shaders/simple.vert", "shaders/jacobi.frag");
    forceS.load("shaders/simple.vert", "shaders/force.frag");
    divergenceS.load("shaders/simple.vert", "shaders/divergence.frag");
    gradientS.load("shaders/simple.vert", "shaders/gradient.frag");
    simpleS.load("shaders/simple.vert", "shaders/simple.frag");
    splatS.load("shaders/simple.vert", "shaders/splat.frag");
    bouyancyS.load("shaders/simple.vert", "shaders/bouyancy.frag");
    addMassS.load("shaders/simple.vert", "shaders/addMass.frag");
    
    
    ofDisableArbTex();
    //img.loadImage("screen.png");
    //img2.loadImage("ros.png");
    ofEnableArbTex();
    
    /* advection = createFluidObj(w, h);
     jacobi = createFluidObj(w, h);
     velocity = createFluidObj(w, h);
     density = createFluidObj(w, h);
     temperature = createFluidObj(w, h);
     pressure = createFluidObj(w, h);
     */
    
    //advection.allocate(w, h, GL_RGB16F);
    jacobi.allocate(w, h, GL_RGB32F);
    velocity.allocate(w, h, GL_RGB32F);
    density.allocate(w, h, GL_RGB);
    pressure.allocate(w, h, GL_RGB32F);
    temperature.allocate(w, h, GL_RGB32F);
    
    
    
    divergence.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGB16F);
    divergence.begin();
	ofClear(0);
    divergence.end();
    
    
    obstacle.allocate(ofGetWindowWidth(), ofGetWindowHeight(), GL_RGB);
    obstacle.begin();
	ofClear(0);
    obstacle.end();
    
    //density.src.begin();
    //img2.draw(0, 0, w, h);
    //density.src.end();
    
    
    p = ofVec2f(ofGetWindowWidth() / 2, ofGetWindowHeight() - 50);
    c = ofVec3f(1.0, 0.3, 0.5);
    //c = ofVec3f(1.0, 1.0, 1.0);
    
    v = ofVec3f(0.0, -2.0, 0.0);
    t = ofVec3f(10.f, 10.f, 10.f);
    
    radius = 30.0;
    
    bouyAlpha = 150.0f;
	bouyBeta = 900.0f;
    temp = 0.0;
    dissipation = 0.98;
    timestep = currTime = lastTime = 0.0;
    
    
    temperature.src.begin();
    ofClear(temp);
    temperature.src.end();
    
    /*
     
     obstacle.begin();
     ofClear(0);
     ofSetColor(255);
     ofRect(2, 2, ofGetWindowWidth() - 4, ofGetWindowHeight() - 4);
     ofSetColor(0);
     obstacle.end();
     */


}

void Fluid::updateFluid(){
    
    // calculate delta time
    currTime = ofGetElapsedTimef();
    timestep = currTime - lastTime;
	lastTime = currTime;
    
    
    /***** FLUID BEGINS HERE ****/
    
    
    //glDisable(GL_DEPTH_TEST);
	//glDisable(GL_CULL_FACE);
    
    
    
    // advect fields
    
    computeAdvection(velocity, dissipation);
    velocity.swap();
    
    
    computeAdvection(temperature, dissipation);
    temperature.swap();
    
    computeAdvection(density, dissipation);
    density.swap();
    
    // buoyancy
    
    //computeForceBouyant();
    //velocity.swap();
    
    //p.x = ofGetMouseX();
    //p.y = ofGetMouseY();
    
    if ( temporalForces.size() != 0){
        for(int i = 0; i < temporalForces.size(); i++){
            ofVec3f tTemp = ofVec3f(temporalForces[i].temp,temporalForces[i].temp,temporalForces[i].temp);
            computeImpulse(temperature, temporalForces[i].pos, tTemp, temporalForces[i].rad);
            if (temporalForces[i].color.length() != 0)
            {
                computeImpulse(density, temporalForces[i].pos, temporalForces[i].color, temporalForces[i].rad);
            }
            if (temporalForces[i].vel.length() != 0)
            {
                ofVec3f tVel = ofVec3f(temporalForces[i].vel.x, temporalForces[i].vel.y, 0.0);
                computeImpulse(velocity , temporalForces[i].pos, tVel, temporalForces[i].rad);
            }
        }
        temporalForces.clear();
    }
    
    
    if ( constantForces.size() != 0){
        for(int i = 0; i < constantForces.size(); i++){
            ofVec3f tTemp = ofVec3f(constantForces[i].temp,constantForces[i].temp,constantForces[i].temp);
            computeImpulse(temperature, constantForces[i].pos, tTemp, constantForces[i].rad);
            if (constantForces[i].color.length() != 0)
            {
                computeImpulse(density, constantForces[i].pos, constantForces[i].color, constantForces[i].rad);
            }
            if (constantForces[i].vel.length() != 0)
            {
                ofVec3f tVel = ofVec3f(constantForces[i].vel.x, constantForces[i].vel.y, 0.0);
                computeImpulse(velocity , constantForces[i].pos, tVel, constantForces[i].rad);
            }
        }
    }
    
    //computeImpulse(temperature, p, t, radius);
    //computeImpulse(density, p, c, radius);
    //computeImpulse(velocity, p, v, radius);
    
    
    // field pressure
    
    computeDivergence();
    // clear the pressure field
    pressure.src.begin();
    ofClear(0);
    pressure.src.end();
    
    for (int i = 0; i < 40; ++i) {
        computeJacobi();
        pressure.swap();
    }
    
    // projection
    
    subtractGradient();
    velocity.swap();
    
    // advect quantity using updated velocity field
    
    ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofDisableBlendMode();
    
}

void Fluid::drawFluid(){
    
    
    
    ofBackground(0);
    
    
    ofPushStyle();
    glEnable(GL_BLEND);
    //glBlendFunc(GL_ONE, GL_ONE);
    ofSetColor(255);
    //bouyant.draw(0, 0);
    //velocity.src.draw(0, 0);
    //pressure.src.draw(0, 0);
    density.src.draw(0, 0, w, h);
    //temperature.src.draw(0, 0);
    //temp2.draw(0, 0);
    glDisable(GL_BLEND);
    ofPopStyle();
    
    //ofDrawBitmapString("fps: " + ofToString(ofGetFrameRate()), 50, 50);
    
}




void Fluid::computeAdvection(fluidObj &fObj, float dissipation)
{
    
    fObj.dst.begin();
    //ofClear(0);
    
    advectionS.begin();
    advectionS.setUniform1f("dx", 1.0f / ofGetWindowWidth());
    advectionS.setUniform1f("timestep", 0.525f);
    advectionS.setUniform1i("width", ofGetWindowWidth());
    advectionS.setUniform1f("dissipation", dissipation);
    
    advectionS.setUniformTexture("velocity", velocity.src.getTextureReference(), 0);
    advectionS.setUniformTexture("source", fObj.src.getTextureReference(), 1);
    advectionS.setUniformTexture("obstacle", obstacle.getTextureReference(), 2);
    
    quad();
    advectionS.end();
    
    fObj.dst.end();
    
}

void Fluid::computeJacobi()
{
    
    float dx = 1.0 / ofGetWindowWidth();
    dx = 1.0;
    // Second, do jacobi iterations to solve for q
	alpha = -(dx * dx);	// 4 for 2D, need to use 6 for 3D
	beta = 4.0f;
	// remon.note: there are some inconsistensies between Harris' dissertation and the GPU Gems article
	// the dissertation suggests values alpha = dx^2, beta = 4 [my sim totally breaks with these!]
	// the GPU Gems article suggests values alpha = -(dx)^2, beta = 4 [=Correct]
	// [Mar 23, 1046pm] I have derived it, so there is a - with the dx^2
    
	// Harris: To achieve good convergence on the solution, we typically use 40 to 80 Jacobi iterations.
	// The initial pressure texture is first cleared to all zero values
	// (in other words, we are using zero as our initial guess for the pressure field).
    
    
    pressure.dst.begin();
    //ofClear(0);
    
    jacobiS.begin();
    jacobiS.setUniform1f("dx", 1.0f / ofGetWindowWidth());
    jacobiS.setUniform1f("alpha", -1.0 * 1.0);
    jacobiS.setUniform1f("beta", 0.25f);
    jacobiS.setUniformTexture("pressure", pressure.src.getTextureReference(), 0);
    jacobiS.setUniformTexture("divergence", divergence.getTextureReference(), 1);
    jacobiS.setUniformTexture("obstacle", obstacle.getTextureReference(), 2);
    
    quad();
    
    jacobiS.end();
    
    pressure.dst.end();
    
}



void Fluid::computeDivergence()
{
    
    divergence.begin();
    //ofClear(0);
    
    divergenceS.begin();
    divergenceS.setUniform1f("dx", 1.0f / ofGetWindowWidth());
    divergenceS.setUniform1f("halfInverseCellSize", 0.5f / 1.0f);
    divergenceS.setUniformTexture("velocity", velocity.src.getTextureReference(), 0);
    divergenceS.setUniformTexture("boundaries", obstacle.getTextureReference(), 1);
    
    quad();
    
    divergenceS.end();
    
    divergence.end();
    
}


void Fluid::subtractGradient()
{
    
    velocity.dst.begin();
    //ofClear(0);
    
    gradientS.begin();
    gradientS.setUniform1f("dx", 1.0f / ofGetWindowWidth());
    gradientS.setUniformTexture("velocity", velocity.src.getTextureReference(), 0);
    gradientS.setUniformTexture("pressure", pressure.src.getTextureReference(), 1);
    gradientS.setUniformTexture("obstacle", obstacle.getTextureReference(), 2);
    
    quad();
    
    gradientS.end();
    
    velocity.dst.end();
    
    ofDisableBlendMode();
    
    
}


void Fluid::computeForceBouyant()
{
    
    /*bouyant.begin();
     ofClear(0);
     
     bouyancyS.begin();
     bouyancyS.setUniform1f("beta", bouyBeta);
     bouyancyS.setUniform1f("alpha", bouyAlpha);
     bouyancyS.setUniform1f("Tamb", 15.0);
     bouyancyS.setUniformTexture("density", density.getTextureReference(), 1);
     bouyancyS.setUniformTexture("temperature", temperature.getTextureReference(), 2);
     quad();
     
     bouyancyS.end();
     
     bouyant.end();
     
     
     
     dest.begin();
     ofClear(0);
     
     forceS.begin();
     forceS.setUniform1f("timestep", ofGetElapsedTimef() * 0.5);
     forceS.setUniformTexture("velocity", velocity.getTextureReference(), 1);
     forceS.setUniformTexture("mass", bouyant.getTextureReference(), 2);
     
     quad();
     
     forceS.end();
     
     dest.end();*/
    
    
    velocity.dst.begin();
    
    bouyancyS.begin();
    bouyancyS.setUniform1f("beta", bouyBeta);
    bouyancyS.setUniform1f("alpha", bouyAlpha);
    bouyancyS.setUniform1f("Tamb", temp);
    bouyancyS.setUniformTexture("velocity", velocity.src.getTextureReference(), 0);
    bouyancyS.setUniformTexture("density", density.src.getTextureReference(), 1);
    bouyancyS.setUniformTexture("temperature", temperature.src.getTextureReference(), 2);
    quad();
    bouyancyS.end();
    
    velocity.dst.end();
    
}




void Fluid::computeImpulse(fluidObj &fObj, ofVec2f &pos, ofVec3f &color, float radius)
{
    glEnable(GL_BLEND);
    //glBlendFunc(GL_ONE, GL_ONE);
    fObj.src.begin();
    //ofClear(0);
    splatS.begin();
    splatS.setUniform2f("Point", pos.x,  pos.y);
    splatS.setUniform1f("Radius", radius);
    splatS.setUniform3f("FillColor", color.x, color.y, color.z);
    quad();
    splatS.end();
    fObj.src.end();
    glDisable(GL_BLEND);
}


void Fluid::addTemporalForce(ofPoint _pos, ofPoint _vel, ofFloatColor _col, float _rad, float _temp, float _den)
{
    punctualForce f;
    
    float scale = 1.0;
    
    f.pos = _pos * scale;
    f.vel = _vel;
    f.color.set(_col.r,_col.g,_col.b);
    f.rad = _rad;
    f.temp = _temp;
    f.den = _den;
    
    temporalForces.push_back(f);
}


void Fluid::addConstantForce(ofPoint _pos, ofPoint _vel, ofFloatColor _col, float _rad, float _temp, float _den)
{
    punctualForce f;
    
    float scale = 1.0;
    
    f.pos = _pos * scale;
    f.vel = _vel;
    f.color.set(_col.r,_col.g,_col.b);
    f.rad = _rad;
    f.temp = _temp;
    f.den = _den;
    
    constantForces.push_back(f);
}

void Fluid::quad()
{
    ofSetColor(255,255);
    
    
    glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex3f(0, 0, 0);
    glTexCoord2f(w, 0); glVertex3f(w, 0, 0);
    glTexCoord2f(w, h); glVertex3f(w, h, 0);
    glTexCoord2f(0,h);  glVertex3f(0,h, 0);
    glEnd();
    
    /*
     // now display
     glBegin(GL_QUADS);
     {
     glTexCoord2f(0, 0); glVertex3f(0, 0, 0);
     glTexCoord2f(1, 0); glVertex3f(1, 0, 0);
     glTexCoord2f(1, 1); glVertex3f(1, 1, 0);
     glTexCoord2f(0, 1); glVertex3f(0, 1, 0);
     }
     glEnd();*/
}