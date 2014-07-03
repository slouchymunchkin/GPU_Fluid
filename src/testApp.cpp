#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    
    ofEnableAlphaBlending();
    ofSetCircleResolution(100);
    ofSetVerticalSync(true);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
    
    
    w = ofGetWindowWidth();
    h = ofGetWindowHeight();
    
    
    vidRecorder.setFfmpegLocation(ofFilePath::getAbsolutePath("ffmpeg"));
    vidRecorder.setVideoCodec("mpeg4");
    vidRecorder.setVideoBitrate("8000k");
    
    bRecording = false;
    
    
    fluid.init(w, h);
    
    
    fluid.addConstantForce(ofPoint(ofGetWindowWidth() / 2, ofGetWindowHeight() - 50), ofPoint(0.0, -2.0), ofFloatColor(1.0, 1.0, 1.0), 30.0);
    fluid.addConstantForce(ofPoint(ofGetWindowWidth() / 2, 50), ofPoint(0.0, 2.0), ofFloatColor(3.0, 0.0, 4.0), 30.0);
    fluid.addConstantForce(ofPoint(50, ofGetWindowHeight() / 2), ofPoint(2.0, 1.0), ofFloatColor(1.0, 0.0, 0.0), 30.0);
    //fluid.addConstantForce(ofPoint(ofGetWindowWidth() - 50 / 2, ofGetWindowHeight() / 2), ofPoint(-2.0, 0.0), ofFloatColor(1.0, 1.0, 1.0), 30.0);

    
}


//--------------------------------------------------------------
void testApp::update(){
    
    
    // Adding temporal Force
    //
    ofPoint m = ofPoint(mouseX,mouseY);
    ofPoint d = (m - oldM)*10.0;
    oldM = m;
    ofPoint c = ofPoint(640*0.5, 480*0.5) - m;
    c.normalize();
    
    
    fluid.addTemporalForce(m, d, ofFloatColor(1.0,1.0,1.0)*sin(ofGetElapsedTimef()),10.0f);
    
    fluid.updateFluid();
    
    if(bRecording){
        fluid.getDensity().readToPixels(recordPixels);
        vidRecorder.addFrame(recordPixels);
    }
    
    ofSetWindowTitle(ofToString(ofGetFrameRate()));

    
}

//--------------------------------------------------------------
void testApp::draw(){
    
    fluid.drawFluid();
    
    
    //ofDrawBitmapString("fps: " + ofToString(ofGetFrameRate()), 50, 50);

    if(bRecording){
        ofSetColor(255, 0, 0);
        ofCircle(ofGetWidth() - 20, 20, 5);
        ofSetColor(255);
        
    }
}




void testApp::exit() {
    vidRecorder.close();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

    if(key=='r'){
        bRecording = !bRecording;
        if(bRecording && !vidRecorder.isInitialized()) {
            vidRecorder.setup("gpu_fluid"+ofGetTimestampString()+".mov", ofGetWindowWidth(), ofGetWindowHeight(), 60); // no audio
        }
    }
    if(key=='c'){
        bRecording = false;
        vidRecorder.close();
    }
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}

