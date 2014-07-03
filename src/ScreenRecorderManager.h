//
//  ScreenRecorderManager.h
//  addon_videorecorder_example
//
//  Created by M on 02/02/14.
//
//

#pragma once

#include "ofxVideoRecorder.h"



class ScreenRecordManager
{
    
    
public:
    ScreenRecordManager();
    ~ScreenRecordManager();
    void recordScreen(ofPixelsRef &pixels);
    void recordScreen(ofFbo &pixels);
    void setFilename(string name) { fileName = name; };
    void stop();
    void start();
    
    
    
private:

    
    
    ofxVideoRecorder    *vidRecorder;
    //ofSoundStream       soundStream;
    bool bRecording;
    int sampleRate;
    int channels;
    string fileName;
    string fileExt;
    
    ofPixels recordPixels;

};