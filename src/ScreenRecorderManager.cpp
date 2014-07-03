//
//  ScreenRecorderManager.cpp
//  addon_videorecorder_example
//
//  Created by M on 02/02/14.
//
//

#include "ScreenRecorderManager.h"


ScreenRecordManager::ScreenRecordManager()
{
    
    vidRecorder = new ofxVideoRecorder();
    vidRecorder->setFfmpegLocation(ofFilePath::getAbsolutePath("ffmpeg")); // use this is you have ffmpeg installed in your data folder
    
    fileName = "testMovie";
    fileExt = ".mov"; // ffmpeg uses the extension to determine the container type. run 'ffmpeg -formats' to see supported formats
    
    // override the default codecs if you like
    // run 'ffmpeg -codecs' to find out what your implementation supports (or -formats on some older versions)
    vidRecorder->setVideoCodec("mpeg4");
    vidRecorder->setVideoBitrate("800k");
    //vidRecorder.setAudioCodec("mp3");
    //vidRecorder.setAudioBitrate("192k");
    
    //    soundStream.listDevices();
    //    soundStream.setDeviceID(11);
    //    soundStream.setup(this, 0, channels, sampleRate, 256, 4);
    
    //ofSetWindowShape(vidGrabber.getWidth(), vidGrabber.getHeight()	);
    //bRecording = false;
    //ofEnableAlphaBlending();
    
    bRecording = false;
    
}


ScreenRecordManager::~ScreenRecordManager()
{
    
    vidRecorder->close();

    
}


void ScreenRecordManager::recordScreen(ofPixelsRef &pixels)
{
    if(bRecording)
        vidRecorder->addFrame(pixels);

 }


void ScreenRecordManager::recordScreen(ofFbo &fbo)
{

    
    if(bRecording)
    {
        fbo.readToPixels(recordPixels);
        vidRecorder->addFrame(recordPixels);
    }
    
}

void ScreenRecordManager::stop()
{
    
    bRecording = false;
    vidRecorder->close();

    
}

void ScreenRecordManager::start()
{

    bRecording = !bRecording;
    if(bRecording && !vidRecorder->isInitialized()) {
        vidRecorder->setup(fileName+ofGetTimestampString()+fileExt, ofGetWindowWidth(), ofGetWindowHeight(), 60); // no audio
    }
    
}

