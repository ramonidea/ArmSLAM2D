#pragma once

#include "ofMain.h"
#include "Robot.h"
#include "World.h"
#include "TSDF.h"

class ofApp: public ofBaseApp
{
    public:
        enum Experiment
        {
            GroundTruth,
            Odometry,
            ConstrainedDescent,
            UnconstraintedDescent
        };

        void setup();
        void update();
        void draw();

        void keyPressed(int key);
        void keyReleased(int key);
        void mouseMoved(int x, int y);
        void mouseDragged(int x, int y, int button);
        void mousePressed(int x, int y, int button);
        void mouseReleased(int x, int y, int button);
        void windowResized(int w, int h);
        void dragEvent(ofDragInfo dragInfo);
        void gotMessage(ofMessage msg);

        arm_slam::Robot<3> robot;
        arm_slam::Robot<3> fakeRobot;
        arm_slam::Robot<3> odomRobot;
        arm_slam::DepthCamera freeCamera;
        arm_slam::Robot<3>::Config offset;
        arm_slam::World world;
        arm_slam::TSDF tsdf;
        ofImage tsdfImg;
        std::vector<float> errs;
        Experiment experimentMode;

};
