#pragma once

#include "ofMain.h"
#include "Robot.h"
#include "World.h"
#include "TSDF.h"

class ofApp: public ofBaseApp
{
    public:
        typedef arm_slam::Robot<3> Robot;
        typedef Robot::Config Config;
        enum Experiment
        {
            GroundTruth,
            Odometry,
            ConstrainedDescent,
            UnconstraintedDescent
        };

        struct ExperimentDatum
        {
                Config robotConfig;
                Config odomConfig;
                Config trackConfig;
                float tsdfError;
                float classificationError;
                float eePosError;
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

        void LoadTrajectory();
        void SaveTrajectory();
        void AppendExperimentDatum();
        void SaveExperimentData();
        Config GetJointNoise(const  arm_slam::Robot<3>::Config& curr);

        Robot robot;
        Robot fakeRobot;
        Robot odomRobot;
        arm_slam::DepthCamera freeCamera;
        Config offset;
        Config zeroCalibration;
        arm_slam::World world;
        arm_slam::TSDF tsdf;
        ofImage tsdfImg;
        std::vector<float> errs;
        Experiment experimentMode;
        float jointNoiseScale;
        bool writeTrajectory;
        bool readTrajectory;
        bool writeExperimentData;
        size_t iter;
        std::vector<Config> recordedTrajectory;
        std::vector<ExperimentDatum> experimentData;
};
