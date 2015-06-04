#include "ofApp.h"
#include <fstream>

//--------------------------------------------------------------
void ofApp::setup()
{
    world.data.loadImage("world.png");
    world.distdata.loadImage("dist.png");
    world.Initialize();
    float linkLengths[] = {50.0f, 40.0f, 25.0f};
    robot.color = ofColor(200, 10, 10);
    fakeRobot.color = ofColor(100, 100, 200, 100);
    odomRobot.color = ofColor(100, 200, 100, 100);
    robot.Initialize(linkLengths);
    fakeRobot.Initialize(linkLengths);
    odomRobot.Initialize(linkLengths);
    Robot::Config config = robot.GetQ();
    robot.SetQ(config);
    fakeRobot.SetQ(config);
    odomRobot.SetQ(config);
    robot.root->localTranslation = ofVec2f(ofGetWidth() / 2, ofGetHeight() / 2);
    fakeRobot.root->localTranslation = robot.root->localTranslation;
    odomRobot.root->localTranslation = robot.root->localTranslation;
    tsdf.Initialize(world, 32.0f);
    tsdfImg.allocate(tsdf.width, tsdf.height, OF_IMAGE_COLOR_ALPHA);
    tsdf.SetColors(&tsdfImg);
    experimentMode = UnconstraintedDescent;
    jointNoiseScale = 0.25f;
    zeroCalibration = GetJointNoise(robot.GetQ());
    writeTrajectory = false;
    readTrajectory = true;
    writeExperimentData = true;
    iter = 0;

    if (readTrajectory)
    {
        LoadTrajectory();
    }
}

void ofApp::LoadTrajectory()
{
    std::ifstream inputStream;
    inputStream.open("./data/traj.txt", std::ios::in);

    while (!inputStream.eof())
    {
        float a, b, c;
        inputStream >> a;
        inputStream >> b;
        inputStream >> c;
        Config config;
        config(0) = a;
        config(1) = b;
        config(2) = c;
        recordedTrajectory.push_back(config);
    }
}

ofApp::Robot::Config ofApp::GetJointNoise(const  Robot::Config& curr)
{
    Robot::Config perturbation;
    perturbation[0] = jointNoiseScale * (ofNoise((float)curr(0), (float)curr(1), (float)curr(2) + 0.5f) - 0.5f);
    perturbation[1] = jointNoiseScale * (ofNoise((float)curr(0), (float)curr(1) + 0.5f, (float)curr(2)) - 0.5f);
    perturbation[2] = jointNoiseScale * (ofNoise((float)curr(0) + 0.5f, (float)curr(1), (float)curr(2)) - 0.5f);
    return perturbation;
}

void ofApp::AppendExperimentDatum()
{
    ExperimentDatum datum;
    datum.odomConfig = odomRobot.GetQ();
    datum.trackConfig = fakeRobot.GetQ();
    datum.robotConfig = robot.GetQ();

    ofVec2f truePos = robot.GetEEPos();
    ofVec2f trackPos = fakeRobot.GetEEPos();
    switch (experimentMode)
    {
        case GroundTruth:
        case ConstrainedDescent:
        case Odometry:
            datum.eePosError = (truePos - trackPos).length();
            break;
        case UnconstraintedDescent:
            datum.eePosError = (truePos - freeCamera.globalTranslation).length();
            break;
    }

    tsdf.ComputeError(world, datum.classificationError, datum.tsdfError);
    experimentData.push_back(datum);
}

void ofApp::SaveExperimentData()
{
    std::ofstream stream;
    stream.open("./data/experiment.txt", std::ios::out);

    for (size_t i = 0; i < experimentData.size(); i++)
    {
        ExperimentDatum& datum = experimentData.at(i);
        stream << datum.tsdfError << " " << datum.classificationError << " " << datum.eePosError
                << " " << datum.odomConfig(0) << " " << datum.odomConfig(1) << " " << datum.odomConfig(2)
                << " " << datum.trackConfig(0) << " " << datum.trackConfig(1) << " " << datum.trackConfig(2)
                << " " << datum.robotConfig(0) << " " << datum.robotConfig(1) << " " << datum.robotConfig(2) << std::endl;
    }
}

//--------------------------------------------------------------
void ofApp::update()
{
    ofVec2f odomEE = odomRobot.GetEEPos();
    float odomRotation = odomRobot.camera->globalRotation;
    if((mouseX > 0 && mouseY > 0) || readTrajectory)
    {
        Robot::Config curr;
        if(!readTrajectory)
        {
            ofVec2f ee = robot.GetEEPos();
            ofVec2f force = ee - ofVec2f(mouseX, mouseY);
            Robot::Config vel = robot.ComputeJacobianTransposeMove(force);
            curr = robot.GetQ();
            robot.SetQ(curr + vel * 1e-5);
        }
        else
        {
            if(iter < recordedTrajectory.size())
            {
                robot.SetQ(recordedTrajectory[iter]);
                curr = robot.GetQ();
            }
            else
            {
                SaveExperimentData();
                exit();
            }
        }
        robot.Update(world);
        Robot::Config perturbation = GetJointNoise(curr) + zeroCalibration * -1.0f;
        switch (experimentMode)
        {
            case GroundTruth:
            {
                fakeRobot.SetQ(robot.GetQ());
                Robot::Config odom = robot.GetQ() + perturbation;
                odomRobot.SetQ(odom);
                break;
            }
            case Odometry:
            case UnconstraintedDescent:
            case ConstrainedDescent:
                Robot::Config fake = robot.GetQ() + offset + perturbation;
                fakeRobot.SetQ(fake);
                Robot::Config odom = robot.GetQ() + perturbation;
                odomRobot.SetQ(odom);
                break;
        }
        iter++;
    }

    fakeRobot.Update(world);
    odomRobot.Update(world);

    ofVec2f odomEEAfter = odomRobot.GetEEPos();
    float odomRotationAfter = odomRobot.camera->globalRotation;

    robot.camera->ComputeGradients(world, false);
    fakeRobot.camera->points = robot.camera->points;
    fakeRobot.camera->noisyPoints = robot.camera->noisyPoints;
    fakeRobot.camera->ComputeGradients(tsdf, true);
    //fakeRobot.camera->gradients = robot.camera->gradients;
    switch(experimentMode)
    {
        case GroundTruth:
        {
            fakeRobot.SetQ(robot.GetQ());
            break;
        }
        case Odometry:
        {
            break;
        }
        case ConstrainedDescent:
        {
            fakeRobot.GradientDescent(100, -1e-7, tsdf);
            break;
        }
        case UnconstraintedDescent:
        {
            freeCamera.localRotation += (odomRotationAfter - odomRotation);
            freeCamera.localTranslation += (odomEEAfter - odomEE);
            freeCamera.points = robot.camera->points;
            freeCamera.noisyPoints = robot.camera->noisyPoints;
            freeCamera.UpdateRecursive();
            freeCamera.ComputeGradients(tsdf, true);
            freeCamera.FreeGradientDescent(tsdf, 100, 0.5f, -1e-6);
            break;
        }
    }

    offset = fakeRobot.GetQ() + odomRobot.GetQ() * -1.0f;

    if(errs.size() > 500)
    {
        errs.erase(errs.begin());
    }

    Robot::Config delta = fakeRobot.GetQ() + robot.GetQ() * -1;

    float err = (delta.Transpose() * delta)[0];
    errs.push_back(err);

    switch(experimentMode)
    {
        case ConstrainedDescent:
        case GroundTruth:
        case Odometry:
        {
            tsdf.FuseRayCloud(fakeRobot.camera->globalTranslation, fakeRobot.camera->globalRotation, fakeRobot.camera->noisyPoints, robot.camera->gradients);
            tsdf.SetColors(&tsdfImg);
            break;
        }
        case UnconstraintedDescent:
        {
            tsdf.FuseRayCloud(freeCamera.globalTranslation, freeCamera.globalRotation, freeCamera.noisyPoints,  robot.camera->gradients);
            tsdf.SetColors(&tsdfImg);
        }
    }

    if (writeTrajectory)
    {
        recordedTrajectory.push_back(robot.GetQ());
    }

    if (readTrajectory)
    {
        AppendExperimentDatum();
    }
}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofHideCursor();
    ofClear(0);
    ofSetColor(255, 255, 255);
    world.data.draw(0, 0);
    tsdfImg.draw(0, 0);
    robot.Draw(false);
    switch(experimentMode)
    {
        case ConstrainedDescent:
        case GroundTruth:
        case Odometry:
            fakeRobot.Draw(true);
            break;
        case UnconstraintedDescent:
            freeCamera.Draw();
            break;
    }
    odomRobot.Draw(false);
    ofSetLineWidth(1);
    ofSetColor(0, 100, 100);
    ofVec2f ee = robot.GetEEPos();
    ofLine(mouseX, mouseY, ee.x, ee.y);


    /*
    ofSetColor(255, 0, 0);
    for(size_t i = 0; i < errs.size() - 1; i++)
    {
        ofLine(i, ofGetHeight() - errs[i] * 100, i + 1, ofGetHeight() - errs[i + 1] * 100);
    }

    if(errs.size() > 0)
    {
        ofSetColor(0, 0, 0);
        std::stringstream ss;
        ss << errs[errs.size() - 1];
        ofDrawBitmapString(ss.str(), 1, 15);
    }
    */
}

void ofApp::SaveTrajectory()
{
    std::ofstream stream;
    stream.open("./data/traj.txt", std::ios::out);

    for (size_t i = 0; i < recordedTrajectory.size(); i++)
    {
        Config& config = recordedTrajectory.at(i);
        stream << config(0) << " " << config(1) << " " << config(2) << std::endl;
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    if(key == 'r')
    {
        Robot::Config curr = fakeRobot.GetQ();
        Robot::Config randConfig;
        randConfig[0] = ofRandom(-0.1f, 0.1f);
        randConfig[1] = ofRandom(-0.1f, 0.1f);
        randConfig[2] = ofRandom(-0.1f, 0.1f);
        fakeRobot.SetQ(curr + randConfig);
    }

    if (key == 's' && writeTrajectory)
    {
        SaveTrajectory();
        exit();
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{

}
