#include "ofApp.h"

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
    arm_slam::Robot<3>::Config config = robot.GetQ();
    robot.SetQ(config);
    fakeRobot.SetQ(config);
    odomRobot.SetQ(config);
    robot.root->localTranslation = ofVec2f(ofGetWidth() / 2, ofGetHeight() / 2);
    fakeRobot.root->localTranslation = robot.root->localTranslation;
    odomRobot.root->localTranslation = robot.root->localTranslation;


}

//--------------------------------------------------------------
void ofApp::update()
{

    if(mouseX > 0 && mouseY > 0)
    {
        ofVec2f ee = robot.GetEEPos();
        ofVec2f force = ee - ofVec2f(mouseX, mouseY);
        arm_slam::Robot<3>::Config vel = robot.ComputeJacobianTransposeMove(force);
        arm_slam::Robot<3>::Config curr = robot.GetQ();
        robot.SetQ(curr + vel * 1e-5);
        robot.Update(world);
        arm_slam::Robot<3>::Config perturbation = vel * 1e-5 * ofRandom(0.1f, 1.9f);
        arm_slam::Robot<3>::Config fake = fakeRobot.GetQ() + perturbation;
        fakeRobot.SetQ(fake);
        arm_slam::Robot<3>::Config odom = odomRobot.GetQ() + perturbation;
        odomRobot.SetQ(odom);
    }

    fakeRobot.Update(world);
    odomRobot.Update(world);

    fakeRobot.camera->points = robot.camera->points;
    fakeRobot.camera->ComputeGradients(world);
    fakeRobot.GradientDescent(1, -1e-6);

    if(errs.size() > 500)
    {
        errs.erase(errs.begin());
    }

    arm_slam::Robot<3>::Config delta = fakeRobot.GetQ() + robot.GetQ() * -1;

    float err = (delta.Transpose() * delta)[0];
    errs.push_back(err);

}

//--------------------------------------------------------------
void ofApp::draw()
{
    ofHideCursor();
    ofClear(0);
    ofSetColor(255, 255, 255);
    world.data.draw(0, 0);
    robot.Draw(false);
    fakeRobot.Draw(true);
    odomRobot.Draw(false);
    ofSetLineWidth(1);
    ofSetColor(0, 100, 100);
    ofVec2f ee = robot.GetEEPos();
    ofLine(mouseX, mouseY, ee.x, ee.y);


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
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
    if(key == 'r')
    {
        arm_slam::Robot<3>::Config curr = fakeRobot.GetQ();
        arm_slam::Robot<3>::Config randConfig;
        randConfig[0] = ofRandom(-0.1f, 0.1f);
        randConfig[1] = ofRandom(-0.1f, 0.1f);
        randConfig[2] = ofRandom(-0.1f, 0.1f);
        fakeRobot.SetQ(curr + randConfig);
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
