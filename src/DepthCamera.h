#ifndef DEPTHCAMERA_H_
#define DEPTHCAMERA_H_

#include "Node.h"
#include "World.h"
#include "TSDF.h"

namespace arm_slam
{
    class DepthCamera : public Node
    {
        public:
            DepthCamera() : Node(), resolution(0.025f), minAngle(-0.75f), maxAngle(0.75f)
            {

            }

            DepthCamera(Node* _parent) : Node(), resolution(0.025f), minAngle(-0.75f), maxAngle(0.75f)
            {
                parent  = _parent;
                parent->children.push_back(this);
            }

            virtual ~DepthCamera()
            {

            }

            void Update(arm_slam::World& map)
            {
                points.clear();
                noisyPoints.clear();
                for(float dt = minAngle; dt < maxAngle; dt+=resolution)
                {
                    float t = globalRotation + dt;
                    ofVec2f dir(cos(t), -sin(t));

                    for (float dl = 0; dl < map.data.width * map.data.height; dl+=1)
                    {
                        ofVec2f p = dir * dl + globalTranslation;
                        if(!map.IsValid((int)p.x, (int)p.y))
                        {
                            break;
                        }
                        if(map.Collides((int)p.x, (int)p.y))
                        {
                            ofVec2f relativeP(cos(dt), -sin(dt));
                            relativeP *= dl;
                            points.push_back(relativeP);
                            //relativeP *= ofRandom(0.95f, 1.05f);
                            noisyPoints.push_back(relativeP);
                            break;
                        }
                    }
                }
            }

            template <typename T> void ComputeGradients(T& map, bool noisy)
            {
                if(noisy)
                {
                    ComputeGradients(map, noisyPoints);
                }
                else
                {
                    ComputeGradients(map, points);
                }

            }

            template <typename T> void ComputeGradients(T& map, const std::vector<ofVec2f>& pts)
            {
                gradients.clear();
                for(size_t i = 0; i < pts.size(); i++)
                {
                    ofVec2f global = globalTranslation + pts[i].getRotatedRad(-globalRotation);
                    gradients.push_back(map.GetGradient((int)global.x, (int)global.y));
                }
            }


            template <typename T> void FreeGradientDescent(T& map, int iters, float translationStep, float rotationStep)
            {
                for(int i = 0; i < iters; i++)
                {
                    ofVec2f transGradient(0, 0);
                    float rotGradient = 0.0f;

                    for (size_t i = 0; i < gradients.size(); i++)
                    {
                        const ofVec2f gi = gradients.at(i);
                        const ofVec2f pi = noisyPoints.at(i).getRotatedRad(globalRotation) + globalTranslation;
                        transGradient += gi;

                        ofVec3f gi3(gi.x, gi.y, 0);
                        ofVec3f pi3(pi.x - globalTranslation.x, pi.y - globalTranslation.y, 0);

                        rotGradient += pi3.cross(gi3).z;
                    }

                    if(gradients.size() > 0)
                    {
                        float pointmult = 1.0f / gradients.size();
                        localTranslation -= transGradient * translationStep * pointmult;
                        localRotation -= rotGradient * rotationStep * pointmult;
                        UpdateRecursive();
                        ComputeGradients(map, true);
                    }
                }
            }

            virtual void Draw()
            {
                const bool hasGradients = gradients.size() > 0;
                for(size_t i = 0; i < noisyPoints.size(); i++)
                {
                    ofVec2f pi = globalTranslation + noisyPoints[i].getRotatedRad(-globalRotation);
                    ofSetColor(100, 100, 100);
                    ofSetLineWidth(1);
                    ofLine(globalTranslation, pi);

                    if(hasGradients)
                    {
                        ofSetColor(255, 0, 0);
                        ofSetLineWidth(1);
                        ofLine(pi, pi + gradients[i]);
                    }

                }
            }

            virtual void DrawRecursive()
            {
                Node::DrawRecursive();
            }

            std::vector<ofVec2f> points;
            std::vector<ofVec2f> noisyPoints;
            std::vector<ofVec2f> gradients;
            float resolution;
            float minAngle;
            float maxAngle;
    };
}
#endif // DEPTHCAMERA_H_ 
