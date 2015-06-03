#ifndef ROBOT_H_
#define ROBOT_H_
#include "ofMain.h"
#include <vector>
#include <string>
#include <strstream>
#include "World.h"
#include "Node.h"
#include "Link.h"
#include "Joint.h"
#include "DepthCamera.h"
#include "BasicMat.h"
namespace arm_slam
{
    template <size_t N> class Robot
    {
        public:
            typedef Robot<N> Type;
            typedef BasicMat<2, N> LinearJacobian;
            typedef BasicMat<N, 1> Config;
            Robot() :
                root(0x0),
                camera(0x0)
            {

            }


            virtual ~Robot()
            {
                delete root;

                for(size_t i = 0; i < N; i++)
                {
                    delete joints[i];
                }

                for(size_t i = 0; i < N + 1; i++)
                {
                    delete links[i];
                }

                delete camera;
            }

            void Draw(bool drawCamera)
            {
                root->DrawRecursive();

                if(drawCamera)
                {
                    camera->Draw();
                }
            }

            void Update(arm_slam::World& map)
            {
                for(size_t i = 0; i < N; i++)
                {
                    joints[i]->Update();
                }

                root->UpdateRecursive();

                camera->Update(map);
            }

            inline const Config& GetQ() const
            {
                return q;
            }

            void SetQ(const Config& q_)
            {
                q = q_;
                for(size_t i = 0; i < N; i++)
                {
                    joints[i]->q = q[i];
                    joints[i]->Update();
                }
            }


            void Initialize(float* lengths)
            {
                root = new Node();
                Node* last = root;
                for(size_t i = 0; i < N + 1; i++)
                {
                    if(i < N)
                    {
                        joints[i] = new Joint(last);
                        Joint* joint = joints[i];
                        last = joint;
                    }

                    links[i] = new Link(last, lengths[i], color);
                    Link* link = links[i];
                    last = link;

                }
                camera = new DepthCamera(last);
            }

            BasicMat<2, 1> MatFromVec2(const ofVec2f& vec)
            {
                BasicMat<2, 1> toReturn;
                toReturn[0] = vec.x;
                toReturn[1] = vec.y;
                return toReturn;
            }

            ofVec2f ComputeForwardKinematics(const ofVec2f& eeOffset)
            {
                return links[N]->globalTranslation + eeOffset.rotateRad(links[N]->globalRotation);
            }

            ofVec2f GetEEPos()
            {
                return links[N]->globalTranslation;
            }

            Config ComputeJacobianTransposeMove(const ofVec2f& force)
            {
                ofVec2f ee = GetEEPos();
                LinearJacobian jc =ComputeLinearJacobian(ee);
                return jc.Transpose() * MatFromVec2(force);
            }

            LinearJacobian ComputeLinearJacobian(const ofVec2f& globalPos)
            {
                LinearJacobian jacobian;
                ofVec3f zi = ofVec3f(0, 0, 1);
                ofVec3f on = ofVec3f(globalPos.x, globalPos.y, 0);
                for(size_t i = 0; i < N; i++)
                {
                    Joint* joint = joints[i];
                    ofVec3f oi = ofVec3f(joint->globalTranslation.x, joint->globalTranslation.y, 0);
                    ofVec3f j_vi = zi.crossed(on - oi);
                    jacobian(0, i) = j_vi.x;
                    jacobian(1, i) = j_vi.y;
                }

                return jacobian;

            }

            template <typename T> void GradientDescent(int iters, float rate, T& map)
            {
                for(int i = 0; i < iters; i++)
                {
                    Config gradient;

                    for (size_t i = 0; i < camera->gradients.size(); i++)
                    {
                        const ofVec2f gi = camera->gradients.at(i);
                        const ofVec2f pi = camera->noisyPoints.at(i).getRotatedRad(-camera->globalRotation) + camera->globalTranslation;
                        LinearJacobian J = ComputeLinearJacobian(pi);
                        Config g = J.Transpose() * MatFromVec2(gi);
                        gradient += g;
                    }

                    if(camera->gradients.size() > 0)
                    {
                        float pointmult = 1.0f / camera->gradients.size();

                        SetQ(q + gradient * rate * -1.0f * pointmult);
                        camera->ComputeGradients(map, true);
                    }
                }
            }

            inline size_t GetDOF()
            {
                return N;
            }


            Node* root;
            Joint* joints[N];
            Link* links[N + 1];
            DepthCamera* camera;
            ofColor color;

        protected:
            Config q;
    };

}

#endif // ROBOT_H_ 
