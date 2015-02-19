#ifndef ROBOT_H_
#define ROBOT_H_
#include "ofMain.h"
#include <vector>
#include <string>
#include <strstream>
#include "World.h"
namespace arm_slam
{

    class Node
    {
        public:
            Node() :
                parent(0x0),
                localTranslation(ofVec2f(0, 0)),
                globalTranslation(ofVec2f(0, 0)),
                localRotation(0.0f),
                globalRotation(0.0f)
            {

            }

            virtual ~Node()
            {

            }

            void UpdateRecursive()
            {
                if(!parent)
                {
                   globalTranslation = localTranslation;
                   globalRotation = localRotation;
                }
                else
                {
                    globalRotation = parent->globalRotation + localRotation;
                    globalTranslation = parent->globalTranslation + localTranslation.getRotatedRad(-globalRotation);
                }

                for (size_t i = 0; i < children.size(); i++)
                {
                    Node* node = children.at(i);
                    node->UpdateRecursive();
                }
            }

            virtual void DrawRecursive()
            {
                for (size_t i = 0; i < children.size(); i++)
                {
                    Node* node = children.at(i);
                    node->DrawRecursive();
                }
            }


            Node* parent;
            std::vector<Node*> children;
            ofVec2f localTranslation;
            ofVec2f globalTranslation;
            float localRotation;
            float globalRotation;
    };

    class Link : public Node
    {
        public:
            Link() : Node()
            {

            }
            Link(Node* _parent, float length, ofColor c) : Node()
            {
                    color = c;
                    parent = _parent;
                    parent->children.push_back(this);
                    localTranslation = ofVec2f(length, 0);
            }

            virtual ~Link()
            {

            }

            virtual void DrawRecursive()
            {
                ofSetColor(color);
                ofSetLineWidth(4);
                if(parent == 0x0)
                {
                    return;
                }
                else
                {

                    ofLine(parent->globalTranslation, globalTranslation);
                }
                Node::DrawRecursive();
            }

            ofColor color;
    };

    class Joint : public Node
    {
        public:
            Joint() : Node(), q(0)
            {

            }
            Joint(Node* _parent) : Node(), q(0)
            {
                parent = _parent;
                parent->children.push_back(this);
            }

            virtual ~Joint()
            {

            }

            void Update()
            {
                localRotation = q;
            }

            float q;
    };


    class DepthCamera : public Node
    {
        public:
            DepthCamera() : Node(), resolution(0.05f), minAngle(-0.5f), maxAngle(0.5f)
            {

            }

            DepthCamera(Node* _parent) : Node(), resolution(0.05f), minAngle(-0.5f), maxAngle(0.5f)
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
                            relativeP *= dl * ofRandom(0.9f, 1.1f);
                            points.push_back(relativeP);
                            break;
                        }
                    }
                }
            }

            void ComputeGradients(arm_slam::World& map)
            {
                gradients.clear();
                for(size_t i = 0; i < points.size(); i++)
                {
                    ofVec2f global = globalTranslation + points[i].getRotatedRad(-globalRotation);
                    gradients.push_back(map.GetGradient((int)global.x, (int)global.y));
                }
            }


            virtual void Draw()
            {
                for(size_t i = 0; i < points.size(); i++)
                {
                    ofVec2f pi = globalTranslation + points[i].getRotatedRad(-globalRotation);
                    ofSetColor(100, 100, 100);
                    ofSetLineWidth(1);
                    ofLine(globalTranslation, pi);


                    ofSetColor(255, 0, 0);
                    ofSetLineWidth(1);
                    ofLine(pi, pi + gradients[i]);

                }
            }

            virtual void DrawRecursive()
            {
                Node::DrawRecursive();
            }

            std::vector<ofVec2f> points;
            std::vector<ofVec2f> gradients;
            float resolution;
            float minAngle;
            float maxAngle;
    };

    template <size_t N, size_t M> class BasicMat
    {
        public:
            typedef BasicMat<N, M> Type;
            BasicMat()
            {
                   for(size_t i = 0; i < N * M; i++)
                   {
                       m[i] = 0.0f;
                   }
            }

            ~BasicMat()
            {

            }

            inline std::string ToString()
            {
                std::stringstream ss;

                for (size_t r = 0; r < N; r++)
                {
                    for (size_t c = 0; c < M; c++)
                    {
                        ss << (*this)(r, c) << " ";
                    }

                    ss << "\n";
                }

                return ss.str();
            }

            inline size_t GetNumRows()
            {
                return N;
            }

            inline size_t GetNumCols()
            {
                return M;
            }

            float& operator[](size_t idx)
            {
                return m[idx];
            }

            const float& operator[](size_t idx) const
            {
                return m[idx];
            }

            float& operator()(size_t r, size_t c)
            {
                return (*this)[r * M + c];
            }

            const float& operator()(size_t r, size_t c) const
            {
                return (*this)[r * M + c];
            }

            void operator=(const Type& other)
            {
                for(size_t i = 0; i < N * M; i++)
                 {
                     m[i] = other[i];
                 }
            }

            /*
            Type operator=(const Type& other)
            {
                Type toReturn = *this;
                for(size_t i = 0; i < N * M; i++)
                {
                    toReturn[i] = other[i];
                }
                return toReturn;
            }
            */

            void operator+=(const Type& other)
            {
                for(size_t i = 0; i < N * M; i++)
                {
                    m[i] += other[i];
                }
            }

            friend Type operator+(Type lhs, const Type& rhs)
            {
                Type toReturn = lhs;
                toReturn += rhs;
                return toReturn;
            }

            Type operator*=(const float scalar)
            {
                Type toReturn = *this;
                for(size_t i = 0; i < N * M; i++)
                {
                    toReturn[i] *= scalar;
                }
                return toReturn;
            }

            friend Type operator*(Type lhs, const float& rhs)
            {
                return lhs *= rhs;
            }

            inline BasicMat<M, N> Transpose() const
            {
                BasicMat<M, N> toReturn;

                for(size_t r = 0; r < N; r++)
                {
                    for(size_t c = 0; c < M; c++)
                    {
                        toReturn(c, r) = (*this)(r, c);
                    }
                }
                return toReturn;
            }

            template <size_t N2> BasicMat<N2, M> PreMult(const BasicMat<N2, N>& lhs)
            {
                    BasicMat<N2, M> toReturn;

                    for (size_t r = 0; r < N2; r++)
                    {
                        for(size_t c = 0; c < M; c++)
                        {
                            for(size_t k = 0; k < N; k++)
                            {
                                toReturn[r, c] += lhs(r, k) * (*this)(k, r);
                            }
                        }
                    }
                    return toReturn;
            }

            template <size_t M2> BasicMat<N, M2> PostMult(const BasicMat<M, M2>& rhs)
            {
                BasicMat<N, M2> toReturn;

                for (size_t r = 0; r < N; r++)
                {
                    for(size_t c = 0; c < M2; c++)
                    {
                        for(size_t k = 0; k < M; k++)
                        {
                            toReturn(r, c) += (*this)(r, k) * rhs(k, c);
                        }
                    }
                }
                return toReturn;
            }

            template <size_t M2> BasicMat<N, M2> operator*=(const BasicMat<M, M2>& rhs)
            {
                return PostMult(rhs);
            }

            template <size_t M2> friend BasicMat<N, M2> operator*(Type lhs, const BasicMat<M, M2>& rhs)
            {
                return lhs *= rhs;
            }

            float m[N * M];
    };


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
                camera->ComputeGradients(map);
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

            void GradientDescent(int iters, float rate)
            {
                for(int i = 0; i < iters; i++)
                {
                    Config gradient;

                    printf("%lu\n", camera->gradients.size());
                    for (size_t i = 0; i < camera->gradients.size(); i++)
                    {
                        const ofVec2f gi = camera->gradients.at(i);
                        const ofVec2f pi = camera->points.at(i).getRotatedRad(-camera->globalRotation) + camera->globalTranslation;
                        LinearJacobian J = ComputeLinearJacobian(pi);
                        Config g = J.Transpose() * MatFromVec2(gi);
                        gradient += g;
                    }

                    SetQ(q + gradient * rate * -1.0f);
                    printf("%s\n", gradient.ToString().c_str());
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
