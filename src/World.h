#ifndef WORLD_H_
#define WORLD_H_

#include "ofMain.h"

namespace arm_slam
{

    class World
    {
        public:
            World();
            virtual ~World();

            bool IsValid(int x, int y)
            {
                return x >= 0 && x < data.getWidth() && y >= 0 && y < data.getHeight();
            }

            bool Collides(int x, int y)
            {
                if(IsValid(x, y))
                {
                    return collisionBuffer[(x + y * data.getWidth())];
                }
                return true;
            }

            float GetDist(int x, int y)
            {
                if(IsValid(x, y))
                {
                    return signedDistance[(x + y * data.getWidth())];
                }
                else
                {
                    return 0.0f;
                }
            }

            ofVec2f GetGradient(int x, int y)
            {
                /*
                float d0 = GetDist(x, y);
                float dx = GetDist(x + 1, y);
                float dy = GetDist(x, y + 1);
                return ofVec2f(dx - d0, dy - d0) * d0;
                */

                int dxplus = (int)Collides(x + 1, y);
                int dyplus = (int)Collides(x, y + 1);
                int dxminus = (int)Collides(x - 1, y);
                int dyminus = (int)Collides(x, y - 1);
                return ofVec2f((dxplus - dxminus) * 0.5f, (dyplus - dyminus) * 0.5f);

            }

            void Initialize()
            {
                collisionBuffer.resize(data.getWidth() * data.getHeight());
                signedDistance.resize(distdata.getWidth() * distdata.getHeight());
                for (int x = 0; x< data.getWidth(); x++)
                {
                    for(int y = 0; y < data.getHeight(); y++)
                    {
                        collisionBuffer[x + y * data.getWidth()] = data.getColor(x, y).r == 0;
                        ofColor dist = distdata.getColor(x, y);
                        signedDistance[x + y * data.getWidth()] = (float)dist.r - (float)dist.g;
                    }
                }
            }

            ofImage data;
            ofImage distdata;
            std::vector<bool> collisionBuffer;
            std::vector<float>signedDistance;
    };

}

#endif // WORLD_H_
