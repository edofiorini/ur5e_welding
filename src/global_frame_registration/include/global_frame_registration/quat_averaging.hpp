#include<tf/tf.h>

namespace quat_averaging
{
    static tf::Quaternion avg(std::vector<tf::Quaternion> rotationList)
    {
        tf::Quaternion qAvg = rotationList[0];
        float weight;

        for (int i = 1; i < rotationList.size(); i++)
        {
            weight = 1.0f / (float)(i + 1);
            qAvg = qAvg.slerp(rotationList[i], weight);
        }

        return qAvg;
    }
};