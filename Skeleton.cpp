#include "Skeleton.h"

using namespace chara;

void Skeleton::init(const BVH& bvh)
{
    for(int i=0; i<bvh.getNumberOfJoint(); i++)
    {
        float x, y, z;
        bvh[i].getOffset(x, y, z);
        SkeletonJoint joint;
        joint.m_parentId = bvh[i].getParentId();
        Transform l2f = Translation(x, y, z); //local2father
        if (!bvh[i].isRoot()) {
            joint.m_l2w = m_joints[joint.m_parentId].m_l2w(l2f);
        } else {
            joint.m_l2w = l2f;
        }

        m_joints.push_back(joint);
    }
}


Point Skeleton::getJointPosition(int i) const
{
    return m_joints[i].m_l2w(Point(0, 0, 0));
}


int Skeleton::getParentId(const int i) const
{
    return m_joints[i].m_parentId;
}

// Parcourir toutes les articulations (SkeletonJoint ou BVHJoint) 
//     Declarer la matrice l2f (pere<-local)
//     Init avec la translation Sffset
//     Parcourir tous les channels
//          Accumuler dans la matrice l2f les translations et rotation en fonction du type de Channel
// Multiplier la matrice l2f avec la matrice l2w (world<-local) du p�re qui est d�j� stock� dans le tableau 
// Attention il peut ne pas y avoir de p�re (pour la racine)
void Skeleton::setPose(const BVH& bvh, int frameNumber)
{
    for(signed int i=0; i<bvh.getNumberOfJoint(); i++)
    {
        float x, y, z;
        bvh[i].getOffset(x, y, z);
        Transform l2f = Translation(x, y, z); //local2father

        for(int j=0; j<bvh[i].getNumberOfChannel(); j++) {
            
            BVHChannel channel = bvh[i].getChannel(j);
            AXIS axis = channel.getAxis();
            float data = channel.getData(frameNumber);

            if (channel.isRotation()) {
                if (axis == AXIS::AXIS_X) {
                    l2f = l2f * RotationX(data);
                } else if (axis == AXIS::AXIS_Y) {
                    l2f = l2f * RotationY(data);
                } else if (axis == AXIS::AXIS_Z) {
                    l2f = l2f * RotationZ(data);
                }
            } else { //if (channel.isTranslation()) {
                if (axis == AXIS::AXIS_X) {
                    l2f = l2f * Translation(data, 0, 0);
                } else if (axis == AXIS::AXIS_Y) {
                    l2f = l2f * Translation(0, data, 0);
                } else if (axis == AXIS::AXIS_Z) {
                    l2f = l2f * Translation(0, 0, data);
                }
            }
        }

        // use l2f, l2w 
        if (!bvh[i].isRoot()) { 
            m_joints[i].m_l2w = m_joints[bvh[i].getParentId()].m_l2w(l2f);
        } else {
            m_joints[i].m_l2w = l2f;
        }
    }

}

float distance(const Skeleton& a, const Skeleton& b) {
    if (a.numberOfJoint() != b.numberOfJoint())
        return -1;
    
    float dist = 0.0f;
    Point p1, p2;
    
    for(signed int i=0; i<a.numberOfJoint(); i++) {
        
        p1 = a.getJointPosition(i);
        p2 = b.getJointPosition(i);

        dist += sqrt(pow(p2.x - p1.x, 2) +
                     pow(p2.y - p1.y, 2) +
                     pow(p2.z - p1.z, 2) * 1.0);
    }
    dist = dist / a.numberOfJoint();
    return dist;
}
