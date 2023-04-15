
#ifndef _CHARANIMVIEWER_H
#define _CHARANIMVIEWER_H


#include "quaternion.h"
#include "Viewer.h"
#include "BVH.h"
#include "Skeleton.h"
#include "TransformQ.h"
#include "CharacterController.h"

#include <PhysicalWorld.h>
#include <string.h>

class CharAnimViewer : public Viewer
{
public:
    CharAnimViewer();

    int init();
    int render();
    int update( const float time, const float delta );

	static CharAnimViewer& singleton() { return *psingleton;  }

protected:
	void bvhDrawRec(const chara::BVHJoint& bvh, const Transform& f2w, int f);

    int m_frameNumber, m_animFrame;    

    Skeleton m_ske, ske1, ske2;

    CharacterController controller;

    PhysicalWorld m_world;

	void draw_skeleton(const Skeleton& ske, Transform translation = Translation(0, 0, 0));

private:
	static CharAnimViewer* psingleton;
};



#endif
