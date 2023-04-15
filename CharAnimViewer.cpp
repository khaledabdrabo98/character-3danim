#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "CharAnimViewer.h"

using namespace std;
using namespace chara;


CharAnimViewer* CharAnimViewer::psingleton = NULL;


CharAnimViewer::CharAnimViewer() : Viewer(), m_frameNumber(0)
{
	psingleton = this;
}


int CharAnimViewer::init()
{
    Viewer::init();
    cout<<"==> master_CharAnim/CharAnimViewer"<<endl;
    m_camera.lookat( Point(0,0,0), 1000 );
	m_camera.rotation(180, 0);
    gl.light( Point(300, 300, 300 ) );

    //b_draw_grid = false;

    m_world.setParticlesCount( 10 );

    init_cylinder();
    init_sphere();

    m_ske.init( controller.getCurrectBVH() );
    m_ske.setPose( controller.getCurrectBVH(), -1 ); // met le skeleton a la pose au repos
	
	// SKELETON DISTANCE TESTING
	// ske1.init( controller.getCurrectBVH() );
	// ske2.init( controller.getCurrectBVH() );
	// ske1.setPose( controller.getCurrectBVH(), 20 );
	// ske2.setPose( controller.getCurrectBVH(), 2 );

    return 0;
}


void CharAnimViewer::draw_skeleton(const Skeleton& ske, Transform translation)
{
    // TODO
	Transform cpos = controller.controller2world();
	cpos = cpos * translation;
	for(int i=0; i<ske.numberOfJoint(); i++)
	{
		Point jointPos = ske.getJointPosition(i);
		Transform j2w = Translation(jointPos.x, jointPos.y, jointPos.z) * Scale(3, 3, 3);
		draw_sphere(cpos * j2w);
		if (ske.getParentId(i) != -1) {
			draw_cylinder(cpos(ske.getJointPosition(ske.getParentId(i))), cpos(ske.getJointPosition(i)), 2);
		} 
	}
}


int CharAnimViewer::render()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    Viewer::manageCameraLight();
    gl.camera(m_camera);

	// Affiche les particules physiques (Question 3 : interaction personnage sphere/ballon)
    //m_world.draw();

	// Affiche le skeleton � partir de la structure lin�aire (tableau) Skeleton
    draw_skeleton( m_ske );

	// SKELETON DISTANCE TESTING
	// draw_skeleton( ske1, Translation(-100, 0, 0) );
	// draw_skeleton( ske2, Translation(100, 0, 0) );

    return 1;
}


int CharAnimViewer::update( const float time, const float delta )
{
    // time est le temps ecoule depuis le demarrage de l'application, en millisecondes,
    // delta est le temps ecoule depuis l'affichage de la derniere image / le dernier appel a draw(), en millisecondes.

	m_frameNumber++;

	m_animFrame = m_frameNumber % controller.getCurrectBVH().getNumberOfFrame();

	if (key_state('i')) { controller.accelerate(0.15f); m_animFrame = controller.getTransitionFrame(); }
	if (key_state('k')) { controller.accelerate(-0.15f); m_animFrame = controller.getTransitionFrame(); }
	if (key_state('l')) { controller.turnXZ(-10); }
	if (key_state('j')) { controller.turnXZ(10); }
	if (key_state('x')) { controller.doKick(); }

	if (m_animFrame == -1) 
		m_animFrame = m_frameNumber % controller.getCurrectBVH().getNumberOfFrame();

	// Insures that the Kick animation is only played once 
	if (controller.isKicking() && m_animFrame == 0) {
		controller.doResting();
	}

	controller.update(delta, m_animFrame);

	m_ske.setPose( controller.getCurrectBVH(), m_animFrame );

    m_world.update(0.1f);

	// SKELETON DISTANCE TESTING
	// float dist = distance(ske1, ske2);
	// cout << dist << endl;

    return 0;
}



