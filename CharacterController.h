#ifndef _CHARACTERCONTROLLER_H
#define _CHARACTERCONTROLLER_H


#include "Viewer.h"
#include "BVH.h"
#include "MotionGraph.h"

#include <cstdlib>
#include <string.h>

const int NB_POSES = 4;
const int vMaxWalking = 5;
const int vMaxRunning = 15;

class CharacterController
{
    public:
        enum STATE
	    {
            STATE_RESTING,
            STATE_WALKING,
            STATE_RUNNING,
            STATE_DO_KICK,
	    };

        CharacterController() : m_v(0), m_vMax(vMaxWalking), v_clicks(0), 
                    moving(false), state(STATE_RESTING), prev_state(STATE_DO_KICK)
        {
            std::string poses[NB_POSES] = { "null", "marcher", "courir", "frapper" };
            for(unsigned int i=0; i<NB_POSES; i++) { 
                std::string smpath = "data/bvh/motionGraph/" + poses[i] + ".bvh";
                const char* smartpath = smpath.c_str(); 
                std::cout << smpath << std::endl;
                m_bvh[i].init( smart_path(smartpath) );

                std::cout << std::endl << "========================"<< std::endl;
                std::cout << "BVH " << poses[i].c_str() <<" description: " << std::endl;
                std::cout << m_bvh[i] << std::endl;
            }
            std::cout << std::endl << "========================" << std::endl;

            motionG.init(getAllBVH());
            // motionG.print();
            
            prev_setting.first = prev_state;
            prev_setting.second = state;
        }

        void update(const float dt, int frameNumber) {
            if (m_v > m_vMax) {
                m_v = m_vMax;
            } else if (m_v < 0.0f) {
                m_v = 0.0f;
            }
            
            if (moving) {
                m_v += m_v * (1.0f / dt);
            } else {
                m_v -= m_v * (1.0f / dt);
            }
            
            // check if state changed, look for best transition frame
            if (prev_setting.first != prev_state || prev_setting.second != state) {
                prev_setting.first = prev_state;
                prev_setting.second = state;
                frame = motionG.getTransitionFromTo(prev_state, state, frameNumber);

                // std::cout << "state changed " << state << ", " << prev_state << std::endl;
                //std::cout << frameNumber << " -> " << frame <<std::endl;
            }

            // std::cout << m_v << std::endl;
            m_ch2w = m_ch2w * Translation(m_v/5, 0, 0);
        }

        void turnXZ(const float& rot_angle_v) {
            m_ch2w  = m_ch2w * RotationY(rot_angle_v);
        }

        void accelerate(const float& speed_inc) {
            v_clicks += speed_inc;

            if (v_clicks > 2.0f) {
                v_clicks = 2.0f;
            } else if (v_clicks <= 0.1f) {
                v_clicks = 0.0f;
                moving = false;
                state = STATE_RESTING;
                prev_state = STATE_WALKING;
            }

            if (v_clicks > 0.1f && v_clicks < 1.0f) {
                m_vMax = vMaxWalking;
                moving = true;
                if (state == STATE_RUNNING) {
                    prev_state = STATE_RUNNING;
                } else if (state == STATE_RESTING) {
                    prev_state = STATE_RESTING;
                }
                state = STATE_WALKING;
            } else if (v_clicks > 1.0f && v_clicks <= 2.0f) {
                m_vMax = vMaxRunning;
                moving = true;
                state = STATE_RUNNING;
                prev_state = STATE_WALKING;
            }

            m_v += speed_inc;
        }

        std::vector<chara::BVH> getAllBVH() {
            std::vector<chara::BVH> bvh;
            for(unsigned int i=0; i<NB_POSES; i++) {
                bvh.push_back(m_bvh[i]);
            }
            return bvh;
        }

        chara::BVH getCurrectBVH() {
            return m_bvh[state];
        }

        void setVelocityMax(const float vmax) {
            m_vMax = vmax;
        }

        const Point position() const {
            return m_ch2w(Point(0, 0, 0));
        }

        const Vector direction() const {
            return m_v * m_ch2w(Vector(1, 0, 0));
        }

        void doKick() {
            if (state != STATE_DO_KICK) {
                prev_state = state;
                state = STATE_DO_KICK;
            }

            // stop/reset character movement
            m_v = 0;
            v_clicks = 0;
            m_vMax = vMaxWalking;
        }

        void doResting() {
            prev_state = STATE_DO_KICK;
            state = STATE_RESTING;
        }

        STATE getCurrentState() {
            return state;
        }

        STATE getPrevState() {
            return prev_state;
        }

        bool isKicking() {
            return (state == STATE_DO_KICK);
        }

        int getTransitionFrame() const {
            return frame;
        }

        float velocity() const {
            return m_v;
        }

        const Transform& controller2world() const { return m_ch2w; }

    protected:
        Transform m_ch2w;   // matrice du character vers le monde
                            // le personnage se déplace vers X
                            // il tourne autour de Y
                            // Z est sa direction droite

        float m_v;          // le vecteur vitesse est m_v * m_ch2w * Vector(1,0,0)
        float m_vMax;       // ne peut pas accélérer plus que m_vMax
        float v_clicks;
        std::pair<STATE, STATE> prev_setting;
        bool inTransition;
        bool moving;
        STATE state, prev_state;
        chara::BVH m_bvh[NB_POSES]; 

        MotionGraph motionG; 
        int frame;          // Transition frame extracted from Motion Graph

};

#endif
