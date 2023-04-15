#ifndef _MOTIONGRAPH_H
#define _MOTIONGRAPH_H


#include "Viewer.h"
#include "BVH.h"
#include "Skeleton.h"

const float SEUIL = 20.0f;

class MotionGraph
{
    public:

    MotionGraph() {};

    void init(std::vector<chara::BVH> bvh) {
        for(unsigned int i=0; i<bvh.size(); i++) {
            m_BVH.push_back(bvh[i]);
        }

        for(unsigned int i=0; i<m_BVH.size(); i++) {
            for(int nbframe=0; nbframe<m_BVH[i].getNumberOfFrame(); nbframe++) {
                GrapheNode node;
                node.id_bvh = i;
                node.frame = nbframe;
                m_GrapheNode.push_back(node);
            }
        }
        computeMachingStates();
    }

    void computeMachingStates() {
        Skeleton ske1, ske2;
        float d;
        // compare all bvhs to all the other ones
        for (unsigned int i=0; i<m_BVH.size(); i++) {
            for (unsigned int j=0; j<m_BVH.size(); j++) {
                
                if (i != j) { // && i < j 
                    ske1.init(m_BVH.at(i));
                    ske2.init(m_BVH.at(j));
                    // compare two bvh frame by frame
                    for (int nframe1=0; nframe1<m_BVH.at(i).getNumberOfFrame(); nframe1++){
                        for (int nframe2=0; nframe2<m_BVH.at(j).getNumberOfFrame(); nframe2++) {
                            
                            ske1.setPose(m_BVH.at(i), nframe1);
                            ske2.setPose(m_BVH.at(j), nframe2);
                            // compute distance between two skeletons
                            d = distance(ske1, ske2);
                            // d = abs(d);
                            
                            if (d != -1 && d < SEUIL) {
                                int nodeid1 = getPreviousBVHFrameSizes(i, nframe1);
                                int nodeid2 = getPreviousBVHFrameSizes(j, nframe2);
                                m_GrapheNode.at(nodeid1).ids_next.push_back(nodeid2);
                            }
                        }
                    }
                }
            }
        }
    }

    // Searching the Motion Graph for the best frame to transition to from the current frame 
    int getTransitionFromTo(int bvh_from, int bvh_to, int frameNumber) {
        int transition_frame = -1;
        int from = getPreviousBVHFrameSizes(bvh_from, frameNumber);
        int to = getPreviousBVHFrameSizes(bvh_to);
        int nbframes_bvhto = m_BVH.at(bvh_to).getNumberOfFrame();
        std::vector<GrapheNodeID> ids_next = m_GrapheNode.at(from).ids_next;
 
        // std::cout << "bvh_from: " << bvh_from << std::endl;
        // std::cout << "bvh_to: " << bvh_to << std::endl;
        // std::cout << "from: " << from << std::endl;
        // std::cout << "to: " << to << std::endl << std::endl;

        // look for best frame to transition into bvh_to
        for (int f=to; f<to+nbframes_bvhto; f++) { 
            if (std::find(ids_next.begin(), ids_next.end(), f)!=ids_next.end()) {
                if (transition_frame = -1)
                    transition_frame = m_GrapheNode.at(f).frame;
            }
        }

        // std::cout << transition_frame << std::endl;
        return transition_frame;
    }

    int getPreviousBVHFrameSizes(unsigned int bvh_index, int frame=0) {
        int sizes = 0;
        for (unsigned int i=0; i<bvh_index; i++) {
            sizes += ( m_BVH.at(i).getNumberOfFrame() );
        }
        return (sizes + frame);
    }

    void print() {
        for(unsigned int i=0; i<m_GrapheNode.size(); i++) {
            std::cout << "[" << i << "]: { " ;
            for(unsigned int j=0; j<m_GrapheNode.at(i).ids_next.size(); j++) {
                std::cout << m_GrapheNode.at(i).ids_next.at(j);
                if (j != m_GrapheNode.at(i).ids_next.size()-1)
                    std::cout << ", ";
            }
            std::cout << " } " << std::endl;
        }
    }

    protected:
        //! L'ensemble des BVH du graphe d'animation
        std::vector<chara::BVH> m_BVH;

        //! Un noeud du graphe d'animation est repéré par un entier=un identifiant
        typedef int GrapheNodeID;

        //! Une animation BVH est repérée par un identifiant=un entier 
        typedef int BVH_ID;
        
        //! Un noeud du graphe contient l'identifiant de l'animation, le numéro 
        //! de la frame et les identifiants des noeuds successeurs 
        //! Remarque : du code plus "joli" aurait créer une classe GrapheNode
        struct GrapheNode
        {
            BVH_ID id_bvh;
            int frame;
            std::vector<GrapheNodeID> ids_next;     //! Liste des nœuds successeurs 
        };

        //! Tous les noeuds du graphe d'animation
        std::vector<GrapheNode> m_GrapheNode;

};

#endif