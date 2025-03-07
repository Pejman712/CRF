/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include "Mapper3d/NormalsColorOctree.hpp"

#include <string>
#include <vector>

namespace crf {
namespace applications {
namespace mapper3d {


// node implementation  --------------------------------------
std::ostream& NormalsColorOctreeNode::writeData(std::ostream &s) const {
    s.write((const char*) &value, sizeof(value));   //NOLINT
    s.write((const char*) &color, sizeof(Color));  //NOLINT
    s.write((const char*) &normals, sizeof(Normals));  //NOLINT

    return s;
    }

std::istream& NormalsColorOctreeNode::readData(std::istream &s) {
    s.read((char*) &value, sizeof(value));  //NOLINT
    s.read((char*) &color, sizeof(Color));  //NOLINT
    s.read((char*) &normals, sizeof(Normals));  //NOLINT

    return s;
}

  NormalsColorOctreeNode::Color NormalsColorOctreeNode::getAverageChildColor() const {
    int mr = 0;
    int mg = 0;
    int mb = 0;
    int c = 0;

    if (children != NULL) {
        for (int i = 0; i < 8; i++) {
        NormalsColorOctreeNode* child = static_cast<NormalsColorOctreeNode*>(children[i]);

            if (child != NULL && child->isColorSet()) {
                mr += child->getColor().r;
                mg += child->getColor().g;
                mb += child->getColor().b;
                ++c;
            }
        }
    }

    if (c > 0) {
          mr /= c;
          mg /= c;
          mb /= c;
        return Color((uint8_t) mr, (uint8_t) mg, (uint8_t) mb);
    } else {  // no child had a color other than white
        return Color(255, 255, 255);
    }
  }


  void NormalsColorOctreeNode::updateColorChildren() {
    color = getAverageChildColor();
  }

  NormalsColorOctreeNode::Normals NormalsColorOctreeNode::getAverageChildNormals() const {
    float mnx = 0.0;
    float mny = 0.0;
    float mnz = 0.0;
    int c = 0;

    if (children != NULL) {
      for (int i = 0; i < 8; i++) {
        NormalsColorOctreeNode* child = static_cast<NormalsColorOctreeNode*>(children[i]);

        if (child != NULL && child->isNormalsSet()) {
          mnx += child->getNormals().normal_x;
          mny += child->getNormals().normal_y;
          mnz += child->getNormals().normal_z;
          ++c;
        }
      }
    }

    if (c > 0) {
        mnx /= c;
        mny /= c;
        mnz /= c;
      return Normals(mnx, mny, mnz);
    } else {  // no child had a normal value other than (0, 0, 0)
        return Normals(0.0, 0.0, 0.0);
    }
  }

  void NormalsColorOctreeNode::updateNormalsChildren() {
    normals = getAverageChildNormals();
  }


  // tree implementation  --------------------------------------
  NormalsColorOctree::NormalsColorOctree(double in_resolution)
  : OccupancyOcTreeBase<NormalsColorOctreeNode>(in_resolution) {
    NormalsColorOctreeMemberInit.ensureLinking();
  }

  NormalsColorOctreeNode* NormalsColorOctree::setNodeColor(const octomap::OcTreeKey& key,
                                             uint8_t r,
                                             uint8_t g,
                                             uint8_t b) {
    NormalsColorOctreeNode* n = search(key);
    if (n != 0) {
      n->setColor(r, g, b);
    }
    return n;
  }

    NormalsColorOctreeNode* NormalsColorOctree::setNodeNormals(const octomap::OcTreeKey& key,
                                             float normal_x,
                                             float normal_y,
                                             float normal_z) {
    NormalsColorOctreeNode* n = search(key);
    if (n != 0) {
      n->setNormals(normal_x, normal_y, normal_z);
    }
    return n;
  }

  bool NormalsColorOctree::pruneNode(NormalsColorOctreeNode* node) {
    if (!isNodeCollapsible(node))
      return false;

    // set value to children's values (all assumed equal)
    node->copyData(*(getNodeChild(node, 0)));

    if (node->isColorSet())  // TODO(svillanu) check
      node->setColor(node->getAverageChildColor());

    if (node->isNormalsSet())  // TODO(svillanu) check
      node->setNormals(node->getAverageChildNormals());

    // delete children
    for (unsigned int i = 0; i < 8; i++) {
      deleteNodeChild(node, i);
    }
    delete[] node->children;
    node->children = NULL;

    return true;
  }

  bool NormalsColorOctree::isNodeCollapsible(const NormalsColorOctreeNode* node) const {
    // all children must exist, must not have children of
    // their own and have the same occupancy probability
    if (!nodeChildExists(node, 0))
      return false;

    const NormalsColorOctreeNode* firstChild = getNodeChild(node, 0);
    if (nodeHasChildren(firstChild))
      return false;

    for (unsigned int i = 1; i < 8; i++) {
      // compare nodes only using their occupancy, ignoring color and normals for pruning
      if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) ||
         !(getNodeChild(node, i)->getValue() == firstChild->getValue()))
        return false;
    }

    return true;
  }

  NormalsColorOctreeNode* NormalsColorOctree::averageNodeColor(const octomap::OcTreeKey& key,
                                                 uint8_t r,
                                                 uint8_t g,
                                                 uint8_t b) {
    NormalsColorOctreeNode* n = search(key);
    if (n != 0) {
      if (n->isColorSet()) {
        NormalsColorOctreeNode::Color prev_color = n->getColor();
        n->setColor((prev_color.r + r)/2, (prev_color.g + g)/2, (prev_color.b + b)/2);
      } else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }

  NormalsColorOctreeNode* NormalsColorOctree::integrateNodeColor(const octomap::OcTreeKey& key,
                                                   uint8_t r,
                                                   uint8_t g,
                                                   uint8_t b) {
    NormalsColorOctreeNode* n = search(key);
    if (n != 0) {
      if (n->isColorSet()) {
        NormalsColorOctreeNode::Color prev_color = n->getColor();
        double node_prob = n->getOccupancy();
        uint8_t new_r = (uint8_t) ((double) prev_color.r * node_prob + (double) r * (0.99-node_prob)); //NOLINT
        uint8_t new_g = (uint8_t) ((double) prev_color.g * node_prob +  (double) g * (0.99-node_prob)); //NOLINT
        uint8_t new_b = (uint8_t) ((double) prev_color.b * node_prob +  (double) b * (0.99-node_prob)); //NOLINT
        n->setColor(new_r, new_g, new_b);
      } else {
        n->setColor(r, g, b);
      }
    }
    return n;
  }

    NormalsColorOctreeNode* NormalsColorOctree::integrateNodeNormals(const octomap::OcTreeKey& key,
                                                   float normal_x,
                                                   float normal_y,
                                                   float normal_z) {
    NormalsColorOctreeNode* n = search(key);
    if (n != 0) {
        n->setNormals(normal_x, normal_y, normal_z);
    }
    return n;
  }

  void NormalsColorOctree::updateInnerOccupancy() {
    this->updateInnerOccupancyRecurs(this->root, 0);
  }

  void NormalsColorOctree::updateInnerOccupancyRecurs(NormalsColorOctreeNode* node,
        unsigned int depth) {
    // only recurse and update for inner nodes:
    if (nodeHasChildren(node)) {
      // return early for last level:
      if (depth < this->tree_depth) {
        for (unsigned int i = 0; i < 8; i++) {
          if (nodeChildExists(node, i)) {
            updateInnerOccupancyRecurs(getNodeChild(node, i), depth+1);
          }
        }
      }
      node->updateOccupancyChildren();
      node->updateColorChildren();
    }
  }

  void NormalsColorOctree::writeColorHistogram(std::string filename) {
#ifdef _MSC_VER
    fprintf(stderr, "The color histogram uses gnuplot, this is not supported under windows.\n");
#else
    // build RGB histogram
    std::vector<int> histogram_r(256, 0);
    std::vector<int> histogram_g(256, 0);
    std::vector<int> histogram_b(256, 0);
    for (NormalsColorOctree::tree_iterator it = this->begin_tree(),
          end=this->end_tree(); it!= end; ++it) {
      if (!it.isLeaf() || !this->isNodeOccupied(*it)) continue;
      NormalsColorOctreeNode::Color& c = it->getColor();
      ++histogram_r[c.r];
      ++histogram_g[c.g];
      ++histogram_b[c.b];
    }
    // plot data
    FILE *gui = popen("gnuplot ", "w");
    fprintf(gui, "set term postscript eps enhanced color\n");
    fprintf(gui, "set output \"%s\"\n", filename.c_str());
    fprintf(gui, "plot [-1:256] ");
    fprintf(gui, "'-' w filledcurve lt 1 lc 1 tit \"r\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 2 tit \"g\",");
    fprintf(gui, "'-' w filledcurve lt 1 lc 3 tit \"b\",");
    fprintf(gui, "'-' w l lt 1 lc 1 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 2 tit \"\",");
    fprintf(gui, "'-' w l lt 1 lc 3 tit \"\"\n");

    for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_r[i]);
    fprintf(gui, "0 0\n"); fprintf(gui, "e\n");
    for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_g[i]);
    fprintf(gui, "0 0\n"); fprintf(gui, "e\n");
    for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_b[i]);
    fprintf(gui, "0 0\n"); fprintf(gui, "e\n");
    for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_r[i]);
    fprintf(gui, "e\n");
    for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_g[i]);
    fprintf(gui, "e\n");
    for (int i = 0; i < 256; ++i) fprintf(gui, "%d %d\n", i, histogram_b[i]);
    fprintf(gui, "e\n");
    fflush(gui);
#endif
  }

  std::ostream& operator<<(std::ostream& out, NormalsColorOctreeNode::Color const& c) {
    return out << '(' << (unsigned int)c.r << ' '
        << (unsigned int)c.g << ' ' << (unsigned int)c.b << ')';
  }


  NormalsColorOctree::StaticMemberInitializer NormalsColorOctree::NormalsColorOctreeMemberInit;

}  // namespace mapper3d
}  // namespace applications
}  // namespace crf
