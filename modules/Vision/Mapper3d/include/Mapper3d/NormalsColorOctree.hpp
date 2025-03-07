#pragma once

/* © Copyright CERN 2018. All rights reserved. This software is released under a CERN proprietary software license.
 * Any permission to use it shall be granted in writing. Requests shall be addressed to CERN through mail-KT@cern.ch
 *
 * Author: Sergio Villanueva Lorente CERN EN/SMM/MRO 
 *
 *  ==================================================================================================
 */

#include <iostream>
#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>
#include <string>

namespace crf {
namespace applications {
namespace mapper3d {

// forward declaraton for "friend"
class NormalsColorOctree;

// node definition
class NormalsColorOctreeNode : public octomap::OcTreeNode {
 public:
    friend class NormalsColorOctree;  // needs access to node children (inherited)

    class Color {
     public:
        Color() : r(255), g(255), b(255) {}
        Color(uint8_t _r, uint8_t _g, uint8_t _b)
            : r(_r), g(_g), b(_b) {}
        inline bool operator== (const Color &other) const {
            return (r == other.r && g == other.g && b == other.b);
        }
        inline bool operator!= (const Color &other) const {
            return (r != other.r || g != other.g || b != other.b);
        }
        uint8_t r, g, b;
    };

    class Normals {
     public:
        Normals() : normal_x(0.0), normal_y(0.0), normal_z(0.0) {}
        Normals(float _normal_x, float _normal_y, float _normal_z)
            : normal_x(_normal_x), normal_y(_normal_y), normal_z(_normal_z) {}
        inline bool operator== (const Normals &other) const {
            return (normal_x == other.normal_x && normal_y == other.normal_y &&
                normal_z == other.normal_z);
        }
        inline bool operator!= (const Normals &other) const {
            return (normal_x != other.normal_x || normal_y != other.normal_y ||
                normal_z != other.normal_z);
        }
        float normal_x, normal_y, normal_z;
    };

 public:
    NormalsColorOctreeNode() : OcTreeNode() {}

    NormalsColorOctreeNode(const NormalsColorOctreeNode& rhs) : OcTreeNode(rhs),
        color(rhs.color), normals(rhs.normals) {}

    bool operator==(const NormalsColorOctreeNode& rhs) const {
      return (rhs.value == value && rhs.color == color && rhs.normals == normals);
    }

    void copyData(const NormalsColorOctreeNode& from) {
        OcTreeNode::copyData(from);
        this->color = from.getColor();
        this->normals = from.getNormals();
    }

    // Color functions
    inline Color getColor() const { return color; }
    inline void  setColor(Color c) {this->color = c; }
    inline void  setColor(uint8_t r, uint8_t g, uint8_t b) {
        this->color = Color(r, g, b);
    }

    Color& getColor() { return color; }

    // has any color been integrated? (pure white is very unlikely...)
    inline bool isColorSet() const {
        return ((color.r != 255) || (color.g != 255) || (color.b != 255));
    }

    void updateColorChildren();
    NormalsColorOctreeNode::Color getAverageChildColor() const;


    // Normals functions
    inline Normals getNormals() const { return normals; }
    inline void  setNormals(Normals n) {this->normals = n; }
    inline void  setNormals(float normal_x, float normal_y, float normal_z) {
         this->normals = Normals(normal_x, normal_y, normal_z);
    }
    Normals& getNormals() { return normals; }

    // has any normal been integrated?
    inline bool isNormalsSet() const {
        return ((normals.normal_x != 0.0) || (normals.normal_y != 0.0) ||
            (normals.normal_z != 0.0));
    }

    void updateNormalsChildren();
    NormalsColorOctreeNode::Normals getAverageChildNormals() const;


    // file I/O
    std::istream& readData(std::istream &s);
    std::ostream& writeData(std::ostream &s) const;

 protected:
    Color color;
    Normals normals;
};


// tree definition
class NormalsColorOctree : public octomap::OccupancyOcTreeBase <NormalsColorOctreeNode> {
 public:
    /// Default constructor, sets resolution of leafs
    explicit NormalsColorOctree(double resolution);

    /// virtual constructor: creates a new object of same type
    /// (Covariant return type requires an up-to-date compiler)
    NormalsColorOctree* create() const {return new NormalsColorOctree(resolution);}

    std::string getTreeType() const {return "NormalsColorOctree";}

     /**
     * Prunes a node when it is collapsible. This overloaded
     * version only considers the node occupancy for pruning,
     * different colors of child nodes are ignored.
     * @return true if pruning was successful
     */
    virtual bool pruneNode(NormalsColorOctreeNode* node);

    virtual bool isNodeCollapsible(const NormalsColorOctreeNode* node) const;

    // set node color at given key or coordinate. Replaces previous color.
    NormalsColorOctreeNode* setNodeColor(const octomap::OcTreeKey& key, uint8_t r,
                                 uint8_t g, uint8_t b);

    NormalsColorOctreeNode* setNodeColor(float x, float y,
                                 float z, uint8_t r,
                                 uint8_t g, uint8_t b) {
        octomap::OcTreeKey key;
        if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key)) return NULL;
        return setNodeColor(key, r, g, b);
    }

    // set node normals at given key or coordinate. Replaces previous normals.
    NormalsColorOctreeNode* setNodeNormals(const octomap::OcTreeKey& key, float normal_x,
                                 float normal_y, float normal_z);

    NormalsColorOctreeNode* setNodeNormals(float x, float y,
                                 float z, float normal_x,
                                 float normal_y, float normal_z) {
        octomap::OcTreeKey key;
        if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key)) return NULL;
        return setNodeNormals(key, normal_x, normal_y, normal_z);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    NormalsColorOctreeNode* averageNodeColor(const octomap::OcTreeKey& key, uint8_t r,
                                  uint8_t g, uint8_t b);

    NormalsColorOctreeNode* averageNodeColor(float x, float y,
                                      float z, uint8_t r,
                                      uint8_t g, uint8_t b) {
        octomap::OcTreeKey key;
        if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key)) return NULL;
        return averageNodeColor(key, r, g, b);
    }

    // integrate color measurement at given key or coordinate. Average with previous color
    NormalsColorOctreeNode* integrateNodeColor(const octomap::OcTreeKey& key, uint8_t r,
                                  uint8_t g, uint8_t b);

    NormalsColorOctreeNode* integrateNodeColor(float x, float y,
                                      float z, uint8_t r,
                                      uint8_t g, uint8_t b) {
        octomap::OcTreeKey key;
        if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key)) return NULL;
        return integrateNodeColor(key, r, g, b);
    }

    // integrate normals measurement at given key or coordinate
    NormalsColorOctreeNode* integrateNodeNormals(const octomap::OcTreeKey& key, float normal_x,
                                  float normal_y, float normal_z);

    NormalsColorOctreeNode* integrateNodeNormals(float x, float y,
                                      float z, float normal_x,
                                      float normal_y, float normal_z) {
        octomap::OcTreeKey key;
        if (!this->coordToKeyChecked(octomap::point3d(x, y, z), key)) return NULL;
        return integrateNodeNormals (key, normal_x, normal_y, normal_z);
    }

    // update inner nodes, sets color to average child color
    void updateInnerOccupancy();

    // uses gnuplot to plot a RGB histogram in EPS format
    void writeColorHistogram(std::string filename);

 protected:
    void updateInnerOccupancyRecurs(NormalsColorOctreeNode* node, unsigned int depth);

    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a 
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer{
     public:
        StaticMemberInitializer() {
            NormalsColorOctree* tree = new NormalsColorOctree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }

         /**
         * Dummy function to ensure that MSVC does not drop the
         * StaticMemberInitializer, causing this tree failing to register.
         * Needs to be called from the constructor of this octree.
         */
         void ensureLinking() {}; //NOLINT
    };
    /// static member to ensure static initialization (only once)
    static StaticMemberInitializer NormalsColorOctreeMemberInit;
};

    //! user friendly output in format (r g b)
    std::ostream& operator<<(std::ostream& out, NormalsColorOctreeNode::Color const& c);

}  // namespace mapper3d
}  // namespace applications
}  // namespace crf
