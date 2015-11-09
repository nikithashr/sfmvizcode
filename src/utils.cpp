#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include "CMU462/CMU462.h"
#include "CMU462/vector3D.h"
#include "CMU462/matrix3x3.h"
#include "CMU462/bbox.h"
#include "CMU462/viewer.h"
#include "CMU462/renderer.h"
#include "CMU462/misc.h"

#include "GL/glew.h"

using namespace std;
using namespace CMU462;

struct Utils {

    static const string PLY_FILETYPE;
    static const string ELEMENT_STR;
    static const string VERTEX_STR;
    static const string ENDHEADER_STR;

    static void display_points_vector(const vector<Vector3D> &v) {
        for (std::vector<Vector3D>::const_iterator i = v.begin(); i != v.end(); ++i) {
            std::cout << *i << std::endl;
        }
    }

    static void display_orientation_vector(const vector<Matrix3x3> &m) {
        for (std::vector<Matrix3x3>::const_iterator i = m.begin(); i != m.end(); ++i) {
            std::cout << *i << std::endl << std::endl;
        }
    }

    static vector<string> split(const string &s, char delim) {
        std::stringstream ss(s);
        string item;
        vector<string> tokens;

        while (getline(ss, item, delim)) {
            tokens.push_back(item);
        }

        return tokens;
    }

    static bool loadPLY(std::string filename, std::vector<Vector3D> &points) {
        std::ifstream infile(filename);

        string metadata;
        int line_num = 0;
        int num_points = 0;

        // read the metadata
        while (std::getline(infile, metadata)) {
            line_num++;

            vector<string> tokens = split(metadata, ' ');

            // first line should read 'ply'
            if (line_num == 1) {
                if (tokens.front().compare(PLY_FILETYPE)) {
                    return false;
                }
            }

            if (!tokens.front().compare(ELEMENT_STR)) {
                // read the number of points
                if (!tokens.at(1).compare(VERTEX_STR)) {
                    num_points = std::stoi(tokens.at(2));
                }
            }

            if (!tokens.front().compare(ENDHEADER_STR)) {
                break;
            }
        }

        // read the point data
        string line;
        while (std::getline(infile, line)) {
            std::istringstream iss(line);
            float x, y, z;
            if (!(iss >> x >> y >> z)) { return false; }
            points.push_back(Vector3D(x, y, z));
        }

        // display_points_vector(points);

        infile.close();

        return (points.size() == num_points);
    }

};

const string Utils::PLY_FILETYPE = "ply";
const string Utils::ELEMENT_STR = "element";
const string Utils::VERTEX_STR = "vertex";
const string Utils::ENDHEADER_STR = "end_header";

struct Dataset {

    std::vector<Vector3D> keypoints;
    std::vector<Vector3D> cameraPos;
    std::vector<Matrix3x3> cameraOrient;
    std::string pts3DFilename;
    std::string cameraPtsFilename;

    float randomFloat() {
        return static_cast<float>(rand()) / RAND_MAX;
    }

    Dataset() {
    }

    Dataset(std::string _pts3DFilename, std::string _cameraPtsFilename) {
        pts3DFilename = _pts3DFilename;
        cameraPtsFilename = _cameraPtsFilename;
    }

    bool load() {
        std::cout << "what the actual" << std::endl;
        if (pts3DFilename.empty() || cameraPtsFilename.empty()) {
            return loadDefault();
        }

        // load the .ply files
        if (!Utils::loadPLY(cameraPtsFilename, cameraPos)) return false;
        if (!Utils::loadPLY(pts3DFilename, keypoints)) return false;

        // hardcode the camera orientation for now
        for (int i=0; i<cameraPos.size(); ++i) {
            Matrix3x3 cameraToWorld;
            cameraToWorld[0] = Vector3D(1,0,0);   // camera x = camera right
            cameraToWorld[1] = Vector3D(0,0,1);   // camera y = camera up
            cameraToWorld[2] = -Vector3D(0,1,0);  // camera -z = camera looking direction
            cameraOrient.push_back(cameraToWorld);
        }

        return true;
    }

    bool loadDefault() {

        // Hardcoding 100 random points and 20 camera positions.  This
        // is probably where you'd load data from your datafiles

        for (int i=0; i<100; i++) {
            const float scale = 5.f;
            keypoints.push_back(scale * Vector3D(randomFloat(), randomFloat(), randomFloat()));
        }

        for (int i=0; i<20; i++) {

            // Hardcoding an example: camera is moving straight
            // forward 4 meters over the 20 example frames
            cameraPos.push_back(Vector3D(0.f, .25f * i, 2.f));

            //
            // m[i] is i'th column of matrix
            //
            // Hardcoding an example. This is the cameraToWorld
            // transform for a camera looking in the (0,1,0) world
            // direction, with the "camera up" as (0,0,1) in world
            // space.
            //
            Matrix3x3 cameraToWorld;
            cameraToWorld[0] = Vector3D(1,0,0);   // camera x = camera right
            cameraToWorld[1] = Vector3D(0,0,1);   // camera y = camera up
            cameraToWorld[2] = -Vector3D(0,1,0);  // camera -z = camera looking direction
            cameraOrient.push_back(cameraToWorld);
        }

        return true;

    }
};
