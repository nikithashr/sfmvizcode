#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <random>

#include "CMU462/CMU462.h"
#include "CMU462/vector3D.h"
#include "CMU462/vector4D.h"
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

    static Vector3D vector_x_matrix(const Vector3D& v, const Matrix3x3& m) {
        Vector3D new_vec;
        for (int i=0; i<3; ++i) {
            new_vec[i] = dot(v, m.column(i));
        }

        return new_vec;
    }

    template <typename T>
    static T clamp(const T& n, const T& lower, const T& upper) {
        return std::max(lower, std::min(n, upper));
    }

    static float solvePlaneForZ(Vector4D plane, float x, float y) {
        float a = plane[0];
        float b = plane[1];
        float c = plane[2];
        float d = plane[3];

        return (-1.f * (a*x + b*y + d)) / c;
    }

    template <typename T>
    static void display_vector(const vector<T>& v) {
        for (auto i = v.begin(); i != v.end(); ++i) {
            std::cout << *i << std::endl;
        }
    }

    static void translate_points(vector<Vector3D>& points, Vector3D center) {
        for (int i = 0; i < points.size(); i++) {
            points[i] -= center;
        }
    }

    static void scale_points(vector<Vector3D>& points, Matrix3x3 scale) {
        for (int i = 0; i < points.size(); i++) {
            points[i] = vector_x_matrix(points[i], scale);
        }
    }

    static float distance_to_plane(Vector4D& plane, Vector3D& point) {
        Vector4D h_point = Vector4D(point, 1.f);
        Vector3D normal = plane.to3D();

        return std::abs(dot(plane, h_point))/normal.norm();
    }

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

    static vector<Vector3D> sampleWithoutReplacement(vector<Vector3D> v, int n_samples) {
        std::random_device rd;
        std::mt19937 randomGenerator(rd());

        int n_elems = v.size() - 1;

        vector<Vector3D> sample;

        for (int i=0; i<n_samples; ++i) {
            std::uniform_int_distribution<> uniformDistribution(0, n_elems);
            int index = uniformDistribution(randomGenerator);
            std::swap(v[index], v[n_elems]);
            sample.push_back(v[n_elems]);
            n_elems--;
        }

        return sample;
    }

    /**
    * Uses 3-point RANSAC
    * TODO: make more robust
    */
    static Vector4D findGroundPlane(vector<Vector3D> &keypoints, 
        int n_iters = 1000, double fraction_inliers = 0.5, double dist_thresh = 0.1) {

        Vector4D ground_plane;

        std::vector<Vector3D> sample;
        Vector3D normal;

        for (int i=0; i<n_iters; ++i) {
            // get 3 random points from the point cloud
            sample = sampleWithoutReplacement(keypoints, 3);

            // find normal to the plane defined by the 3 sample points
            normal = cross(sample[2] - sample[0], sample[1] - sample[0]);

            // find the coefficient of the plane eqn
            double d = dot(normal, sample[0]);

            ground_plane = Vector4D(normal, -1*d);

            int c_inliers = 0;
            // calculate distance of each point from the ground plane
            // and count the number that fall within the distance threshold
            for (int j=0; j<keypoints.size(); ++j) {
                float dist = dot(ground_plane, keypoints[j]) / normal.norm();
                if (dist < dist_thresh) { c_inliers++; }
            }

            // we have met the inlier threshold
            if (((double)c_inliers / keypoints.size()) > fraction_inliers) {
                break;
            }
        }

        return ground_plane;
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

        Vector3D center = cameraPos.back();
        Utils::translate_points(cameraPos, center);
        Utils::translate_points(keypoints, center);
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
