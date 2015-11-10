#include <string>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "CMU462/CMU462.h"
#include "CMU462/vector3D.h"
#include "CMU462/matrix3x3.h"
#include "CMU462/bbox.h"
#include "CMU462/viewer.h"
#include "CMU462/renderer.h"
#include "CMU462/misc.h"

#include "GL/glew.h"

#include "utils.cpp"

#define USAGE_ERR 1
#define DATA_LOAD_ERR 2

using namespace std;
using namespace CMU462;

class VizCamera {
public:

    VizCamera() {
        set_parameters(5, PI / 2, PI / 4);
    }

    void set_projection() {
        Vector3D obj = pos + dir * r;
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(60.0, 1.0, 0.01f, 100.0f);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        gluLookAt(pos.x, pos.y, pos.z,
                  obj.x, obj.y, obj.z,
                  up.x,  up.y,  up.z);
    }

    void set_parameters(float dist, float cameraPhi, float cameraTheta) {
        r = dist;
        phi = cameraPhi;
        theta = cameraTheta;

        double sinTheta = sin(theta);
        if (sinTheta == 0) {
            theta += EPS_F;
            sinTheta = sin(theta);
        }
        dir = -Vector3D(sinTheta * cos(phi),
                        -sinTheta * sin(phi),
                        cos(theta));
        pos = -dir * r;

        Vector3D upVec(0, 0, sinTheta > 0 ? 1 : -1);
        Vector3D screenXDir = cross(upVec, -dir);
        screenXDir.normalize();
        up = cross(screenXDir, dir);
        up.normalize();

    }

    Vector3D pos;
    Vector3D dir;
    Vector3D up;
    float r;
    float phi;
    float theta;
};

void draw_coordinates() {

    glBegin(GL_LINES);

    // x axis in red
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3i(0,0,0);
    glVertex3i(1,0,0);

    // y axis in green
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3i(0,0,0);
    glVertex3i(0,1,0);

    // z axis in blue
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3i(0,0,0);
    glVertex3i(0,0,1);

    // draw a grid in the x-y plane
    const int resolution = 10;

    glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
    for (int x = 0; x <= resolution; x++) {
        glVertex3f(x - resolution/2, -resolution/2, 0);
        glVertex3f(x - resolution/2, resolution/2, 0);
    }
    for (int z = 0; z <= resolution; z++) {
        glVertex3f(-resolution/2, z - resolution/2, 0);
        glVertex3f( resolution/2, z - resolution/2, 0);
    }

    glEnd();
}

bool less_by_x(const Vector3D& p1, const Vector3D& p2) {
    return p1[0] < p2[0];
}

bool less_by_y(const Vector3D& p1, const Vector3D& p2) {
    return p1[1] < p2[1];
}

bool less_by_z(const Vector3D& p1, const Vector3D& p2) {
    return p1[2] < p2[2];
}

void drawPlane(const Vector4D& plane, 
    float min_x, float max_x, float min_y, float max_y) {
    
    glBegin(GL_QUADS);

    glVertex3f(min_x, min_y, Utils::solvePlaneForZ(plane, min_x, min_y));
    glVertex3f(min_x, max_y, Utils::solvePlaneForZ(plane, min_x, max_y));
    glVertex3f(max_x, max_y, Utils::solvePlaneForZ(plane, max_x, max_y));
    glVertex3f(max_x, min_y, Utils::solvePlaneForZ(plane, max_x, min_y));

    glEnd();
}

void drawCameraFrustum(const Vector3D& pos, const Matrix3x3& orient) {

    const float dist = 5.f;

    Vector3D p1(-1.f, -1.f, -dist);
    Vector3D p2(1.f, -1.f, -dist);
    Vector3D p3(1.f, 1.f, -dist);
    Vector3D p4(-1.f, 1.f, -dist);

    const float scale = 0.1f;

    Vector3D p1World = pos + orient * (scale * p1);
    Vector3D p2World = pos + orient * (scale * p2);
    Vector3D p3World = pos + orient * (scale * p3);
    Vector3D p4World = pos + orient * (scale * p4);

    glBegin(GL_LINES);

    glVertex3f(pos.x, pos.y, pos.z);
    glVertex3f(p1World.x, p1World.y, p1World.z);
    glVertex3f(pos.x, pos.y, pos.z);
    glVertex3f(p2World.x, p2World.y, p2World.z);
    glVertex3f(pos.x, pos.y, pos.z);
    glVertex3f(p3World.x, p3World.y, p3World.z);
    glVertex3f(pos.x, pos.y, pos.z);
    glVertex3f(p4World.x, p4World.y, p4World.z);

    glVertex3f(p1World.x, p1World.y, p1World.z);
    glVertex3f(p2World.x, p2World.y, p2World.z);
    glVertex3f(p2World.x, p2World.y, p2World.z);
    glVertex3f(p3World.x, p3World.y, p3World.z);
    glVertex3f(p3World.x, p3World.y, p3World.z);
    glVertex3f(p4World.x, p4World.y, p4World.z);
    glVertex3f(p4World.x, p4World.y, p4World.z);
    glVertex3f(p1World.x, p1World.y, p1World.z);

    glEnd();
}


class VizApp : public Renderer {
private:
    Vector4D* ground_plane;
    bool draw_plane = true;

    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;

public:

    void calculateKeypointBounds() {
        auto mmx = std::minmax_element(
            data->keypoints.begin(), data->keypoints.end(), less_by_x);
        min_x = (*mmx.first)[0];
        max_x = (*mmx.second)[0];

        auto mmy = std::minmax_element(
            data->keypoints.begin(), data->keypoints.end(), less_by_y);
        min_y = (*mmy.first)[1];
        max_y = (*mmy.second)[1];

        auto mmz = std::minmax_element(
            data->keypoints.begin(), data->keypoints.end(), less_by_z);
        min_z = (*mmz.first)[2];
        max_z = (*mmz.second)[2];
    }

    VizApp(Dataset* data) {
        this->data = data;
        calculateKeypointBounds();
    }

    VizApp(Dataset* data, Vector4D* ground_plane) {
        this->data = data;
        this->ground_plane = ground_plane;
        calculateKeypointBounds();
    }

    ~VizApp() { }

    string name() {
        return "VizApp";
    }

    string info() {
        return "VizApp";
    }

    void init() {
        text_mgr.init(use_hdpi);

        resetView();
        currentCamera = 0;
    }

    void resetView() {

        float phi = PI / 2.f;
        float theta = PI / 4.f;

        BBox bbox;
        for (size_t i=0; i<data->keypoints.size();i++) {
            bbox.expand(data->keypoints[i]);
        }
        for (size_t i=0; i<data->cameraPos.size();i++) {
            bbox.expand(data->cameraPos[i]);
        }

        float dist = bbox.extent[bbox.max_dimension()];

        printf("Dist = %f\n", dist);

        vizCamera.set_parameters(2.f * dist, phi, theta);

    }

    // called on display resize
    void resize(size_t w, size_t h) {
        this->w = w;
        this->h = h;
        text_mgr.resize(w,h);
    }

    void cursor_event(float x, float y) {

        // left-click drag rotates
        if (left_down) {
            float phi = vizCamera.phi + (x - mouse_x) / w * PI;
            float theta = vizCamera.theta - (y - mouse_y) / w * PI;
            vizCamera.set_parameters(vizCamera.r, phi, theta);
        }
        mouse_x = x;
        mouse_y = y;
        float anchor_x = 2 * (x + 10 - .5 * w) / w;
        float anchor_y = 2 * (.5 * h - y + 10) / h;
    }

    // scroll wheel zooms
    void scroll_event(float offset_x, float offset_y) {
        float dist = vizCamera.r + (offset_x + offset_y) * vizCamera.r * 0.1;
        vizCamera.set_parameters(dist, vizCamera.phi, vizCamera.theta);
    }

    void mouse_event(int key, int event, unsigned char mods) {
        if (key == MOUSE_LEFT) {
            if (event == EVENT_PRESS) left_down = true;
            if (event == EVENT_RELEASE) left_down = false;
        }
    }

    // handle key pressed
    void keyboard_event(int key, int event, unsigned char mods) {
        string s;
        switch (event) {
        case EVENT_PRESS:
            switch (key) {
            case 'r':
            case 'R':
                resetView();
                break;
            default:
                break;
            case 't':
            case 'T':
                vizCamera.set_parameters(vizCamera.r, PI / 2.f, 0.f);
                break;
            case 's':
            case 'S':
                vizCamera.set_parameters(vizCamera.r, 0.f, PI / 2);
                break;
            case 'p':
            case 'P':
                draw_plane = !draw_plane;
                break;
            case KEYBOARD_LEFT:
                if (currentCamera == 0)
                    currentCamera = data->cameraPos.size()-1;
                else
                    currentCamera--;
                printf("Current Camera = %d\n", currentCamera);
                break;
            case KEYBOARD_RIGHT:
                if (currentCamera == data->cameraPos.size()-1)
                    currentCamera = 0;
                else
                    currentCamera++;
                printf("Current Camera = %d\n", currentCamera);
                break;
            }
        case EVENT_RELEASE:
            break;
        case EVENT_REPEAT:
            break;
        }

//         if (key == KEYBOARD_ENTER) {
//             s += "Enter";
//         } else {
//             char c = key;
//             s += c;
//         }
    }

    // this is the main method that draws things
    void render() {

        vizCamera.set_projection();
        draw_coordinates();
        text_mgr.render();

        // draw keypoints

        // set color to gray for now
        glColor4f(0.6f, 0.6f, 0.6f, 1.f);
        glPointSize(4.f);
        glBegin(GL_POINTS);
        for (size_t i=0; i<data->keypoints.size(); i++) {
            glVertex3f(data->keypoints[i].x, data->keypoints[i].y, data->keypoints[i].z);
        }
        glEnd();


        // draw camera path as a line
        glColor3f(1.f, 0.f, 0.f);
        glBegin(GL_LINE_STRIP);
        for (size_t i=0; i<data->cameraPos.size(); i++) {
            glVertex3f(data->cameraPos[i].x, data->cameraPos[i].y, data->cameraPos[i].z);
        }
        glEnd();

        // draw point for each camera position
        glColor3f(1.f, 0.8f, 0.f);
        glBegin(GL_POINTS);
        for (size_t i=0; i<data->cameraPos.size(); i++) {
            glVertex3f(data->cameraPos[i].x, data->cameraPos[i].y, data->cameraPos[i].z);
        }
        glEnd();

        // draw orientation of current camera
        glColor3f(1.f, 1.f, 1.f);
        drawCameraFrustum(data->cameraPos[currentCamera], data->cameraOrient[currentCamera]);

        // draw the estimated ground plane
        glColor4f(1.f, 1.f, 0.f, 0.3f);
        if (draw_plane) {
            drawPlane(*ground_plane, min_x, max_x, min_y, max_y);
        }
    }


private:

    // OSD text manager
    OSDText text_mgr;

    // frame buffer size
    size_t w, h;

    // key states
    bool left_down;

    // camera
    VizCamera vizCamera;

    // mouse location
    float mouse_x;
    float mouse_y;

    int currentCamera;

    Dataset* data;

};


int main( int argc, char** argv ) {

    if (argc < 3) {
        printf("Usage: %s <3D points .ply> <camera points .ply>\n", argv[0]);
        return USAGE_ERR;
    }

    // load dataset
    Dataset data(argv[1], argv[2]);
    if (!data.load()) {
        printf("Unable to load the point cloud data\n");
        return DATA_LOAD_ERR;
    };

    // estimate ground plane
    Vector4D ground_plane = Utils::findGroundPlane(data.keypoints);
    
    // create viewer
    Viewer viewer = Viewer();

    // defined a user space renderer
    Renderer* renderer = new VizApp(&data, &ground_plane);

    // set user space renderer
    viewer.set_renderer(renderer);

    // start the viewer
    viewer.init();
    viewer.start();

    return 0;
}
