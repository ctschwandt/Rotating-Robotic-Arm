// File: main.cpp
// Name: Cole Schwandt
//
//
// Description:
// Robotic arm with rotations

#include <iostream>
#include <cmath>
#include <GL/freeglut.h>
#include "gl3d.h"
#include "View.h"
#include "SingletonView.h"
#include "Reshape.h"
#include "Keyboard.h"
#include "Material.h"
#include "Light.h"

//==============================================================
// Config
//==============================================================
namespace cfg
{
    // -------- camera --------
    const GLfloat EYE_X = 7.0f;
    const GLfloat EYE_Y = 5.0f;
    const GLfloat EYE_Z = 7.0f;

    // -------- clear/depth --------
    const GLfloat CLEAR_R = 1.0f;
    const GLfloat CLEAR_G = 1.0f;
    const GLfloat CLEAR_B = 1.0f;
    const GLfloat CLEAR_A = 0.0f;
    const GLfloat CLEAR_DEPTH = 1.0f;

    // -------- tessellation (glut) --------
    const GLint SLICES = 20;
    const GLint STACKS = 20;

    // -------- materials --------
    const int MAT_JOINT = mygllib::Material::CHROME;
    const int MAT_LINKS = mygllib::Material::PEARL;

    // -------- light --------
    const GLenum LIGHT_ID = GL_LIGHT0;
    const GLfloat LIGHT_AMBIENT[4]  = { 0.5f, 0.5f, 0.5f, 0.5f };
    const GLfloat LIGHT_DIFFUSE[4]  = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat LIGHT_SPECULAR[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
    const GLfloat LIGHT_POS[4]      = { 4.0f, 6.0f, 3.0f, 1.0f };

    // -------- base (cube scaled) --------
    const GLfloat BASE_SIZE = 1.0f;    
    const GLfloat BASE_SX   = 5.0f;
    const GLfloat BASE_SY   = 0.5f;
    const GLfloat BASE_SZ   = 5.0f;

    // -------- joints --------
    const GLfloat JOINT_R = 1.0f; // was 1

    // -------- links (cylinders) --------
    const GLfloat ARM_R = 0.5f;
    const GLfloat ARM_L = 2.0f;
    const GLfloat ROT_Z_TO_Y = -90.0f;  
    const GLfloat OVERLAP_FRAC = 0.25f;
    
    inline GLfloat LINK_GAP()
    {
        const GLfloat m = (ARM_R < JOINT_R ? ARM_R : JOINT_R);
        return -OVERLAP_FRAC * m;
    }

    // -------- hand/fingers --------
    const GLfloat PALM_SIZE = 1.0f;
    const GLfloat FINGER_JOINT_R = 0.15f;
    const GLfloat FINGER_DIGIT_R = 0.1f;
    const GLfloat FINGER_DIGIT_L = 0.5f;
    
    const GLfloat PALM_TO_FINGER_Y = 0.5f * PALM_SIZE;
    const GLfloat FINGER_OFFSET_X  = 0.23f * JOINT_R;
    const GLfloat FINGER_OFFSET_Z  = 0.30f * JOINT_R;    
}

//==============================================================
// Lighting
//==============================================================
mygllib::Light light(
    cfg::LIGHT_ID,
    cfg::LIGHT_AMBIENT[0], cfg::LIGHT_AMBIENT[1], cfg::LIGHT_AMBIENT[2], cfg::LIGHT_AMBIENT[3],
    cfg::LIGHT_DIFFUSE[0], cfg::LIGHT_DIFFUSE[1], cfg::LIGHT_DIFFUSE[2], cfg::LIGHT_DIFFUSE[3],
    cfg::LIGHT_SPECULAR[0], cfg::LIGHT_SPECULAR[1], cfg::LIGHT_SPECULAR[2], cfg::LIGHT_SPECULAR[3],
    cfg::LIGHT_POS[0], cfg::LIGHT_POS[1], cfg::LIGHT_POS[2], cfg::LIGHT_POS[3]
);

//==============================================================
// Globals
//==============================================================
// position of base
GLfloat xb = 0.0f;
GLfloat yb = 0.0f;
GLfloat zb = 0.0f;

// rotations of joints (degrees)
//================================
GLfloat dt = 2.0f;
// Shoulder (upper arm base joint)
GLfloat shoulder_pitch = 0.0f;  // rotate about X → motion in YZ plane
GLfloat shoulder_yaw   = 0.0f;  // rotate about Y → motion in XZ plane
GLfloat shoulder_roll  = 0.0f;  // rotate about Z → motion in XY plane

// Elbow (forearm joint)
GLfloat elbow_pitch = 0.0f;     // rotate about X → YZ plane
GLfloat elbow_yaw   = 0.0f;     // rotate about Y → XZ plane
GLfloat elbow_roll  = 0.0f;     // rotate about Z → XY plane

// Fingers
GLfloat grip = 0.0f;            // 0=open … 1=closed (pinch)

struct FingerAngles { GLfloat baseZ, jointZ, tipY; };

// OPEN pose
const FingerAngles OPEN_F0 = { -60.0f, +60.0f, -35.0f };
const FingerAngles OPEN_F1 = { +60.0f, -60.0f, +35.0f };
const FingerAngles OPEN_F2 = { +60.0f, -60.0f, +35.0f };

// CLOSED pose (pinch): stronger curl, zero tip twist for a tight pinch
const FingerAngles CLOSED_F0 = { -150.0f, +120.0f, 0.0f };
const FingerAngles CLOSED_F1 = { +150.0f, -120.0f, 0.0f };
const FingerAngles CLOSED_F2 = { +150.0f, -120.0f, 0.0f };

inline GLfloat lerp(GLfloat a, GLfloat b, GLfloat t)
{
    return a + t * (b - a);
}

inline FingerAngles mix(const FingerAngles& a, const FingerAngles& b, GLfloat t)
{
    return { lerp(a.baseZ,  b.baseZ,  t),
             lerp(a.jointZ, b.jointZ, t),
             lerp(a.tipY,   b.tipY,   t) };
}

void init()
{
    mygllib::View & view = *(mygllib::SingletonView::getInstance());
    view.eyex() = cfg::EYE_X;
    view.eyey() = cfg::EYE_Y;
    view.eyez() = cfg::EYE_Z;
    view.set_projection();
    view.lookat();

    glClearColor(cfg::CLEAR_R, cfg::CLEAR_G, cfg::CLEAR_B, cfg::CLEAR_A);
    //glClearDepth(cfg::CLEAR_DEPTH);

    light.set();
    mygllib::Light::all_on();
    light.on();
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);
}

inline void clamp_grip()
{
    if (grip < 0.0f) grip = 0.0f;
    if (grip > 1.0f) grip = 1.0f;
}

//==============================================================
// Material helpers
//==============================================================
void set_mat0()
{
    mygllib::Material(cfg::MAT_JOINT).set();
}

void set_col1()
{
    mygllib::Material(cfg::MAT_LINKS).set();
}

//==============================================================
// Glut drawing helpers
//==============================================================
void draw_cube(GLfloat size)
{
    glutSolidCube(size);
}

void draw_sphere(GLfloat r, GLint slices = cfg::SLICES, GLint stacks = cfg::STACKS)
{
    glutSolidSphere(r, slices, stacks);
}

void draw_cylinder(GLfloat r, GLfloat h, GLint slices = cfg::SLICES, GLint stacks = cfg::STACKS)
{
    glutSolidCylinder(r, h, slices, stacks);
}

//==============================================================
// Robot part drawers
//==============================================================
void draw_base(GLfloat size)
{
    set_col1();
    draw_cube(size);
}

void draw_joint(GLfloat r)
{
    set_mat0();
    draw_sphere(r);
}

void draw_arm(GLfloat r, GLfloat l)
{
    set_col1();
    draw_cylinder(r, l);
}

void draw_finger(GLfloat tx, GLfloat ty, GLfloat tz,
                 GLfloat baseZDeg, GLfloat jointZDeg, GLfloat tipYDeg)
{
    glPushMatrix();
    {
        // place on palm
        glTranslatef(tx, ty, tz);
        draw_joint(cfg::FINGER_JOINT_R);

        // first phalanx: your Z-bend, then aim Z->Y and draw
        glRotatef(baseZDeg, 0.0f, 0.0f, 1.0f);
        glPushMatrix();
        {
            glRotatef(cfg::ROT_Z_TO_Y, 1.0f, 0.0f, 0.0f);
            draw_arm(cfg::FINGER_DIGIT_R, cfg::FINGER_DIGIT_L);
        }
        glPopMatrix();

        // middle joint block: walk L in (palm) Y, apply your joint Z, draw joint
        glPushMatrix();
        {
            glTranslatef(0.0f, cfg::FINGER_DIGIT_L, 0.0f);
            glRotatef(jointZDeg, 0.0f, 0.0f, 1.0f);
            draw_joint(cfg::FINGER_JOINT_R);

            // second phalanx: aim Z->Y then your Y-rotation, draw
            glPushMatrix();
            {
                glRotatef(cfg::ROT_Z_TO_Y, 1.0f, 0.0f, 0.0f);
                glRotatef(tipYDeg, 0.0f, 1.0f, 0.0f);
                draw_arm(cfg::FINGER_DIGIT_R, cfg::FINGER_DIGIT_L);
            }
            glPopMatrix();
        }
        glPopMatrix();
    }
    glPopMatrix();
}

//==============================================================
// Display
//==============================================================
void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    mygllib::SingletonView::getInstance()->lookat();

    mygllib::Light::all_off();
    mygllib::draw_xz_plane();
    mygllib::draw_axes();
    mygllib::Light::all_on();
    
    light.on();
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);
    light.set_position();
    
    // base
    glPushMatrix();
    {
        glTranslatef(xb, yb, zb);
        glScalef(cfg::BASE_SX, cfg::BASE_SY, cfg::BASE_SZ);
        draw_base(cfg::BASE_SIZE);
    }
    glPopMatrix();

    // upper arm joint (shoulder)
    glPushMatrix();
    {
        glRotatef(shoulder_pitch, 1.0f, 0.0f, 0.0f); // X
        glRotatef(shoulder_yaw,   0.0f, 1.0f, 0.0f); // Y
        glRotatef(shoulder_roll,  0.0f, 0.0f, 1.0f); // Z

        draw_joint(cfg::JOINT_R);
 
        // upper arm
        glPushMatrix();
        {
            glTranslatef(0.0f, cfg::JOINT_R + cfg::LINK_GAP(), 0.0f);

            glPushMatrix();
            {
                glRotatef(cfg::ROT_Z_TO_Y, 1.0f, 0.0f, 0.0f);
                draw_arm(cfg::ARM_R, cfg::ARM_L);
            }
            glPopMatrix();

            // forearm joint (elbow)
            glTranslatef(0.0f, cfg::ARM_L - cfg::LINK_GAP(), 0.0f);
            glRotatef(elbow_pitch, 1.0f, 0.0f, 0.0f); // X
            glRotatef(elbow_yaw,   0.0f, 1.0f, 0.0f); // Y
            glRotatef(elbow_roll,  0.0f, 0.0f, 1.0f); // Z

            draw_joint(cfg::JOINT_R);

            // forearm
            glTranslatef(0.0f, cfg::JOINT_R + cfg::LINK_GAP(), 0.0f);
            glPushMatrix();
            {
                glRotatef(cfg::ROT_Z_TO_Y, 1.0f, 0.0f, 0.0f);
                draw_arm(cfg::ARM_R, cfg::ARM_L);
            }
            glPopMatrix();
        
            // palm: top of forearm plus half the palm cube
            glTranslatef(0.0f, cfg::ARM_L + 0.5f * cfg::PALM_SIZE, 0.0f);
            draw_cube(cfg::PALM_SIZE);

            // fingers
            const GLfloat t = -grip;

            // finger 0 (center)
            {
                const FingerAngles A = mix(OPEN_F0, CLOSED_F0, t);
                draw_finger(+cfg::FINGER_OFFSET_X, cfg::PALM_TO_FINGER_Y, 0.0f,
                            A.baseZ, A.jointZ, A.tipY);
            }

            // finger 1 (left, front-right)
            {
                const FingerAngles A = mix(OPEN_F1, CLOSED_F1, t);
                draw_finger(-cfg::FINGER_OFFSET_X, cfg::PALM_TO_FINGER_Y, +cfg::FINGER_OFFSET_Z,
                            A.baseZ, A.jointZ, A.tipY);
            }

            // finger 2 (left, back-right)
            {
                const FingerAngles A = mix(OPEN_F2, CLOSED_F2, t);
                draw_finger(-cfg::FINGER_OFFSET_X, cfg::PALM_TO_FINGER_Y, -cfg::FINGER_OFFSET_Z,
                            A.baseZ, A.jointZ, A.tipY);
            }
        }
        glPopMatrix();
    }
    glPopMatrix();

    glutSwapBuffers();
}

//==============================================================
// User input
//==============================================================
void specialkeyboard(int key, int, int)
{
    switch (key)
    {
        // Shoulder (upper arm)
        case GLUT_KEY_F1:  shoulder_pitch += dt; break;
        case GLUT_KEY_F2:  shoulder_pitch -= dt; break;
        case GLUT_KEY_F3:  shoulder_yaw   += dt; break;
        case GLUT_KEY_F4:  shoulder_yaw   -= dt; break;
        case GLUT_KEY_F5: shoulder_roll  += dt; break;
        case GLUT_KEY_F6: shoulder_roll  -= dt; break;

        // Elbow (forearm)
        case GLUT_KEY_F7: elbow_pitch += dt; break;
        case GLUT_KEY_F8: elbow_pitch -= dt; break;
        case GLUT_KEY_F9: elbow_yaw   += dt; break;
        case GLUT_KEY_F10: elbow_yaw   -= dt; break;
        case GLUT_KEY_F11: elbow_roll  += dt; break;
        case GLUT_KEY_F12: elbow_roll  -= dt; break;

        // Grip (fingers)
        case GLUT_KEY_UP:  grip += 0.05f; clamp_grip(); break;
        case GLUT_KEY_DOWN: grip -= 0.05f; clamp_grip(); break;

        default: break;
    }

    glutPostRedisplay();
}

//==============================================================
// main
//==============================================================
int main(int argc, char ** argv)
{
    mygllib::init3d();
    init();
    glutDisplayFunc(display);
    glutKeyboardFunc(mygllib::Keyboard::keyboard);
    glutSpecialFunc(specialkeyboard);
    glutReshapeFunc(mygllib::Reshape::reshape);
    glutMainLoop();
    
    return 0;
}
