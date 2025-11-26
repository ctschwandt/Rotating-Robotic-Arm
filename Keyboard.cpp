// File  : Keyboard.cpp
// Author: Cole Schwandt

#include <GL/freeglut.h>
#include "View.h"
#include "SingletonView.h"
#include "Keyboard.h"

void mygllib::Keyboard::keyboard(unsigned char key, int w, int h)
{
    mygllib::View & view = *(mygllib::SingletonView::getInstance());

    switch (key)
    {
        case 'x': view.eyex() -= 0.1; break;
        case 'X': view.eyex() += 0.1; break;
        case 'y': view.eyey() -= 0.1; break;
        case 'Y': view.eyey() += 0.1; break;
        case 'z': view.eyez() -= 0.1; break;
        case 'Z': view.eyez() += 0.1; break;
            
        case 'v': view.fovy() -= 0.1; break;
        case 'V': view.fovy() += 0.1; break;            
        case 'a': view.aspect() -= 0.1; break;
        case 'A': view.aspect() += 0.1; break;
        case 'n': view.zNear() -= 0.1; break;
        case 'N': view.zNear() += 0.1; break;
        case 'f': view.zFar() -= 0.1; break;
        case 'F': view.zFar() += 0.1; break;
    }

    // clamp values
    if (view.fovy() < 15.0f)  view.fovy() = 15.f;
    if (view.fovy() > 120.0f) view.fovy() = 120.f;

    if (view.aspect() < 0.1f) view.aspect() = 0.1f;

    if (view.zNear() < 1e-4f) view.zNear() = 1e-4f;
    if (view.zFar()  < view.zNear() * 10.f) view.zFar() = view.zNear() * 10.f;

    view.set_projection();
    view.lookat();
    //light.set_position();
    glutPostRedisplay();
}
