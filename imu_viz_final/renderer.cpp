#include "renderer.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <GL/glut.h>

void Renderer::initialize()
{
    int argc = 0;
    glutInit(&argc, nullptr);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    setupLighting();
    humanModel.load("human.glb");
}

void Renderer::setupLighting()
{
    GLfloat amb[]  = {0.30f, 0.30f, 0.30f, 1.0f};
    GLfloat diff[] = {0.90f, 0.88f, 0.84f, 1.0f};
    GLfloat spec[] = {0.30f, 0.30f, 0.30f, 1.0f};
    GLfloat pos0[] = {8.0f, 18.0f, 10.0f, 1.0f};
    GLfloat pos1[] = {-5.0f, 10.0f, -6.0f, 1.0f};
    GLfloat fill[] = {0.25f, 0.25f, 0.28f, 1.0f};
    GLfloat zero[] = {0.0f,  0.0f,  0.0f, 1.0f};

    glLightfv(GL_LIGHT0, GL_POSITION, pos0);
    glLightfv(GL_LIGHT0, GL_AMBIENT,  amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE,  diff);
    glLightfv(GL_LIGHT0, GL_SPECULAR, spec);
    glLightfv(GL_LIGHT1, GL_POSITION, pos1);
    glLightfv(GL_LIGHT1, GL_AMBIENT,  zero);
    glLightfv(GL_LIGHT1, GL_DIFFUSE,  fill);
    glLightfv(GL_LIGHT1, GL_SPECULAR, zero);
}

void Renderer::cycleCameraView()
{
    switch (cameraView) {
        case CameraView::FRONT: cameraView = CameraView::BACK;  break;
        case CameraView::BACK:  cameraView = CameraView::SIDE;  break;
        case CameraView::SIDE:  cameraView = CameraView::FRONT; break;
    }
}

void Renderer::render(
    const glm::quat& leftForearmLocal,
    const glm::quat& rightForearmLocal,
    const glm::quat& leftUpperArmLocal,
    const glm::quat& rightUpperArmLocal,
    const glm::quat& leftThighLocal,
    const glm::quat& rightThighLocal,
    const glm::quat& leftShinLocal,
    const glm::quat& rightShinLocal,
    const glm::quat& chestWorld,
    const glm::quat& hipsWorld,
    bool placementGuideMode)
{
    glClearColor(0.07f, 0.07f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);

    int width  = vp[2];
    int height = vp[3];

    float aspect = (float)width / (float)height;
    glFrustum(-aspect * 0.5f, aspect * 0.5f, -0.5f, 0.5f, 1.5f, 200.0f);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    switch (cameraView) {
        case CameraView::FRONT:
            glTranslatef(0.0f, -9.0f, -38.0f);
            break;
        case CameraView::BACK:
            glTranslatef(0.0f, -9.0f, -38.0f);
            glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
            break;
        case CameraView::SIDE:
            glTranslatef(0.0f, -9.0f, -38.0f);
            glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
            break;
    }

    setupLighting();
    humanModel.draw(
        leftForearmLocal, rightForearmLocal,
        leftUpperArmLocal, rightUpperArmLocal,
        leftThighLocal, rightThighLocal,
        leftShinLocal, rightShinLocal,
        chestWorld, hipsWorld,
        placementGuideMode
    );

    drawWorldAxes();

    drawTrackingAxesHud(
        leftForearmLocal, rightForearmLocal,
        leftUpperArmLocal, rightUpperArmLocal,
        leftThighLocal, rightThighLocal,
        leftShinLocal, rightShinLocal,
        chestWorld, hipsWorld
    );

    if (placementGuideMode) {
        drawPlacementGuideOverlay();
    }
}

void Renderer::drawWorldAxes()
{
    glDisable(GL_LIGHTING);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(3,0,0);
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,3,0);
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,3);
    glEnd();
    glLineWidth(1.0f);
    glEnable(GL_LIGHTING);
}

void Renderer::drawTrackingAxesHud(const glm::quat& lFAL,
                                   const glm::quat& rFAL,
                                   const glm::quat& lUAL,
                                   const glm::quat& rUAL,
                                   const glm::quat& lTHL,
                                   const glm::quat& rTHL,
                                   const glm::quat& lSHL,
                                   const glm::quat& rSHL,
                                   const glm::quat& chestW,
                                   const glm::quat& hipsW)
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    GLint vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);

    float width  = (float)vp[2];
    float height = (float)vp[3];

    glOrtho(0.0, width, 0.0, height, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    const float scale = 18.0f;
    const float gap   = 60.0f;
    const float x     = width - 70.0f;
    const float x2    = x - 80.0f;
    const float x3    = x2 - 80.0f;
    const float top   = height - 50.0f;

    auto drawLabel = [&](float px, float py, const char* text) {
        glColor3f(0.90f, 0.90f, 0.92f);
        glRasterPos2f(px - 14.0f, py + 24.0f);
        for (const char* c = text; *c; ++c)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, *c);
    };

    drawLabel(x,  top,              "L_FA");  drawHudAxisWidget(x,  top,              lFAL,  scale);
    drawLabel(x,  top - gap,        "R_FA");  drawHudAxisWidget(x,  top - gap,        rFAL,  scale);
    drawLabel(x,  top - gap * 2.0f, "L_UA");  drawHudAxisWidget(x,  top - gap * 2.0f, lUAL,  scale);
    drawLabel(x,  top - gap * 3.0f, "R_UA");  drawHudAxisWidget(x,  top - gap * 3.0f, rUAL,  scale);
    drawLabel(x2, top,              "L_TH");  drawHudAxisWidget(x2, top,              lTHL,  scale);
    drawLabel(x2, top - gap,        "R_TH");  drawHudAxisWidget(x2, top - gap,        rTHL,  scale);
    drawLabel(x2, top - gap * 2.0f, "L_SH");  drawHudAxisWidget(x2, top - gap * 2.0f, lSHL,  scale);
    drawLabel(x2, top - gap * 3.0f, "R_SH");  drawHudAxisWidget(x2, top - gap * 3.0f, rSHL,  scale);
    drawLabel(x3, top,              "HIPS");  drawHudAxisWidget(x3, top,              hipsW, scale);
    drawLabel(x3, top - gap,        "CHEST"); drawHudAxisWidget(x3, top - gap,        chestW, scale);

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}

void Renderer::drawHudAxisWidget(float cx, float cy,
                                 const glm::quat& q,
                                 float scale)
{
    glm::mat4 viewRot(1.0f);
    switch (cameraView) {
        case CameraView::FRONT:
            break;
        case CameraView::BACK:
            viewRot = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0,1,0));
            break;
        case CameraView::SIDE:
            viewRot = glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(0,1,0));
            break;
    }

    glm::vec3 axes[3] = {
        glm::vec3(viewRot * glm::vec4(q * glm::vec3(1,0,0), 0.0f)),
        glm::vec3(viewRot * glm::vec4(q * glm::vec3(0,1,0), 0.0f)),
        glm::vec3(viewRot * glm::vec4(q * glm::vec3(0,0,1), 0.0f))
    };

    glColor3f(0.22f, 0.22f, 0.26f);
    glBegin(GL_LINE_LOOP);
    glVertex2f(cx - 24.0f, cy - 20.0f);
    glVertex2f(cx + 24.0f, cy - 20.0f);
    glVertex2f(cx + 24.0f, cy + 20.0f);
    glVertex2f(cx - 24.0f, cy + 20.0f);
    glEnd();

    glPointSize(4.0f);
    glColor3f(0.85f, 0.85f, 0.88f);
    glBegin(GL_POINTS);
    glVertex2f(cx, cy);
    glEnd();

    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.12f, 0.10f);
    glVertex2f(cx, cy);
    glVertex2f(cx + axes[0].x * scale, cy + axes[0].y * scale);

    glColor3f(0.20f, 1.0f, 0.20f);
    glVertex2f(cx, cy);
    glVertex2f(cx + axes[1].x * scale, cy + axes[1].y * scale);

    glColor3f(0.20f, 0.45f, 1.0f);
    glVertex2f(cx, cy);
    glVertex2f(cx + axes[2].x * scale, cy + axes[2].y * scale);
    glEnd();
    glLineWidth(1.0f);
    glPointSize(1.0f);
}

void Renderer::drawPlacementGuideOverlay()
{
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, WINDOW_WIDTH, 0.0, WINDOW_HEIGHT, -1.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    const float left = 24.0f;
    const float top  = (float)WINDOW_HEIGHT - 40.0f;

    glColor4f(0.0f, 0.0f, 0.0f, 0.45f);
    glBegin(GL_QUADS);
    glVertex2f(left - 12.0f, top + 24.0f);
    glVertex2f(left + 380.0f, top + 24.0f);
    glVertex2f(left + 380.0f, top - 132.0f);
    glVertex2f(left - 12.0f, top - 132.0f);
    glEnd();

    auto drawLine = [&](float py, const char* text, float r, float g, float b) {
        glColor3f(r, g, b);
        glRasterPos2f(left, py);
        for (const char* c = text; *c; ++c)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *c);
    };

    drawLine(top,         "SENSOR PLACEMENT RULES",                   0.55f, 1.0f, 0.6f);
    drawLine(top - 24.0f, "- Mount every sensor vertically",          0.9f,  0.9f, 0.92f);
    drawLine(top - 44.0f, "- LED must point upward (toward the head)", 0.9f, 0.9f, 0.92f);
    drawLine(top - 64.0f, "- Sensor must be centered on the marker",  0.9f,  0.9f, 0.92f);
    drawLine(top - 84.0f, "- Do not rotate the sensor around the limb", 0.9f, 0.9f, 0.92f);
    drawLine(top - 104.0f,"- Match sensor labels exactly",            0.9f,  0.9f, 0.92f);

    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
}