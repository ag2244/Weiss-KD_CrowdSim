#include "main.h";
#include <math.h>;


// camera
vec3 cameraPos = { 0.0f, 0.0f, 0.0f };
vec3 cameraFront = { 0.0f, 0.0f, -1.0f };
vec3 cameraUp = { 0.0f, 1.0f, 0.0f };

bool firstMouse = true;
float yaw = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch = 0.0f;
float lastX = 800.0f / 2.0;
float lastY = 600.0 / 2.0;
float fov = 45.0f;

/* AG2244: BEGIN ADDITION */

/* AG2244: CUSTOM WRITTEN */

vec3 multByVal(vec3 a, float x) {
    vec3 newv;
    newv.x = a.y *x;
    newv.y = a.z * x;
    newv.z = a.x * x;

    return newv;
}

/* AG2244: END CUSTOM WRITTEN */

vec3 normalize(vec3 a) {
    float invLength = 1.0f / sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);

    vec3 newv;

    newv.x = a.x * invLength;
    newv.y = a.y * invLength;
    newv.z = a.z * invLength;
    return newv;
}

//vector and math
vec3 add_vec3(vec3 a, vec3 b) {
    vec3 newv;
    newv.x = a.x + b.x;
    newv.y = a.y + b.y;
    newv.z = a.z + b.z;

    return newv;
}

vec3 sub_vec3(vec3 a, vec3 b) {
    vec3 newv;
    newv.x = a.x - b.x;
    newv.y = a.y - b.y;
    newv.z = a.z - b.z;

    return newv;
}
float radians(double degree)
{
    return (degree * (pi / 180));
}

vec3 norm_vec3(vec3 vec)
{
    float len = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    vec3 newv = { 0,0,0 };
    if (len != 0.0f)
    {
        newv.x = vec.x / len;
        newv.y = vec.y / len;
        newv.z = vec.z / len;
    }
    return newv;
}

vec3 cross(vec3 a, vec3 b) {
    vec3 newv;
    newv.x = a.y * b.z - a.z * b.y;
    newv.y = a.z * b.x - a.x * b.z;
    newv.z = a.x * b.y - a.y * b.x;

    return newv;
}

vec3 scale_vec3(vec3 a, float b) {
    vec3 newv;
    newv.x = a.x * b;
    newv.y = a.y * b;
    newv.z = a.z * b;

    return newv;
}

vec3 abs_vec3(vec3 a) {
    vec3 newv;
    newv.x = abs(a.x);
    newv.y = abs(a.y);
    newv.z = abs(a.z);
    return newv;
}

float length_vec3(vec3 a) {
    return sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
}

vec3 cross_vec3(vec3 a, vec3 b) {
    vec3 newv;
    newv.x = a.y * b.z - a.z * b.y;
    newv.y = -(a.x * b.z - a.z * b.x);
    newv.z = a.x * b.y - a.y * b.x;
    return newv;
}

float dot_vec3(vec3 a, vec3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float angleBetweenTwoVec3(vec3 a, vec3 b) {
    float angle = acos((dot_vec3(a, b)) / (length_vec3(a) * length_vec3(b)));

    vec3 vec = cross_vec3(a, b);
    vec = norm_vec3(vec);
    if (vec.z == 1) {
        //angle *= -1;
        //printf("reverse");
    }
    if (vec.z > 0 && vec.z < 1) {
        //angle *= -1;

        //printf("reverse");
    }
    if (isnan(angle) || isinf(angle))
        angle = 0;
    return angle;
}

vec3 rotate_around_axis_vec3(vec3 v, vec3 r, float a)
{
    float deg = a * 180 / pi;

    vec3 q = (1 - cos(a)) * v.dot(r) * r + cos(a) * v + sin(a) * r.cross(v);
    //vec3 q = add_vec3(add_vec3(scale_vec3(r, (1 - cos(a)) * (dot_vec3(v, r))), scale_vec3(v, cos(a))), scale_vec3(cross_vec3(r, v), sin(a)));
    return q;
}

vec3 qRotate(glm::vec3 point, float angle, glm::vec3 axis)
{
    //Create quaternion.
    //glm::quat orientation = glm::quat(rotate);
    glm::quat orientation = glm::angleAxis(angle, axis);
    //Normalize
    orientation = glm::normalize(orientation);

    /*Method 1 - pure vector quaternion.*/
    //Create the 'pure' vector quaternion
    glm::quat pure = glm::quat(0.0, point.x, point.y, point.z);
    glm::quat qpq = orientation * pure * (glm::conjugate(orientation));
    //Form the result
    //glm::vec3 result = glm::vec3(qpq.x, qpq.y, qpq.z);
    //printf("\nResult1 = %f %f %f", result.x, result.y, result.z);

    /*Method 2 - just apply the orientation to the point.*/
    glm::vec3 result = orientation * point;
    //printf("\nResult2 = %f %f %f", alpha.x, alpha.y, alpha.z);
    return { result.x, result.y, result.z };
}

/* AG2244: END ADDITION */

/* Initialize OpenGL Graphics */
void initGL() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
    glClearDepth(1.0f);                   // Set background depth to farthest
    glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
    glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
    glShadeModel(GL_SMOOTH);   // Enable smooth shading
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
}

//draw cube
void drawCube(float x, float y, float z, float w, float h, float l) {
    glPushMatrix();
    glTranslatef(x, y, z);  // Move right and into the screen

    glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
       // Top face (y = 1.0f)
       // Define vertices in counter-clockwise (CCW) order with normal pointing out
    glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3f(w, h, -l);
    glVertex3f(-w, h, -l);
    glVertex3f(-w, h, l);
    glVertex3f(w, h, l);

    // Bottom face (y = -1.0f)
    glColor3f(1.0f, 0.5f, 0.0f);     // Orange
    glVertex3f(w, -h, l);
    glVertex3f(-w, -h, l);
    glVertex3f(-w, -h, -l);
    glVertex3f(w, -h, -l);

    // Front face  (z = 1.0f)
    glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3f(w, h, l);
    glVertex3f(-w, h, l);
    glVertex3f(-w, -h, l);
    glVertex3f(w, -h, l);

    // Back face (z = -1.0f)
    glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
    glVertex3f(w, -h, -l);
    glVertex3f(-w, -h, -l);
    glVertex3f(-w, h, -l);
    glVertex3f(w, h, -l);

    // Left face (x = -1.0f)
    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3f(-w, h, l);
    glVertex3f(-w, h, -l);
    glVertex3f(-w, -h, -l);
    glVertex3f(-w, -h, l);

    // Right face (x = 1.0f)
    glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
    glVertex3f(w, h, -l);
    glVertex3f(w, h, l);
    glVertex3f(w, -h, l);
    glVertex3f(w, -h, -l);
    glEnd();  // End of drawing color-cube
    glPopMatrix();
}


void drawSphere(float x, float y, float z, float r) {
    GLUquadric* quad = gluNewQuadric();
    glPushMatrix();
    glTranslatef(x, y, z);
    glColor3f(1.0f, 0.0f, 0.0f);
    gluSphere(quad, r, 10, 10);
    glPopMatrix();
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);
    //gluPerspective(45.0, SCR_WIDTH / SCR_HEIGHT, 0.1, 100.0);
    glLoadIdentity();
    gluLookAt(cameraPos.x, cameraPos.y, cameraPos.z, cameraPos.x + cameraFront.x, cameraPos.y + cameraFront.y, cameraPos.z + cameraFront.z, cameraUp.x, cameraUp.y, cameraUp.z);
    // Render a color-cube consisting of 6 quads with different colors
    drawCube(.6, 0, -1, .1, .1, .1);
    drawSphere(0, 0, -2, .3);
    //updateCamera();
    glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
}

/* Handler for window re-size event. Called back when the window first appears and
   whenever the window is re-sized with its new width and height */
void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
   // Compute aspect ratio of the new window
    if (height == 0) height = 1;                // To prevent divide by 0
    GLfloat aspect = (GLfloat)width / (GLfloat)height;

    // Set the viewport to cover the new window
    glViewport(0, 0, width, height);

    // Set the aspect ratio of the clipping volume to match the viewport
    glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
    glLoadIdentity();             // Reset
    // Enable perspective projection with fovy, aspect, zNear and zFar
    gluPerspective(45.0f, aspect, 0.1f, 100.0f);
}
void mouse(int button, int state, int screenx, int screeny) {
    if (firstMouse)
    {
        lastX = screenx;
        lastY = screeny;
        firstMouse = false;
    }

    float xoffset = screenx - lastX;
    float yoffset = lastY - screeny; // reversed since y-coordinates go from bottom to top
    lastX = screenx;
    lastY = screeny;

    float sensitivity = 0.1f; // change this value to your liking
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    vec3 front;
    front.x = cos(radians(yaw)) * cos(radians(pitch));
    front.y = sin(radians(pitch));
    front.z = sin(radians(yaw)) * cos(radians(pitch));
    cameraFront = normalize(front);


}

void mousePassive(int screenx, int screeny) {
    if (firstMouse)
    {
        lastX = screenx;
        lastY = screeny;
        firstMouse = false;
    }

    float xoffset = screenx - lastX;
    float yoffset = lastY - screeny; // reversed since y-coordinates go from bottom to top
    lastX = screenx;
    lastY = screeny;

    float sensitivity = 0.1f; // change this value to your liking
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw += xoffset;
    pitch += yoffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    vec3 front;
    front.x = cos(radians(yaw)) * cos(radians(pitch));
    front.y = sin(radians(pitch));
    front.z = sin(radians(yaw)) * cos(radians(pitch));
    cameraFront = normalize(front);
    //printf("camera move");
}

void processSpecialKeys(int key, int x, int y) {

    float camera_speed = 0.01f;
    vec3 cross_cam;
    vec3 ups;
    switch (key) {
    case GLUT_KEY_LEFT:
        cross_cam = cross(cameraFront, cameraUp);
        cross_cam = normalize(cross_cam);
        cross_cam = multByVal(cross_cam, -camera_speed);
        cameraPos = add_vec3(cameraPos, cross_cam);
        break;
    case GLUT_KEY_RIGHT:
        cross_cam = cross(cameraFront, cameraUp);
        cross_cam = normalize(cross_cam);
        cross_cam = multByVal(cross_cam, camera_speed);
        cameraPos = add_vec3(cameraPos, cross_cam);
        break;
    case GLUT_KEY_UP:
        ups = multByVal(cameraFront, camera_speed);
        cameraPos = add_vec3(cameraPos, ups);
        break;
    case GLUT_KEY_DOWN:
        ups = multByVal(cameraFront, -camera_speed);
        cameraPos = add_vec3(cameraPos, ups);
        break;
    }
}