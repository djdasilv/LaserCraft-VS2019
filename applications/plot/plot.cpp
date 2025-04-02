//==============================================================================
//
//  TELEOPERATION
//
//==============================================================================

#include "chai3d.h"
#include <GLFW/glfw3.h>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include "Data_process.h"
#include <random>

using namespace chai3d;
using namespace std;

//------------------------------------------------------------------------------
// STATE MACHINE
// 
// STATE_IDLE: 
// The system is waiting for the operator to press the button. The robot is 
// holding its latest desired position.
// 
// STATE_TELEOPERATION:
// The operator is moving the haptic device and driving the robot to a new 
// desired position.
//------------------------------------------------------------------------------
enum cState
{
    STATE_IDLE,
    STATE_TELEOPERATION
};
enum cStateGradient
{
    STATE_AQUIREPOINT1,
    STATE_AQUIREPOINT2
};

enum cStateScanx
{
    STATE_FORWARD_X,
    STATE_BACKWARD_X
};
enum cStateScany
{
    STATE_FORWARD_Y,
    STATE_BACKWARD_Y
};



//------------------------------------------------------------------------------
// GENERAL VARIABLES
//------------------------------------------------------------------------------

//variables for gradient:

bool scan_x = false;
bool scan_y = false;
bool scan_z = false;
double gradient;
double deltaz = 0.00005; // chercher la bonne valeur !!
cVector3d P1;
double P1z;
cVector3d P2;
double P2z;


cVector3d voltageGradient;

cVector3d forceGradient;
cVector3d hapticPosGrad0;// position enregistrée quand le mode approach est activé.
cVector3d robotPosScan0;
chrono::high_resolution_clock::time_point timePointScan0 = chrono::high_resolution_clock::now();

double voltageLevel;

// a flag to indicate if the simulation currently running
bool simulationRunning = false;

// a flag to indicate if the simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the sensor rate
cFrequencyCounter freqCounterSensor;

// a frequency counter to measure the robot control rate
cFrequencyCounter freqCounterRobotDevice;

// a frequency counter to measure the haptic control rate
cFrequencyCounter freqCounterHapticDevice;

// sensor thread
cThread* sensorThread;

// robot thread
cThread* robotDeviceThread;

// haptic thread
cThread* hapticDeviceThread;

// device handler
cHapticDeviceHandler* handler;

// haptic device object
cGenericHapticDevicePtr hapticDevice = nullptr;

// robot device object
cGenericHapticDevicePtr robotDevice = nullptr;

// scale factor between haptic device and robot device
double scaleFactor = 0.2;

// haptic damping factor computed by the laser sensor
double hapticDampingFactor = 0.0;

// desired robot position
cVector3d robotPosDes(0, 0, 0);

// desired robot position simulation for scanning
cVector3d robotPosDesScan(0, 0, 0);

// current robot position
cVector3d robotPosCur(0, 0, 0);

// current robot velocity
cVector3d robotVelCur(0, 0, 0);

// rotation matrix of robot device
cMatrix3d robotRot;

// offset position between robot and graphic cube/cursor
cVector3d offset;

// mutex robot
cMutex mutexDevices;

// state machine
cState state;

// state gradient
cStateGradient stateGradient = STATE_AQUIREPOINT1;

// state scan x
cStateScanx StateScanx = STATE_FORWARD_X;

// state scan y
cStateScany StateScany = STATE_FORWARD_Y;

double threshold = 2;

//------------------------------------------------------------------------------
// CHAI3D GRAPHIC VARIABLES AND OBJECTS
//------------------------------------------------------------------------------

// a flag which indicates if the fullscreen mode is activated
bool fullscreen = false;

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight* light;

// a font for rendering text
cFontPtr font;

// a text label at the bottom of the window to display status
cLabel* labelMessage;

// a text label at the bottom of the window to display the value of the gradient
cLabel* labelGradient;

// a small sphere (cursor) representing the desired position of the robot
cShapeSphere* cursorRobotPosDes;

// a small sphere (cursor) representing the desired position of the robot while scanning
cShapeSphere* cursorRobotPosDesScan;

// a virtual voxel like object
cVoxelObject* object;

// 3D image data
cMultiImagePtr image;

// resolution of voxel model
int voxelModelResolution = 256;

// mutex for voxel object
cMutex mutexVoxel;

// 3D texture object
cTexture3dPtr texture;

// region of voxels being updated
cCollisionAABBBox volumeUpdate;

// flag that indicates that voxels have been updated
bool flagMarkVolumeForUpdate = false;

// a scope to monitor position values of haptic device
cScope* scope;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

ofstream* outAdress;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the laser sensor aquisition loop
void updateSensor(void);

// this function contains the robot device control loop
void updateRobotDevice(void);

// this function contains the haptic device control loop
void updateHapticDevice(void);

// this function set the color of a voxel at a given position in the volume
void setVoxel(cVector3d& a_pos, cColorb& a_color);

// this function closes the application
void close(void);

// This function locks restricts mouvement along x,y or z  axis
// by applying a spring force to the axis coordinate, that is set when
// user presses the button.
void axis_locking(double* forcex, double* forcey, double* forcez);

// This function does a z-axis sweep across the sample to the find the 
// maximum z-coordinate, which corresponds to the surface of the sample
void auto_scan(void);

//Ths fuction draws colored pixels at the robots position 
// according to the voltage reading signal
void draw_pixels(void);

//==============================================================================
/*
    DEMO:   plot.cpp

    This application connects a haptic device to a robot device in a
    teleoperation mode.

    A small graphical scope plots a signal value acquired by an DAC board
    (laser sensor)

    A 3D volume image is used to render laser sensor data.
*/
//==============================================================================


//Added variables and functions (to be sorted)
ofstream outFile;
bool reverseMode=false;
bool lock_y = false;
bool lock_x = false;
bool lock_z = false;
cVector3d posX, posY, posZ;
double maxSignal;
cVector3d maxPosition;
bool scan_finished(false);
void reset_pixels();

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: plot" << endl;
    cout << "Copyright 2003-2023" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[0] - scale factor 0.005x" << endl;
    cout << "[1] - scale factor 0.02x" << endl;
    cout << "[2] - scale factor 0.05x" << endl;
    cout << "[3] - scale factor 0.20x" << endl;
    cout << "[4] - scale factor 0.50x" << endl;
    cout << "[c] - reset offset" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[g] - Find first plane of focus on a glass sample" << endl;
    cout << "[x] - Lock translation in x direction" << endl;
    cout << "[y] - Lock translation in y direction" << endl;
    cout << "[z] - Lock translation in z direction" << endl;
    cout << "[t] - Move to surface of the sample" << endl;
    cout << "[u] - Draw voxels" << endl;
    cout << "[r] - Reset all voxels" << endl;
    cout << "[S] - Set new maximum obtained voltage [V]. Default set to 2 V." << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.9 * mode->height;
    int h = 0.9 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // enable double buffering
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

    // set the desired number of samples to use for multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(1);

#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(0.03, 0.0, 0.02),    // camera position (eye)
        cVector3d(0.0, 0.0, 0.0),    // look at position (target)
        cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

// set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.0001, 1.0);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(-1.0, 0.0, 0.0);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONT_CALIBRI_20();

    // create a label to display the haptic and graphic rate of the simulation
    labelMessage = new cLabel(font);
    camera->m_frontLayer->addChild(labelMessage);

    // creer label pour afficher valeur gradient 
    labelGradient = new cLabel(font);
    camera->m_frontLayer->addChild(labelGradient);




    ////////////////////////////////////////////////////////////////////////////
    // In the following lines we set up several widgets to display position
    // and velocity data coming from the haptic device. For each widget we
    // define a range of values to expect from the haptic device. In this
    // example the units are meters (as we are tracking a position signal!) and 
    // have a set a default range between -0.1 to 0.1 meters. If you are using 
    // devices with a small or larger workspace, you may want to adjust these 
    // values accordingly. The other settings will modify the visual appearance
    // of the widgets. Have fun playing with these values!
    ////////////////////////////////////////////////////////////////////////////

    // create a scope to plot haptic device position data
    scope = new cScope();
    camera->m_frontLayer->addChild(scope);
    scope->setLocalPos(100, 60);
    scope->setRange(0.0, 2.0);
    scope->setSignalEnabled(true, false, false, false);
    scope->setTransparencyLevel(0.7);


    //--------------------------------------------------------------------------
    // CREATE ROBOT SPHERE
    //--------------------------------------------------------------------------

    // create small sphere

    cursorRobotPosDes = new cShapeSphere(0.0003);

    // add cursor to world
    world->addChild(cursorRobotPosDes);

    // set cursor color
    cursorRobotPosDes->m_material->setBlueCornflower();


    //--------------------------------------------------------------------------
    // CREATE ROBOT SPHERE simulation (scanning)
    //--------------------------------------------------------------------------

    // create small sphere

    cursorRobotPosDesScan = new cShapeSphere(0.0003);

    // add cursor to world
    world->addChild(cursorRobotPosDesScan);

    // set cursor color
    cursorRobotPosDesScan->m_material->setRed();



    //--------------------------------------------------------------------------
    // CREATE VOXEL OBJECT
    //--------------------------------------------------------------------------

    // create a volumetric model
    object = new cVoxelObject();

    // add object to world
    world->addChild(object);

    // set object position in scene
    object->setLocalPos(0.0, 0.0, 0.0);

    // set the dimensions by assigning the position of the min and max corners
    object->m_minCorner.set(-0.008, -0.008, -0.008);
    object->m_maxCorner.set(0.008, 0.008, 0.008);

    // set the texture coordinate at each corner.
    object->m_minTextureCoord.set(0.0, 0.0, 0.0);
    object->m_maxTextureCoord.set(1.0, 1.0, 1.0);

    // set material color
    object->m_material->setOrangeCoral();
    // show/hide boundary box
    object->setShowBoundaryBox(true);


    //--------------------------------------------------------------------------
    // CREATE VOXEL DATA
    //--------------------------------------------------------------------------

    // create multi image data structure
    image = cMultiImage::create();

    // allocate 3D image data
    image->allocate(voxelModelResolution, voxelModelResolution, voxelModelResolution, GL_RGBA);

    // create texture
    texture = cTexture3d::create();

    // assign texture to voxel object
    object->setTexture(texture);

    // assign volumetric image to texture
    texture->setImage(image);

    // set quality of graphic rendering
    object->setQuality(0.5);

    // set default rendering mode
    object->setRenderingModeIsosurfaceMaterial();
    object->setRenderingModeIsosurfaceColors();


    //--------------------------------------------------------------------------
    // CREATE ROTATION MATRIX FOR ROBOT
    //--------------------------------------------------------------------------

    // set identity matrix
    robotRot.identity();

    // rotate the robot device base to the desired angle
    robotRot.rotateAboutGlobalAxisDeg(0, 0, 1, 180);


    //--------------------------------------------------------------------------
    // DETECT ROBOT AND HAPTIC DEVICES
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get number of devices
    unsigned int numDevices = handler->getNumDevices();

    // get pointer to devices from handler (if available)
    cGenericHapticDevicePtr device0 = cGenericHapticDevice::create();
    cGenericHapticDevicePtr device1 = cGenericHapticDevice::create();

    cHapticDeviceInfo infoDevice0;
    cHapticDeviceInfo infoDevice1;

    if (numDevices >= 1)
    {
        handler->getDevice(device0, 0);
        infoDevice0 = device0->getSpecifications();
    }

    if (numDevices >= 2)
    {
        handler->getDevice(device1, 1);
        infoDevice1 = device1->getSpecifications();
    }

    // assign devices based on their type (omega.3 or sigma.7)
    if (infoDevice0.m_model == cHapticDeviceModel::C_HAPTIC_DEVICE_SIGMA_7)
    {
        robotDevice = device0;
        hapticDevice = device1;
    }
    else if (infoDevice1.m_model == cHapticDeviceModel::C_HAPTIC_DEVICE_SIGMA_7)
    {
        robotDevice = device1;
        hapticDevice = device0;
    }
    else
    {
        robotDevice = device1;
        hapticDevice = device0;
    }
    // open connection to robot device
    robotDevice->open();

    // update position data
    cVector3d pos;
    robotDevice->getPosition(pos);

    // update current robot position by taking into account robot rotation matrix
    robotPosCur = robotRot * pos;

    // set desired robot position to current robot position
    robotPosDes = robotPosCur;

    // open connection to haptic device
    hapticDevice->open();


    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // intialize state machine
    state = STATE_IDLE;

    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // create a thread which starts the main laser sensor loop
    sensorThread = new cThread();
    sensorThread->start(updateSensor, CTHREAD_PRIORITY_GRAPHICS);

    // create a thread which starts the robot device control loop
    robotDeviceThread = new cThread();
    robotDeviceThread->start(updateRobotDevice, CTHREAD_PRIORITY_HAPTICS);

    // create a thread which starts the haptic device control loop
    hapticDeviceThread = new cThread();
    hapticDeviceThread->start(updateHapticDevice, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width = a_width;
    height = a_height;

    // update position of scope
    scope->setSize(width - 200, 70);
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    if (a_key == GLFW_KEY_T)
    {
        robotPosDes = maxPosition;
        cout << "Moved to :" << maxPosition.x() << " " << maxPosition.y() << " " << maxPosition.z() << endl;
        cout << "Moved to :" << robotPosCur.x() << " " << robotPosCur.y() << " " << robotPosCur.z() << endl;

    }
    //Resets all voxels if R key is pressed
    if (a_key == GLFW_KEY_R && a_action == GLFW_PRESS)
    {
        reset_pixels();
    }

    //Lock X translation
    if (a_key == GLFW_KEY_X && (a_action != GLFW_PRESS))
    {   
        hapticDevice->getPosition(posX);
        lock_x = !lock_x;
    }
    
    //Lock Y translation
        if (a_key == GLFW_KEY_Y && (a_action != GLFW_PRESS))
    {   
        hapticDevice->getPosition(posY);
        lock_y = !lock_y;
    }
    //Lock Z translation
    if (a_key == GLFW_KEY_Z && (a_action != GLFW_PRESS))
    {
        hapticDevice->getPosition(posZ);
        lock_z = !lock_z;
    }


    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    //Approaches sample on z-axis and writes output signal to file
    //Warning: old file is written over each time this mode is activated
    if ((a_key == GLFW_KEY_G || a_key == GLFW_KEY_H || a_key == GLFW_KEY_J) && (a_action == GLFW_PRESS))
    {
        if (((!scan_x) && a_key == GLFW_KEY_G) || ((!scan_y) && a_key == GLFW_KEY_H) || (!scan_z && a_key == GLFW_KEY_J)) {
            outFile.open("output.csv", std::ios::trunc);
            if (outFile.is_open()) {
                cVector3d tmp(0,0,0);
                switch (a_key) {
                    case GLFW_KEY_G:
                        outFile << "Position in x [m],Voltage [V]" << std::endl;
                        break;
                    case GLFW_KEY_H:
                        outFile << "Position in y [m],Voltage [V]" << std::endl;
                        break;
                    case GLFW_KEY_J:
                        outFile << "Position in z [m],Voltage [V]" << std::endl;
                        robotDevice->getPosition(tmp);
                        tmp = robotRot * tmp;
                        updateMax(tmp , 0, maxPosition, true);
                        //cout << "Postion update to :" << maxPosition.x() << " " << maxPosition.y() << " " << maxPosition.z() << endl;
                        break;
                    default:
                        break;
                }
                cout << "Data appended to output.csv successfully." << endl;
            }
            else cout << "Error opening file." << std::endl;
        }
        else
        {
            outFile.close();
            cout << "Data written and doc closed" << std::endl;
        }
        switch (a_key){
            case GLFW_KEY_G:
                scan_x = !scan_x;
                break;
            case GLFW_KEY_H:
                scan_y = !scan_y;
                break;
            case GLFW_KEY_J:
                scan_z = !scan_z;
                break;
            default:
                break;
        }
        hapticDevice->getPosition(hapticPosGrad0);
     
        

    }

    //Toggles the pixel coloring update
    if ((a_key == GLFW_KEY_U) && (a_action == GLFW_PRESS))
    {
        if (scan_finished) scan_finished = false;
        else scan_finished = true;

        cout << scan_finished << endl;
    }

    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    //Modify scale factor accord to which button is pressed
    if (state == STATE_IDLE) {
        switch (a_key) {
        case GLFW_KEY_0:
            scaleFactor = 0.001;
            break;
        case GLFW_KEY_1:
            scaleFactor = 0.02;
            break;
        case GLFW_KEY_2:
            scaleFactor = 0.05;
            break;
        case GLFW_KEY_3:
            scaleFactor = 0.2;
            break;
        case GLFW_KEY_4:
            scaleFactor = 0.5;
            break;
        default:
            break;
        }
    }


    // option - reset offset
    if (a_key == GLFW_KEY_C)
    {
        offset = robotPosDes;
    }

    // option - toggle fullscreen
    if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(1);

            // set the desired number of samples to use for multisampling
            glfwWindowHint(GLFW_SAMPLES, 4);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(1);

            // set the desired number of samples to use for multisampling
            glfwWindowHint(GLFW_SAMPLES, 4);
        }
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(500); }

    // delete resources
    delete handler;
    delete sensorThread;
    delete hapticDeviceThread;
    delete robotDeviceThread;
    delete world;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
  
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update status message
    labelMessage->setText("Graphics: " + cStr(freqCounterGraphics.getFrequency(), 0) + " Hz  /  " +
        "Sensor: " + cStr(freqCounterSensor.getFrequency(), 0) + " Hz  /  " +
        "Haptic Device: " + cStr(freqCounterHapticDevice.getFrequency(), 0) + " Hz  /  " +
        "Robot Device: " + cStr(freqCounterRobotDevice.getFrequency(), 0) + " Hz  /  " +
        "Scale Factor: " + cStr(scaleFactor, 4) + "x");

    // update position of label
    labelMessage->setLocalPos((int)(0.5 * (width - labelMessage->getWidth())), 15);

    // update labelgradient et mettre à jour la pos du label, mettre au dessus de labelMessage
    labelGradient->setLocalPos((int)(0.5 * (width - labelGradient->getWidth())), 35);//centre le message au bas de l'écran

    labelGradient->setText("X scan: " + cStr(scan_x) + "  /  " +
        "Y Scan : " + cStr(scan_y) + "  /  " +
        "Z Scan : " + cStr(scan_z));

    /////////////////////////////////////////////////////////////////////
    // VOLUME UPDATE
    /////////////////////////////////////////////////////////////////////

    // update region of voxels to be updated
    if (flagMarkVolumeForUpdate)
    {
        mutexVoxel.acquire();
        cVector3d min = volumeUpdate.m_min;
        cVector3d max = volumeUpdate.m_max;
        volumeUpdate.setEmpty();
        mutexVoxel.release();
        texture->markForPartialUpdate(min, max);
        flagMarkVolumeForUpdate = false;
    }

    // update position of cursor
    mutexDevices.acquire();
    cursorRobotPosDes->setLocalPos(robotPosDes - offset);
    mutexDevices.release();

    // update position of cursor for scanning
    mutexDevices.acquire();
    cursorRobotPosDesScan->setLocalPos(robotPosDesScan - offset);
    mutexDevices.release();


    /////////////////////////////////////////////////////////////// //////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, false);

    // render world
    camera->renderView(width, height);

    // wait until all OpenGL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//------------------------------------------------------------------------------

void setVoxel(cVector3d& a_pos, cColorb& a_color)
{
    // compute size of volume
    double sizeX = object->m_maxCorner.x() - object->m_minCorner.x();
    double sizeY = object->m_maxCorner.y() - object->m_minCorner.y();
    double sizeZ = object->m_maxCorner.z() - object->m_minCorner.z();

    // compute number of voxels for each side
    int numVoxelX = image->getWidth();
    int numVoxelY = image->getHeight();
    int numVoxelZ = image->getImageCount();

    // compute voxel indices
    int voxelIndexX = (int)((a_pos.x() - object->m_minCorner.x()) * numVoxelX / sizeX);
    int voxelIndexY = (int)((a_pos.y() - object->m_minCorner.y()) * numVoxelY / sizeY);
    int voxelIndexZ = (int)((a_pos.z() - object->m_minCorner.z()) * numVoxelZ / sizeZ);

    // clamp index values within image volume
    voxelIndexX = cClamp(voxelIndexX, 0, numVoxelX - 1);
    voxelIndexY = cClamp(voxelIndexY, 0, numVoxelY - 1);
    voxelIndexZ = cClamp(voxelIndexZ, 0, numVoxelZ - 1);

    // update voxel color
    object->m_texture->m_image->setVoxelColor(voxelIndexX, voxelIndexY, voxelIndexZ, a_color);

    // mark voxel for update
    mutexVoxel.acquire();
    volumeUpdate.enclose(cVector3d(voxelIndexX, voxelIndexY, voxelIndexZ));
    mutexVoxel.release();
    flagMarkVolumeForUpdate = true;

}



//------------------------------------------------------------------------------

void updateSensor(void)
{
    //--------------------------------------------------------------------------
    // INIT I/O BOARD
    //--------------------------------------------------------------------------

    int Row, Col;
    int BoardNum = 0;
    int ULStat = 0;
    int LowChan = 0;
    int HighChan = 0;
    int Range = UNI5VOLTS;
    short Status = 0;
    long CurCount;
    long CurIndex;
    int Count = 100;
    long Rate = 100000;
    unsigned Options;
    HANDLE MemHandle = 0;
    WORD* ADData;
    DWORD* ADData32;
    float    RevLevel = (float)CURRENTREVNUM;
    BOOL HighResAD = FALSE;
    int  ADRes;

    // Declare UL Revision Level
    ULStat = cbDeclareRevision(&RevLevel);

    // Initiate error handling
    //     Parameters:
    //         PRINTALL :all warnings and errors encountered will be printed
    //         DONTSTOP :program will continue even if error occurs.
    //                  Note that STOPALL and STOPFATAL are only effective in
    //                  Windows applications, not Console applications.
    cbErrHandling(PRINTALL, DONTSTOP);

    // Get the resolution of A/D
    cbGetConfig(BOARDINFO, BoardNum, 0, BIADRES, &ADRes);

    // If ADRes is equal to zero, exit as card has not acquisition card has not been detected
    if (ADRes == 0)
    {
        return;
    }

    // check If the resolution of A/D is higher than 16 bit.
    //    If it is, then the A/D is high resolution.
    if (ADRes > 16)
        HighResAD = TRUE;

    //  set aside memory to hold data
    if (HighResAD)
    {
        MemHandle = cbWinBufAlloc32(Count);
        ADData32 = (DWORD*)MemHandle;
    }
    else
    {
        MemHandle = cbWinBufAlloc(Count);
        ADData = (WORD*)MemHandle;
    }

    if (!MemHandle)
    {
        printf("\nout of memory\n");
        exit(1);
    }

    WORD DataValue;
    ULStat = cbFromEngUnits(BoardNum, Range, 0.0, &DataValue);
    ULStat = cbAOut(BoardNum, 0, Range, DataValue);

    // Collect the values with cbAInScan() in BACKGROUND mode
    //     Parameters:
    //          BoardNum    :the number used by CB.CFG to describe this board
    //          LowChan     :low channel of the scan
    //          HighChan    :high channel of the scan
    //          Count       :the total number of A/D samples to collect
    //          Rate        :sample rate in samples per second
    //          Gain        :the gain for the board
    //          ADData[]    :the array for the collected data values
    //          Options     :data collection options
    Options = NOCONVERTDATA + FOREGROUND; // BACKGROUND;

    // main haptic simulation loop
    while (simulationRunning)
    {
        // start data scanning operation
        ULStat = cbAInScan(BoardNum, LowChan, HighChan, Count, &Rate, Range, MemHandle, Options);

        // wait for data aquisition to complete
        Status = RUNNING;
        while (Status == RUNNING)
        {
            // check the status of the current background operation
            // parameters:
            //     BoardNum  :the number used by CB.CFG to describe this board
            //     Status    :current status of the operation (IDLE or RUNNING)
            //     CurCount  :current number of samples collected
            //     CurIndex  :index to the last data value transferred
            //     FunctionType: A/D operation (AIFUNCTIOM)
            ULStat = cbGetStatus(BoardNum, &Status, &CurCount, &CurIndex, AIFUNCTION);
        }

        // process data
        for (int i = 0; i < Count; i++)
        {
            // get next data value
            WORD dataValue = 0;

            if (HighResAD)
            {
                dataValue = ADData32[i];
            }
            else
            {
                dataValue = ADData[i];
            }
            
            // convert data value to a sensor voltage level
            voltageLevel = 5.0 * (((double)(dataValue)-2048.0) / 964.0);
            
            //Apply a gaussian filter to smoothen sensor values
            static KalmanFilter filter(voltageLevel,0.1,0.2);
            voltageLevel = filter.applyFilter(voltageLevel);

            // compute a haptic damping factor based on laser signal
            double dampingGain = 0.4;
            hapticDampingFactor = dampingGain * voltageLevel;
            
            // display data to scope
            scope->setSignalValues(voltageLevel);

            //draw_pixels();

            //compute gradient along direction of travel:
            double intensityAtPoint1=0;
            double intensityAtPoint2=0;
            if (stateGradient == STATE_AQUIREPOINT1)
            {

                intensityAtPoint1 = voltageLevel;//save voltage intensity at the current position of the robot
                robotDevice->getPosition(P1); //save current position of the robot
                stateGradient = STATE_AQUIREPOINT2;
             

            }
            else if (stateGradient == STATE_AQUIREPOINT2)
            {   
                
                cVector3d deltaVector;
                robotDevice->getPosition(P2);//save current position of the robot
                deltaVector= P2 - P1;// calcul of the displacement in z-axis

                if (deltaVector.length() > deltaz)// check if the displacement in z-axis is big enough to calulate a gradient 
                {
                    intensityAtPoint2 = voltageLevel;
                    float gradX, gradY, gradZ;
                    if (deltaVector.x() > deltaz)  gradX = ((intensityAtPoint2 - intensityAtPoint1) / deltaVector.x());
                    else gradX = 0;
                    if (deltaVector.y() > deltaz)  gradY = ((intensityAtPoint2 - intensityAtPoint1) / deltaVector.y());
                    else gradY = 0;
                    if (deltaVector.z() > deltaz)  gradZ = ((intensityAtPoint2 - intensityAtPoint1) / deltaVector.z());
                    else gradZ = 0;
                    cVector3d temp(gradX, gradY , gradZ);
                    voltageGradient = temp; //calculate the gradient
                    //gradientData.push_back(gradient);
                    stateGradient = STATE_AQUIREPOINT1;
                }
                else
                {
                    stateGradient = STATE_AQUIREPOINT2;
                }


            }
        }

        // update frequency counter
        freqCounterSensor.signal(Count);
    }

    // the BACKGROUND operation must be explicitly stopped
    //     Parameters:
    //          BoardNum    :the number used by CB.CFG to describe this board
    //          FunctionType: A/D operation (AIFUNCTIOM)
    ULStat = cbStopBackground(BoardNum, AIFUNCTION);

    cbWinBufFree(MemHandle);

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------

void updateRobotDevice(void)
{
    // activate robot forces
    robotDevice->enableForces(true);

    // get time point 0
    chrono::high_resolution_clock::time_point timePoint0 = chrono::high_resolution_clock::now();

    

    // main robot control loop
    while (simulationRunning)
    {
        // get time point 1

        chrono::high_resolution_clock::time_point timePoint1 = chrono::high_resolution_clock::now();
        double timeInSeconds = chrono::duration<double>(timePoint1 - timePoint0).count();

        // get current position of robot
        cVector3d pos(0, 0, 0);
        robotDevice->getPosition(pos);

        // get current velocity of robot
        cVector3d vel(0, 0, 0);
        robotDevice->getLinearVelocity(vel);

        // acquire mutex
        mutexDevices.acquire();

        // update variables by taking into account rotation matrix of robot
        robotPosCur = robotRot * pos;
        robotVelCur = robotRot * vel;

        // compute spring force to move robot toward desired position (robotPosDes) 
        double Kp = 200;
        double Kv = 10;
        cVector3d force = Kp * (robotPosDes - robotPosCur) - Kv * robotVelCur;
        
        auto_scan();
        // release mutex
        mutexDevices.release();

        // apply force after taking into account rotation matrix of robot
        robotDevice->setForce(cTranspose(robotRot) * force);

        draw_pixels();

        // update frequency counter
        freqCounterRobotDevice.signal();


    }

    // close connection to robot
    robotDevice->close();
}

//------------------------------------------------------------------------------


void updateHapticDevice(void)
{
    // variable to store robot device position when user button is pressed
    cVector3d robotPosDes0;

    // variable to store haptic device position when user button is pressed
    cVector3d hapticPos0;

    // activate haptic forces
    hapticDevice->enableForces(true);

    // set spring stiffness constant
    double Kp = 200;

    // main haptic control loop
    while (simulationRunning)
    {
        //--------------------------------------------------------------------------
        // GET DATA FROM HAPTIC DEVICE
        //--------------------------------------------------------------------------

        // get current position of haptic device
        cVector3d hapticPos(0, 0, 0);
        hapticDevice->getPosition(hapticPos);

        // get current velocity of haptic device
        cVector3d hapticVel(0, 0, 0);
        hapticDevice->getLinearVelocity(hapticVel);

        // get user button state
        bool userButton0 = false;
        hapticDevice->getUserSwitch(0, userButton0);

        bool userButton1 = false;
        hapticDevice->getUserSwitch(1, userButton1);

        bool userButton = userButton0 | userButton1;


        //--------------------------------------------------------------------------
        // STATE MACHINE
        //--------------------------------------------------------------------------

        // acquire mutex
        mutexDevices.acquire();

        // initialize haptic force
        cVector3d force(0.0, 0.0, 0.0);

        //
        // STATE IDLE
        //
        if (state == STATE_IDLE)
        {
            
            // check if user has pressed the button
            if (userButton == true)
            {
                // reset clutching positions for robot device and haptic device
                robotPosDes0 = robotPosDes;
                hapticPos0 = hapticPos;

                // go into teleoperation mode
                state = STATE_TELEOPERATION;
            }
        }

        //
        // STATE TELEOPERATION
        //
        if (state == STATE_TELEOPERATION)
        {
            if (userButton == false)
            {
                // user has released button, go into idle mode
                state = STATE_IDLE;
                //cout << "Max at :" << maxPosition.x() << " " << maxPosition.y() << " " << maxPosition.z() << endl;
                //cout << "Robot position at :" << robotPosDes.x() << " " << robotPosDes.y() << " " << robotPosDes.z() << endl;
            }
            else
            {
                // compute new desired robot position based on new haptic device position
                robotPosDes = robotPosDes0 + scaleFactor * (hapticPos - hapticPos0);

                //////////////////////////////////////////////////////////////////////////////////////////////
                // 
                // compute a haptic spring force between the desired robot position and its current position.
                // if the robot cannot reach a desired position, the user will feel the constraint as a spring
                // force.
                // 
                //////////////////////////////////////////////////////////////////////////////////////////////

                // set spring stiffness constant
                //double Kp = 200;

                // compute spring force
                force = Kp * (robotPosCur - robotPosDes);


                //////////////////////////////////////////////////////////////////////////////////////////////
                // 
                // compute a damping force based of the laser sensor data.
                // 
                //////////////////////////////////////////////////////////////////////////////////////////////

                // set damping constant
                double Kv = 10;

                // set damping factor based of hapticDampingFactor value computed in laser sensor thread; the value
                // is clamped between 0.0 and 1.0 to avoid any instabilities from the haptic device
                double damping = cClamp(hapticDampingFactor, 0.0, 1.0);
                //double exp_coefficient = 0.01;
                
                // calculate damping force as proportional de haptic device velocity; add force to previously computed force
                force += -cClamp( Kv * pow(voltageLevel,3)/ hapticVel.length(),0.0, 30.0)*hapticVel;

            }
        }


        //////////////////////////////////////////////////////////////////////////////////////////////
        // 
        // compute a planar haptic spring force on all three axis. This force is computed 
        // for both IDLE and TELEOPERATION states
        // 
        ////////////////////////////////////////////////////////////////////////////////////////////// 
        cVector3d springforce;
        double forcex = 0;
        double forcey = 0;
        double forcez = 0;
        Kp = 20; // définir coefficent approprié. 
        double Kv = 5;

        // Compute the force to lock haptic in x,y or z direction according to which variable is set to true
        axis_locking(&forcex, &forcey, &forcez);
        force += cVector3d(forcex, forcey, forcez);
        //force.set(forcex, forcey, forcez);//applying force to the haptic device


        // release mutex
        mutexDevices.release();


        //--------------------------------------------------------------------------
        // SEND FORCE TO HAPTIC DEVICE
        //--------------------------------------------------------------------------

        // apply force to haptic device
        hapticDevice->setForce(force);

        // update frequency counter
        freqCounterHapticDevice.signal();
    }

    // close connection to haptic device
    hapticDevice->close();
}

// This function locks restricts mouvement along x,y or z  axis
// by applying a spring force to the axis coordinate, that is set when
// user presses the button.
void axis_locking(double* forcex, double* forcey, double* forcez) {
    double K_axis = 2000, Kv = 5;
    cVector3d hapticPos(0, 0, 0);
    hapticDevice->getPosition(hapticPos);

    // get current velocity of haptic device
    cVector3d hapticVel(0, 0, 0);
    hapticDevice->getLinearVelocity(hapticVel);

    if (lock_x) *forcex += K_axis * (posX.x() - hapticPos.x());
    else if (!lock_x)  *forcex += 0;

    if (lock_y) *forcey += K_axis * (posY.y() - hapticPos.y());
    else if (!lock_y) *forcey += 0;

    if (lock_z) *forcez += K_axis * (posZ.z() - hapticPos.z());
    else if (!lock_z) *forcez += 0;

    return;
}

// This function does a z-axis sweep across the sample to the find the 
// maximum z-coordinate, which corresponds to the surface of the sample
void auto_scan(void) {
    if (scan_x == true || scan_y == true || scan_z == true) {
        static int i = 0;
        if (i % 10 == 0) {
            if (scan_x) outFile << robotPosCur.x() << "," << voltageLevel << endl;
            else if (scan_y) {
                outFile << robotPosCur.y() << "," << voltageLevel << endl;
            }
            else if (scan_z) {
                outFile << robotPosCur.z() << "," << voltageLevel << endl;
                maxSignal = updateMax(robotPosCur, voltageLevel, maxPosition, false);
                threshold = 0.2 * maxSignal;
            }
            outFile.flush();

            i = 0;

        }
        i++;

        //find the focus plane
        cVector3d maxPos;
        cVector3d scan_vector(0, 0, 0);

        if (scan_x) scan_vector.set(0.0000001, 0, 0);
        else if (scan_y) scan_vector.set(0, 0.0000001, 0);
        else if (scan_z) scan_vector.set(0, 0, 0.000001);
        if(abs(robotPosDes.y()-robotPosCur.y()) < 0.01) robotPosDes = robotPosDes + scan_vector;
        cout << robotPosCur.x() << " " << robotPosCur.y() << " " << robotPosCur.z() << endl;
    }
    return;
}

//Ths fuction draws colored pixels at the robots position 
// according to the voltage reading signal
void draw_pixels(void) {
    // draw a voxel if voltage level reaches a certain value
    cout << scan_z << " " << scan_finished << endl;
    if (!scan_z && scan_finished) {
        cColorb color(255,255,255);
        if (voltageLevel > 0.9 * threshold) {
            color.set(255, 165, 0, 200);  // Opaque Red
        }
        else if (voltageLevel > 0.7 * threshold) {
            color.set(255, 165, 0, 200);  // Orange (less opaque)
        }
        else if (voltageLevel > 0.5 * threshold) {
            color.set(255, 255, 0, 150);  // Yellow (more transparent)
        }
        else if (voltageLevel > 0.2 * threshold) {
            color.set(0, 255, 0, 100);  // Green (even more transparent)
        }
        else if (voltageLevel > 0.05 * threshold) {
            color.set(255, 255, 255, 50);  // Very transparent white
        }
        else {
            color.set(255, 255, 255, 20);  // Extremely transparent white
        }

        
        static cVector3d posi(-0, -0, -0);
        posi = posi + cVector3d(0.000001, 0.000001, 0.000001);
        setVoxel(posi, color);
        
        setVoxel(robotPosCur-offset, color);

    }

}

//This function resest all drawn pixels 
void reset_pixels() {
    // Get volume dimensions
    int numVoxelX = object->m_texture->m_image->getWidth();
    int numVoxelY = object->m_texture->m_image->getHeight();
    int numVoxelZ = object->m_texture->m_image->getImageCount();

    // Loop through all voxels and set them to the reset color
    for (int x = 0; x < numVoxelX; x++) {
        for (int y = 0; y < numVoxelY; y++) {
            for (int z = 0; z < numVoxelZ; z++) {
                object->m_texture->m_image->setVoxelColor(x, y, z, cColorb(0,0,0,0));
            }
        }
    }

    // Mark the whole volume for update
    mutexVoxel.acquire();
    volumeUpdate.enclose(cVector3d(0, 0, 0));
    volumeUpdate.enclose(cVector3d(numVoxelX - 1, numVoxelY - 1, numVoxelZ - 1));
    mutexVoxel.release();

    flagMarkVolumeForUpdate = true;
}
