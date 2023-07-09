#include <string>
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <iomanip>
#include <csignal>
#include <limits.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include "vehicle.hpp"


/* Signal handling for test inputs */

volatile bool wantsEnvironmentInput;
volatile bool wantsVehicleInput;

void environment_handler(int signum) {
    wantsEnvironmentInput = true;
}

void vehicle_handler(int signum) {
    wantsVehicleInput = true;
}


/* Sensor Fusion */

class IMU {

    private:
        
    double currentVelocity;    // current speed of the vehicle

    public:

    IMU() {
        currentVelocity = 0;
    }

    IMU(double velo) {
        currentVelocity = velo;
    }
    
    double getCurrentVelocity() { return currentVelocity; }

    void setCurrentVelocity(double velo) { currentVelocity = velo; }

};


class Scanners {
    
    private:

    double lane_width;    // total lane width of the current road
    double right_line;    // distance to right line of lane
    double left_line;     // distance to left line of lane

    bool marked_road;     // if the lanes are marked on the current road

    public:

    Scanners() {
        lane_width = 8;
        right_line = 1;
        left_line = 1;
        marked_road = false;
    }

    Scanners( double width, double right, double left, bool marked) {
        lane_width = width;
        right_line = right;
        left_line = left;
        marked_road = marked;
    }

     void setLaneWidth(double width) {
        if(!marked_road) return;
        this->lane_width = width > 7 ? width : 7;
        this->right_line = (width - 6) / 2;
        this->left_line = (width - 6) / 2;
    }

    void setMarkedRoad(bool marked) { this->marked_road = marked; }

    double getLaneWidth() {
        if(!marked_road) return -1.0;
        return this->lane_width;
    }

    double distanceFromLineRight() {
        if(!marked_road) return -1.0;
        return this->right_line;
    }

    double distanceFromLineLeft() {
        if(!marked_road) return -1.0;
        return this->left_line;
    }

    bool onMarkedRoad() { return this->marked_road; }

};


class GPS {
    
    private:
    
    bool onHighway;            // true if on a highway, false if not
    bool onLocalRoute;         // true if on a local road, false if not

    int numberOfLanes;         // number of lanes on the current road
    int laneNumber;            // the lane the car is in on the current road (1 is lane furthest left)
    
    public:

    GPS() {
        onHighway = false;
        onLocalRoute = false;
        numberOfLanes = 1;
        laneNumber = 1;
    }

    GPS(bool H, bool L, int num_lanes, int lane) {
        if(H && L) {
            onHighway = false;
            onLocalRoute = false;
        } else {
            onHighway = H;
            onLocalRoute = L;
        }
        numberOfLanes = num_lanes > 0 ? num_lanes : 1;
        if(lane < 1) laneNumber = 1;
        else if (lane > numberOfLanes) laneNumber = numberOfLanes;
        else laneNumber = lane;
    }

    void setOnHighway() {
        this->onHighway = true;
        this->onLocalRoute = false;
    }
    
    void setOnLocalRoad() {
        this->onHighway = false;
        this->onLocalRoute = true;
    }

    void setOnUnregisteredRoad() {
        this->onHighway = false;
        this->onLocalRoute = false;
    }

    void setNumberOfLanes(int num) { if(num > 0) this->numberOfLanes = num; }

    void setLaneNumber(int lane) { if(lane > 0 && lane <= this->numberOfLanes) this->laneNumber = lane; }
    
    bool isOnHighway() { return this->onHighway; }

    bool isOnLocalRoute() { return this->onLocalRoute;}

    bool isOnUnregisteredRoad() { return !this->onHighway && !this->onLocalRoute; }

    int getNumberOfLanes() { return this->numberOfLanes; }
    
    int getLaneNumber() { return this->laneNumber; }
    
};


class SensorsAndCameras {

    private:
    
    double lightLevel;         // light level outside
    double distanceInFront;    // distance of car in front
    double distanceBehind;     // distance of car behind

    bool objectRight;       // true if object to right, false if not
    bool objectLeft;        // true if object to left, false if not
    bool rainDetected;      // true if its raining, false if its not
    
    public:

    SensorsAndCameras() {
        this->lightLevel = 200;
        this->distanceInFront = INT_MAX;
        this->distanceBehind = INT_MAX;
        this->objectRight = false;
        this->objectLeft = false;
        this->rainDetected = false;
    }
    
    void setLightLevel(double level) { this->lightLevel = level; }

    void setDistanceInFront(double distance) { this->distanceInFront = distance; }

    void setDistanceBehind(double distance) { this->distanceBehind = distance; }

    void setObjectRight(bool value) { this->objectRight = value; }

    void setObjectLeft(bool value) { this->objectLeft = value; }

    void setRain(bool value) { this->rainDetected = value; }

    double getLightLevel() const { return this->lightLevel; }
     
    double getDistanceInFront() const { return this->distanceInFront; }

    double getDistanceBehind() const { return this->distanceBehind; }

    bool isObjectRight() const { return this->objectRight; }

    bool isObjectLeft() const { return this->objectLeft; }

    bool getRainDetected() { return this->rainDetected; }

};


/* Vehicle Control */

class VehicleControl {

    private:

    bool ccActive;              // true is on, false is off
    bool windshieldWipers;      // true is on, false is off

    int headlightLevel;         // 0 is off, 1 is on, 2 is high beams
    int gear;                   // 0 P, 1 R, 2 N, 3 D
    int turnSignal;             // -1 left, 0 none, 1 right

    public:

    VehicleControl() {
        this->ccActive = false;
        this->headlightLevel = 0;
        this->gear = 0;
        this->windshieldWipers = false;
        this->turnSignal = 0;
    }

    VehicleControl(bool cc, bool inDrive) {
        this->ccActive = cc;
        this->headlightLevel = 0;
        this->gear = inDrive ? 3 : 0;
        this->windshieldWipers = false;
        this->turnSignal = 0;
    }

    void startCC(IMU &imu, GPS &gps) { if(gps.isOnHighway() && imu.getCurrentVelocity() > 0) ccActive = true; }
    void stopCC() { ccActive = false; }

    void setGear(int val) { if (val >= 0 && val <= 3) this->gear = val; }

    void turnOffHeadlights() { this->headlightLevel = 0; }
    void turnOnHeadLights(int level) {
        if(level == 0) this->headlightLevel = 0;
        if(level == 1) this->headlightLevel = 1;
        if(level > 1) this->headlightLevel = 2;
    }

    void leftTurnSignal() { this->turnSignal = -1; }
    void rightTurnSignal() { this->turnSignal = 1; }
    void turnComplete() { turnSignal = 0; }

    void turnOnWindshieldWipers(bool wipers) { this->windshieldWipers = wipers; }

    bool getccActive() { return this->ccActive; }

    int getHeadLightLevel() { return this->headlightLevel; }

    int getGear() { return this->gear; }

    int getTurn() { return this->turnSignal; }

    bool windshieldWipersOn() { return this->windshieldWipers; }

    void brake(IMU &imu, SensorsAndCameras &sensorsAndCameras, int intensity) {  // intensity = 1, 2, 3,  - for if in reverse
        bool reversing = imu.getCurrentVelocity() < 5;
        if(intensity == 1) {
            imu.setCurrentVelocity(imu.getCurrentVelocity() * .95);
            if(this->gear == 3 || this->gear == 2 || !reversing) {
                if(imu.getCurrentVelocity() < 5) imu.setCurrentVelocity(0);
                else sensorsAndCameras.setDistanceInFront(sensorsAndCameras.getDistanceInFront() + 10);
            } else if (this->gear == 1) {
                 if(imu.getCurrentVelocity() > -5) imu.setCurrentVelocity(0);
            }
        } else if( intensity == 2) {
            imu.setCurrentVelocity(imu.getCurrentVelocity() * .90);
            if(this->gear == 3 || this->gear == 2 || !reversing ) {
                if(imu.getCurrentVelocity() < 5) imu.setCurrentVelocity(0);
                else sensorsAndCameras.setDistanceInFront(sensorsAndCameras.getDistanceInFront() + 15);
            } else if (this->gear == 1) {
                 if(imu.getCurrentVelocity() > -5) imu.setCurrentVelocity(0);
            }
        } else {
            imu.setCurrentVelocity(imu.getCurrentVelocity() * .85);
            if(this->gear == 3 || this->gear == 2 || !reversing) {
                if(imu.getCurrentVelocity() < 5) imu.setCurrentVelocity(0);
                else sensorsAndCameras.setDistanceInFront(sensorsAndCameras.getDistanceInFront() + 20);
            } else if (this->gear == 1) {
                 if(imu.getCurrentVelocity() > -5) imu.setCurrentVelocity(0);
            }
        }
    }

    void brakeTo(IMU &imu, SensorsAndCameras &sensorsAndCameras, int speed) {
        brake(imu, sensorsAndCameras, 2);
        if((imu.getCurrentVelocity() <= speed && this->gear == 3) ||
            imu.getCurrentVelocity() >= speed && this->gear == 1) {
                imu.setCurrentVelocity(speed);
        }
        if(speed < 5 && this->gear == 3 && imu.getCurrentVelocity() < 5) imu.setCurrentVelocity(speed);
        if(speed > -5 && this->gear == 1 && imu.getCurrentVelocity() > -5) imu.setCurrentVelocity(speed);
    }

    void accelerateTo(IMU &imu, SensorsAndCameras &sensorsAndCameras, int speed) {
        if(imu.getCurrentVelocity() <= 10 && this->gear == 3 ) imu.setCurrentVelocity(10 * 1.2);
        else if (imu.getCurrentVelocity() >= -10 && this->gear == 1 ) imu.setCurrentVelocity(-10 * 1.2);
        else imu.setCurrentVelocity(imu.getCurrentVelocity() * 1.10);
        if(this->gear == 3) {
            sensorsAndCameras.setDistanceInFront(sensorsAndCameras.getDistanceInFront() - 10);
            sensorsAndCameras.setDistanceBehind(sensorsAndCameras.getDistanceBehind() + 10);
            if(imu.getCurrentVelocity() >= speed) {
                imu.setCurrentVelocity(speed);
            }
        } else if (this->gear == 1) {
            sensorsAndCameras.setDistanceInFront(sensorsAndCameras.getDistanceInFront() + 10);
            if(imu.getCurrentVelocity() <= speed) {
                imu.setCurrentVelocity(speed);
            }
        }
    }

};


/* Display */

class Display {

    private:

    struct status_struct status;

    public:

    Display() :
        status{0,0,false,false,false,false,false,false,-1,0,false,1,1,false,false}
    {}

    void set_status(status_struct stat) { status = stat; }

    void set_speed(int speed) { status.speed = speed; }
    
    void set_gear(int gear) { status.gear = gear; }

    void set_cruise_control_active(bool active) { status.cruise_control_active = active; }

    void set_wipers(bool on) { status.wipers_on = on; }

    void set_cars_in_front(bool in_front) { status.cars_in_front = in_front; }

    void set_cars_in_back(bool in_back) { status.cars_in_back = in_back; }

    void set_cars_on_left(bool on_left) { status.cars_on_left = on_left; }

    void set_cars_on_right(bool on_right) { status.cars_on_right = on_right; }

    void set_lane_warning(int warning) { status.lane_warning = warning; }

    void set_headlights(int level) { status.headlights = level; }

    void set_rearview(bool needsRearView) { status.rear_view = needsRearView; }

    void set_lane(int lane) { status.lane = lane; }

    void set_num_lanes(int num_lanes) { status.num_lanes = num_lanes; }

    void set_right_turn(bool right_turn) { status.rightTurn = right_turn; }

    void set_left_turn(bool left_turn) { status.leftTurn = left_turn; }

    void print_display() {

        // Clear the console
        std::cout << "\033[2J\033[1;1H";

        std::cout << "\n                                                      Alset-IoT Simulation:\n                                                Ctrl+C to change the environment\n                                                Ctrl+Z to make a vehicle input\n                                                -1 after sending signal to exit\n";
        
        std::cout <<
                "\n                                                            " << status.speed << " mph\n\n";
                    
        if (status.gear == 0) {
        std::cout <<
                "                                                            [P]ark\n\n";
        } else if (status.gear == 1) {
        std::cout <<
                "                                                          [R]everse\n\n";
        } else if (status.gear == 2) {
        std::cout <<
                "                                                          [N]eutral\n\n";
        } else if (status.gear == 3) {
        std::cout <<
                "                                                           [D]rive\n\n";
        }


        if (status.cars_in_front) {
        std::cout <<
                "                                                    |      CAR HERE      |\n";
        } else {
        std::cout <<
                "                                                    |                    |\n";
        }
        
        if (status.lane_warning == 0) {
        std::cout <<
                "                         ALERT! Lane Change Warning |                    |\n";
        } else if (status.lane_warning == 1) {
        std::cout <<
                "                                                    |                    | ALERT! Lane Change Warning\n";
        } else {
        std::cout <<
                "                                                    |                    |\n";
        }
        
        if (status.headlights == 2) {
        std::cout <<
                "                                                    |     \\   / \\   /    |\n";
        } else {
        std::cout <<
                "                                                    |                    |\n";
        }

        if (status.headlights == 1 || status.headlights == 2) {
        std::cout <<
                "                                                    |      \\ /   \\ /     |\n";
        } else {
        std::cout <<
                "                                                    |                    |\n";
        }

            
        std::cout <<
                "                                                    |      --------      |\n";

        std::cout <<
                "                                                    |    (|        |)    |\n";
        
        if (status.leftTurn) {
            std::cout <<
                "                                                    | <-- |        |     |\n";
        } else if (status.rightTurn) {
            std::cout <<
                "                                                    |     |        | --> |\n";
        } else {
            std::cout <<
                "                                                    |     |        |     |\n";
        }

        if (status.cars_on_left && status.cars_on_right) {
            std::cout <<
                "                                     CAR            |     |        |     |      CAR\n";
        } else if (status.cars_on_left) {
            std::cout <<
                "                                     CAR            |     |        |     |\n";
        } else if (status.cars_on_right) {
            std::cout <<
                "                                                    |     |        |     |      CAR\n";
        } else {
            std::cout <<
                "                                                    |     |        |     |\n";
        }

        if (status.cars_on_left && status.cars_on_right) {
            std::cout <<
                "                                     HERE           |     |        |     |      HERE\n";
        } else if (status.cars_on_left) {
            std::cout <<
                "                                     HERE           |     |        |     |\n";
        } else if (status.cars_on_right) {
            std::cout <<
                "                                                    |     |        |     |      HERE\n";
        } else {
            std::cout <<
                "                                                    |     |        |     |\n";
        }

        std::cout <<
                "                                                    |    (|        |)    |\n";

        std::cout <<
                "                                                    |      --------      |\n";

        std::cout <<
                "                                                    |                    |\n";

        std::cout <<
                "                                                    |                    |\n";

        if (status.cars_in_back) {
            std::cout <<
                "                                                    |      CAR HERE      |\n";
        } else {
            std::cout <<
                "                                                    |                    |\n";
        }

        std::cout <<
                "                                                    |                    |\n";
        
        std::cout <<
                "                                                    |      lane: " << status.lane << "       |\n";

        std::cout <<
                "                                                    |                    |\n\n";

        if (status.cruise_control_active) {
            std::cout <<
                "                                                    Cruise Control Active\n\n";
        }

        if (status.wipers_on) {
            std::cout <<
                "                                                           Wipers on\n";
        }

        if (status.rear_view && status.cars_in_back) {
        std::cout <<
                "\n                                                     --------------------\n";
        std::cout <<
                "                                                    |  Rear View Camera  |\n";
        std::cout <<
                "                                                    |      CAR HERE      |\n";
        std::cout <<
                "                                                    |                    |\n";
        std::cout <<
                "                                                     --------------------\n";
        } else if (status.rear_view && !status.cars_in_back) {
        std::cout <<
                "\n                                                     --------------------\n";
        std::cout <<
                "                                                    |  Rear View Camera  |\n";
        std::cout <<
                "                                                    |                    |\n";
        std::cout <<
                "                                                    |                    |\n";
        std::cout <<
                "                                                     --------------------\n";
        }
    }

};


/* Planning */

class Planning {

    private:

    VehicleControl vehicleControl;
    IMU imu;
    Scanners scanners;
    GPS gps;
    SensorsAndCameras sensorsAndCameras;
    Display display;

    bool wantsToAcc;        // when car is told to accelerate
    bool wantsToBrk;        // when car is told to break
    int speed_wanted;       // the speed to accelerate or break to

    public:
    
    /* initialize an instance of every class */

    Planning() {
        vehicleControl = VehicleControl(true,true);
        imu = IMU(60);
        scanners = Scanners(12, 3, 3, true);
        gps = GPS(true, false, 4, 2);
        sensorsAndCameras = SensorsAndCameras();
        display = Display();
    }


    /* updates vehicle when case detected */

    void brakeWhenObjectDetected() {
        if(vehicleControl.getGear() == 2 || vehicleControl.getGear() == 3) {
            if (imu.getCurrentVelocity() != 0 && sensorsAndCameras.getDistanceInFront() > 20 && sensorsAndCameras.getDistanceInFront() < 100) {
                vehicleControl.brake(imu, sensorsAndCameras, 1);
                wantsToAcc = false;
            } else if (imu.getCurrentVelocity() != 0 && sensorsAndCameras.getDistanceInFront() > 10 && sensorsAndCameras.getDistanceInFront() <= 20) {
                vehicleControl.brake(imu, sensorsAndCameras, 2);
                wantsToAcc = false;
            } else if (imu.getCurrentVelocity() != 0 && sensorsAndCameras.getDistanceInFront() > 0 && sensorsAndCameras.getDistanceInFront() <= 10) {
                vehicleControl.brake(imu, sensorsAndCameras, 3);
                wantsToAcc = false;
            }
        } else if (vehicleControl.getGear() == 1) {
             if (imu.getCurrentVelocity() != 0 && sensorsAndCameras.getDistanceBehind() > 0 && sensorsAndCameras.getDistanceBehind() < 20) {
                vehicleControl.brake(imu, sensorsAndCameras, 3);
                wantsToAcc = false;
             }
        }
    }

    void automaticallyChangeLane() {
        if (imu.getCurrentVelocity() != 0 && vehicleControl.getccActive() && gps.getNumberOfLanes() > 1) {
            //if one of the turn signals is active
            if (vehicleControl.getTurn() < 0) {  //left turn
                if(!sensorsAndCameras.isObjectLeft() && gps.getLaneNumber() > 1) {
                    gps.setLaneNumber(gps.getLaneNumber() - 1);
                    vehicleControl.turnComplete();
                    sensorsAndCameras.setObjectRight(false);
                } else {
                    vehicleControl.turnComplete();
                }
            } else if (vehicleControl.getTurn() > 0) {  //right turn
                if(!sensorsAndCameras.isObjectRight() && gps.getLaneNumber() < gps.getNumberOfLanes()) {
                    gps.setLaneNumber(gps.getLaneNumber() + 1);
                    vehicleControl.turnComplete();
                    sensorsAndCameras.setObjectLeft(false);
                } else {
                    vehicleControl.turnComplete();
                }
            }
        }
    }

    void automaticHeadLights() {
        if ((sensorsAndCameras.getLightLevel() < 200 || sensorsAndCameras.getRainDetected()) && vehicleControl.getHeadLightLevel() == 0) {
            vehicleControl.turnOnHeadLights(1);
        } else if((sensorsAndCameras.getLightLevel() >= 200 && !sensorsAndCameras.getRainDetected()) && vehicleControl.getHeadLightLevel() > 0) {
            vehicleControl.turnOffHeadlights();
        }
    }

    void automaticaHighBeams() {
        if (sensorsAndCameras.getLightLevel() < 50
            && imu.getCurrentVelocity() > 25
            && !sensorsAndCameras.getRainDetected()
            && sensorsAndCameras.getDistanceInFront() >= 100) {
                if(vehicleControl.getHeadLightLevel() == 1 ) {
                    vehicleControl.turnOnHeadLights(2);
                }
            }
        else if(vehicleControl.getHeadLightLevel() == 2) {
            vehicleControl.turnOnHeadLights(1);
        }
    }

    void automaticWindshieldWipers() {
        if (sensorsAndCameras.getRainDetected()) {
            vehicleControl.turnOnWindshieldWipers(true);
        } else {
            vehicleControl.turnOnWindshieldWipers(false);
        }
    }

    void gearControl() {
        // No need to worry about neutral, not implemented yet
        if(vehicleControl.getGear() == 0 && imu.getCurrentVelocity() != 0) {
            imu.setCurrentVelocity(0);
        }
        else if(vehicleControl.getGear() == 1 && imu.getCurrentVelocity() > 0) {
            imu.setCurrentVelocity(0);
        }
        else if(vehicleControl.getGear() == 3 && imu.getCurrentVelocity() < 0) {
            imu.setCurrentVelocity(0);
        }

        if((vehicleControl.getGear() == 0 || vehicleControl.getGear() == 1 || vehicleControl.getGear() == 2)
            && vehicleControl.getccActive()) vehicleControl.stopCC();
        else if(vehicleControl.getGear() == 3 && !vehicleControl.getccActive()) vehicleControl.startCC(imu,gps);

    }

    void acc() {
        int gear = vehicleControl.getGear();
        if(wantsToAcc) {
            vehicleControl.accelerateTo(imu, sensorsAndCameras, speed_wanted);
            if(imu.getCurrentVelocity() >= speed_wanted && gear == 3) wantsToAcc = false;
            if(imu.getCurrentVelocity() <= speed_wanted && gear == 1) wantsToAcc = false;
        }
    }

    void brk() {
        int gear = vehicleControl.getGear();
        if(wantsToBrk) {
            vehicleControl.brakeTo(imu, sensorsAndCameras, speed_wanted);
            if(imu.getCurrentVelocity() <= speed_wanted && gear == 3 ) wantsToBrk = false;
            if(imu.getCurrentVelocity() >= speed_wanted && gear == 1 ) wantsToBrk = false;
        }
    }

    void check_all() {
        brakeWhenObjectDetected();
        acc();
        brk();
        automaticHeadLights();
        automaticallyChangeLane();
        automaticaHighBeams();
        automaticWindshieldWipers();
        gearControl();
    }


    /* updating display*/

    void automaticObjectDetection() {
        display.set_cars_in_front(sensorsAndCameras.getDistanceInFront() < 100);
        display.set_cars_in_back(sensorsAndCameras.getDistanceBehind() < 20);
        display.set_cars_on_left(sensorsAndCameras.isObjectLeft());
        display.set_cars_on_right(sensorsAndCameras.isObjectRight());
    }
    
    void wipersOn() {
        display.set_wipers(vehicleControl.windshieldWipersOn());
    }

    void headlightLevel() {
       display.set_headlights(vehicleControl.getHeadLightLevel());
    }

    void currentSpeed() {
        display.set_speed((int)imu.getCurrentVelocity());
    }

    void automaticRearCamera() {
        display.set_rearview(vehicleControl.getGear() == 1 && imu.getCurrentVelocity() <= 0);
    }

    void checkLanes() {
        display.set_lane(gps.getLaneNumber());
    }

    void checkTurn() {
        display.set_left_turn(vehicleControl.getTurn() == -1);
        display.set_right_turn(vehicleControl.getTurn() == 1);

    }

    void detectLaneDeparture() {
        if (imu.getCurrentVelocity() != 0 && !gps.isOnUnregisteredRoad()) {
            if (scanners.distanceFromLineLeft() <= 0) {
                display.set_lane_warning(0); // changing lane to the left
            }

            if (scanners.distanceFromLineRight() <= 0) {
                display.set_lane_warning(1); // changing lane to the right
            }
        }
    }

    void checkGear() {
        display.set_gear(vehicleControl.getGear());
    }

    void checkCC() {
        display.set_cruise_control_active(vehicleControl.getccActive());
    }

    void checkWarnings() {
        if (vehicleControl.getTurn() == 0
        || (vehicleControl.getTurn() == -1 && !sensorsAndCameras.isObjectLeft())
        || (vehicleControl.getTurn() == 1 && !sensorsAndCameras.isObjectRight())) {
            display.set_lane_warning(-1);
        } else if (vehicleControl.getTurn() == -1 && sensorsAndCameras.isObjectLeft()) {
            display.set_lane_warning(0);
        } else if (vehicleControl.getTurn() == 1 && sensorsAndCameras.isObjectRight()) {
            display.set_lane_warning(1);
        }
    }

    void updateDisplay() {
        checkGear();
        checkTurn();
        checkLanes();
        automaticObjectDetection();
        wipersOn();
        headlightLevel();
        currentSpeed();
        detectLaneDeparture();
        automaticRearCamera();
        checkWarnings();
        checkCC();
    }

    
    /* Run system */

    void run_systems() {

        display.print_display();

        signal(SIGINT, environment_handler);
        signal(SIGTSTP, vehicle_handler);
        
        while(true) {

            check_all();
            updateDisplay();
            display.print_display();

            if (wantsEnvironmentInput) {

                wantsEnvironmentInput = false;

                int input;
                std::cout << "\n\n                   0: default, 1: car in front, 2: car behind, 3: car to the side, 4: light level, 5: toggle rain\n";
                std::cout << "\n                   Change Environment: ";
                std::cin >> input;
                int val;

                switch(input) {
                    case -1:
                        exit(0);
                    case 0:
                        sensorsAndCameras = SensorsAndCameras();
                        break;
                    case 1:
                        std::cout << "                   Distance in front: ";
                        std::cin >> val;
                        sensorsAndCameras.setDistanceInFront(val);
                        break;
                    case 2:
                        std::cout << "                   Distance behind: ";
                        std::cin >> val;
                        sensorsAndCameras.setDistanceBehind(val);
                        break;
                    case 3:
                        std::cout << "                   Object left (-1) or right (1): ";
                        std::cin >> val;
                        if(val < 0) sensorsAndCameras.setObjectLeft(true);
                        else if(val > 0) sensorsAndCameras.setObjectRight(true);
                        else {
                            sensorsAndCameras.setObjectLeft(false);
                            sensorsAndCameras.setObjectRight(false);
                        }
                        break;
                    case 4:
                        std::cout << "                   Light Level: ";
                        std::cin >> val;
                        if(val < 0) val = 0;
                        sensorsAndCameras.setLightLevel(val);
                        break;
                    case 5:
                        std::cout << "                   Rain on (1) or off (0): ";
                        std::cin >> val;
                        sensorsAndCameras.setRain(val > 0);
                        break;
                    default:
                        break;
                    }
                
                updateDisplay();
                display.print_display();
            }

            if (wantsVehicleInput) {

                wantsVehicleInput = false;

                int input;
                std::cout << "\n\n                            0: default, 1: apply brake, 2: accelerate, 3: change gear, 4: turn signal\n";
                std::cout << "\n                            Vehicle Input: ";
                std::cin >> input;
                int val;

                switch(input) {
                    case -1:
                        exit(0);
                    case 0:
                        imu = IMU(60);
                        gps = GPS(true, false, 4, 2);
                        break;
                    case 1:
                        std::cout << "                            Brake to what speed? ";
                        std::cin >> val;
                        if( (vehicleControl.getGear() == 3 && val < 0
                            || val > imu.getCurrentVelocity()
                            ) ||
                            (vehicleControl.getGear() == 1 && val > 0 ) ||
                            vehicleControl.getGear() == 0 ) break;
                        wantsToBrk = true;
                        wantsToAcc = false;
                        speed_wanted = val;
                        break;
                    case 2:
                        std::cout << "                            Accelerate to what speed? ";
                        std::cin >> val;
                        if( (vehicleControl.getGear() == 3 && val < 0) ||
                            (vehicleControl.getGear() == 1 && val > 0 ) ||
                            vehicleControl.getGear() == 0 ) break;
                        wantsToAcc = true;
                        wantsToBrk = false;
                        speed_wanted = val;
                        break;
                    case 3:
                        if(imu.getCurrentVelocity() < -5 || imu.getCurrentVelocity() > 5) {
                            std::cout << "                            Can only change gear at low speeds\n";
                            std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                            break;
                        }
                        std::cout << "                            Change gear to park (0), reverse (1), drive (3)? ";
                        std::cin >> val;
                        if(val == 0 || val == 1 || val == 3) vehicleControl.setGear(val);
                        // note that neutral is not fully implemented so it is not included
                        break;
                    case 4:
                        std::cout << "                            Turn signal left (-1) or right (1): ";
                        std::cin >> val;
                        if(val < 0) vehicleControl.leftTurnSignal();
                        else if(val > 0) vehicleControl.rightTurnSignal();
                        break;
                    default:
                        break;
                    }
                
                updateDisplay();
                display.print_display();
            }

            for (int i = 0; i < 40; i++) {
                if (!wantsEnvironmentInput && !wantsVehicleInput) std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        }

    }

};



