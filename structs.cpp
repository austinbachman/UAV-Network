// Program Information ////////////////////////////////////////////////////////
/**
 * @file structs.cpp
 *
 * @details Contains definitions for all structs used in simulator
 *
 * @note None
 */

 // Precompiler directives /////////////////////////////////////////////////////

#ifndef STRUCTS_CPP
#define STRUCTS_CPP

// Global Constants ///////////////////////////////////////////////////////////

static const int FLEET_SIZE = 20;
static const int MAX_CONNECTION_DIST = 20;

int dronesUsed = 0;

// Struct Definitions /////////////////////////////////////////////////////////

/**
 * @brief Drone struct is used to hold all relative data for each Drone in the
 * 	 	  simulator
 *
 * @details Uses primitive data type fields to store data such as the drone
 *  	 	identifier, battery life, and position. Also Drone pointers are
 * 	 	 	used to establish relays of Drones
 */ 
struct Drone
{
    int droneID;
    int battery;
    int xPos;
    int yPos;
    int disNum; //number of disaster linking to
    bool inFlight;
    bool firstDrone; //closest to base station
    Drone* previous;
    Drone* next;
};

/**
 * @brief Disaster struct is used to hold all relative data for each Disaster
 * 	 	  in the simulator
 *
 * @details Uses primitive data type fields to store data such as the disaster
 *  	 	identifier and position
 */ 
struct Disaster
{
    int xPos;
    int yPos;
    int disNum;
};

// Terminating precompiler directives  ////////////////////////////////////////

#endif
