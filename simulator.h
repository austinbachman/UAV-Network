// Program Information ////////////////////////////////////////////////////////
/**
 * @file simulator.h
 *
 * @details Header file for simulator which provides prototypes for all
 *          functions used in simulator
 *
 * @Note None
 */

// Precompiler directives /////////////////////////////////////////////////////

#ifndef SIMULATOR_H
#define SIMULATOR_H

// Header Files ///////////////////////////////////////////////////////////////

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include "structs.cpp"

using namespace std;

// Function Prototypes ////////////////////////////////////////////////////////

void generateFleet( Drone*& fleet );
void makeDisaster( Disaster& dis, const int disCount );
bool generateRelay( const Disaster& dis, Drone*& fleet );
int highestPower( const Drone* fleet );
int distance( int x1, int y1, int x2, int y2 );
void displayRelay( Drone* fleet, int disCount );
float getDirVector( int x1, int y1, int x2, int y2 );
int findClosest( const Drone* fleet, const Disaster& dis );
int getUserChoice();
void simulateTime( Drone*& fleet );
void updateBatteries( Drone*& fleet, const int currentTime );
bool replaceDrone( const int index, Drone*& fleet, const int currentTime );
bool sendPacket( Drone*& fleet );
void printMap(Drone* fleet, vector<Disaster> disasterSet);

// Terminating precompiler directives  ////////////////////////////////////////

#endif
