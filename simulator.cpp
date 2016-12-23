// Program Information ////////////////////////////////////////////////////////
/**
 * @file simulator.cpp
 *
 * @details Implementation file for simulator which provides function
 * 	 	 	definitions and documentation
 *
 * @Note None
 */

 // Precompiler directives /////////////////////////////////////////////////////

#ifndef SIMULATOR_CPP
#define SIMULATOR_CPP

// Header File ////////////////////////////////////////////////////////////////

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include "simulator.h"

using namespace std;

// Begin Function Definitions /////////////////////////////////////////////////

/**
 * @brief Creates an array of Drones and initializes all Drones with default
 *        values for all data fields
 *
 * @details Each Drone is given an ID, a random starting battery level 
 *          between 50-100 (100 is a full battery, 0 is an empty battery),
 *          and default values for the rest of the fields
 *          
 * @param in: fleet
 *            An empty array of Drones passed by reference to be filled with data
 *
 * @return void
 *
 * @note None
 */
void generateFleet( Drone*& fleet )
{
    int index;
    
    fleet = new Drone[FLEET_SIZE];
    
    for( index = 0; index < FLEET_SIZE; index++ )
    {
        fleet[index].droneID = index;
        fleet[index].battery = (50 + rand() % 51); //battery at random value 50-100
        fleet[index].xPos = 0;
        fleet[index].yPos = 0;
        fleet[index].disNum = -1;
        fleet[index].previous = NULL;
        fleet[index].next = NULL;
        fleet[index].inFlight = false;
        fleet[index].firstDrone = false;
    }
}

/**
 * @brief Takes input from the console of the coordinates of a new disaster
 *
 * @details Disaster coordinates must be between 1-100 for x and y. Coordinates
 *          are saved into the Disaster struct
 *          
 * @param in: dis
 *            A Disaster struct passed by reference for the coordinates to be
 *            stored in
 *
 *            disCount
 *            Used by main function to keep track of how many Disasters exist
 *
 * @return void
 *
 * @note None
 */
void makeDisaster( Disaster& dis, const int disCount )
{
    cout << "Enter x coordinate of disaster (1-100): ";
    cin >> dis.xPos;
    while( dis.xPos > 100 || dis.xPos < 1 )
    {
        cout << "Must be between 1 and 100: ";
        cin >> dis.xPos;
    }
    cout << "Enter y coordinate of disaster (1-100): ";
    cin >> dis.yPos;
    while( dis.yPos > 100 || dis.yPos < 1 )
    {
        cout << "Must be between 1 and 100: ";
        cin >> dis.yPos;
    }
    
    dis.disNum = disCount;
}

/**
 * @brief If possible assigns availible drones to a Disaster to establish a
 *        link to the Command and Control Center
 *
 * @details Uses calculated distances and Drone information to determine if
 *          a relay is possible. If it is Drones are assigned to the relay
 *          and marked in flight
 *          
 * @param in: dis
 *            Disaster in need of a Drone relay
 *
 *            fleet
 *            Drone Array of all Drones
 *
 * @return bool Evidence of successfully established relay 
 *
 * @note Only establishes a relay if possible to establish relay and maintain
 *       20% of total Drone count in Command and Control Center
 */
bool generateRelay( const Disaster& dis, Drone*& fleet )
{
    int nextIndex, last;
    int prevX = 0, prevY = 0;
    int closestDrone = findClosest( fleet, dis );
    int distanceToDisaster;
    int numDronesNeeded;
    Drone* prevDrone = NULL;
    nextIndex = highestPower( fleet );
    
    if( nextIndex < 0 )
    {
        return false;
    }
    
    if( closestDrone < 0 )
    {
        distanceToDisaster = distance( 0, 0, dis.xPos, dis.yPos );
        prevX = 0;
        prevY = 0;
    }
    else
    {
        distanceToDisaster = distance( fleet[closestDrone].xPos,
                    fleet[closestDrone].yPos, dis.xPos, dis.yPos );
        prevX = fleet[closestDrone].xPos;
        prevY = fleet[closestDrone].yPos;
        prevDrone = &(fleet[closestDrone]);
    }
    
    if( distanceToDisaster < MAX_CONNECTION_DIST - MAX_CONNECTION_DIST * .05 
        && closestDrone >= 0 )
    {
        return true;
    }
    
    numDronesNeeded = max( 1, (int)ceil( (float)distanceToDisaster
                                    / (float)MAX_CONNECTION_DIST ) );
    
    //keep 20% of total drones available to replace drones as they time out
    if( numDronesNeeded > FLEET_SIZE * 0.8 - dronesUsed )
    {
        return false;
    }
    
    do
    {
        if( nextIndex < 0 )
        {
            return false;
        }
        
        fleet[nextIndex].inFlight = true;
        dronesUsed++;
        fleet[nextIndex].disNum = dis.disNum;
        
        //calculate drone coordinates with 5% margin of error
        prevX = ceil( (float)(dis.xPos-prevX) / (float)max(numDronesNeeded, 1) )
                            + prevX - MAX_CONNECTION_DIST
                            * .05 * getDirVector( prevX, prevY, dis.xPos, dis.yPos);
        fleet[nextIndex].xPos = prevX;
        prevY = ceil( (float)(dis.yPos-prevY) / (float)max(numDronesNeeded, 1) )
                            + prevY - MAX_CONNECTION_DIST
                            * .05 * getDirVector( prevX, prevY, dis.yPos, dis.xPos);
        fleet[nextIndex].yPos = prevY;
        //cout << prevY << endl << prevX << endl;
        fleet[nextIndex].previous = prevDrone;
        if( prevDrone != NULL && prevDrone->disNum == dis.disNum )
        {
            prevDrone->next = &(fleet[nextIndex]);
        }
        else
        {
            fleet[nextIndex].firstDrone = true;
        }
        prevDrone = &(fleet[nextIndex]);
        last = fleet[nextIndex].droneID;
        nextIndex = highestPower( fleet );
        numDronesNeeded--;
    } while( distance( fleet[last].xPos, fleet[last].yPos, dis.xPos, dis.yPos )
                > MAX_CONNECTION_DIST );

    return true;
}

/**
 * @brief Returns the index of the Drone with the most full battery
 *
 * @details Loops through every Drone and finds the maximum battery value
 *          
 * @param in: fleet
 *            Drone array of all desired Drones to search through
 *        
 * @return int index of highest battery Drone
 *
 * @note None
 */
int highestPower( const Drone* fleet )
{
    int max = -1;
    int highestPower = 0;
    int index;
    for( index = 0; index < FLEET_SIZE; index++ )
    {
        if( fleet[index].battery > highestPower && !fleet[index].inFlight )
        {
            highestPower = fleet[index].battery;
            max = index;
        }
    }
    
    return max;
}

/**
 * @brief Calculates the distance between two points on an xy plane
 *
 * @details Uses the distance formula
 *          
 * @param in: x1
 *            First x coordinate
 *
 *            y1
 *            First y coordinate
 *
 *            x2
 *            Second x coordinate
 *
 *            y2
 *            Second y coordinate
 *
 * @return int result of distance calculation
 *
 * @note None
 */
int distance( int x1, int y1, int x2, int y2 )
{
    int x = x2-x1;
    int y = y2-y1;

    return sqrt( x*x + y*y );
}

/**
 * @brief Prints to console all relays to Disasters
 *
 * @details For every Disaster prints the relay information including DroneIDs,
 *          positions, and battery life 
 *          
 * @param in: fleet
 *            Drone array of all Drones
 *
 *            disCount
 *            Number of Disasters
 *        
 * @return void
 *
 * @note None
 */
void displayRelay( Drone* fleet, int disCount )
{
    int droneIndex;
    int disIndex;
    bool relayExists = false;
    bool lastNode = false;
    Drone* current;
    
    cout << endl;
    
    for( disIndex = 0; disIndex < disCount; disIndex++ )
    {
        //find first drone in relay
        for( droneIndex = 0; droneIndex < FLEET_SIZE; droneIndex++ )
        {
            if( fleet[droneIndex].firstDrone && fleet[droneIndex].disNum == disIndex )
            {
                relayExists = true;
                break;
            }
        }
        
        cout << "Disaster #" << disIndex << ":" << endl;
        
        if(relayExists)
        {
            current = &(fleet[droneIndex]);
            
            while( current != NULL )
            {
                cout << "Drone #" << current->droneID << " is at " << current->xPos << ", "
                     << current->yPos << ", battery = " << current->battery
                     << "%" << endl;
                
                current = current->next;
            }
        }
        else
        {
            cout << "Covered by other disaster relay" << endl;
        }
        
        relayExists = false;
    }

}

/**
 * @brief Calculates the direction vector for use in generateRelay
 *
 * @details Needed to determine the Drone relay path
 *          
 * @param in: x1
 *            First x coordinate
 *
 *            y1
 *            First y coordinate
 *
 *            x2
 *            Second x coordinate
 *
 *            y2
 *            Second y coordinate          
 *        
 * @return float 
 *
 * @note None
 */
float getDirVector( int x1, int y1, int x2, int y2 )
{
    float x = x2 - x1;
    float y = y2 - y1;
    
    if( max(x, y) == 0 )
    {
        return 1;
    }
    else
    {
        return min(x, y) / max(x, y);
    }
}

/**
 * @brief Determines which in flight Drone is closest to the Disaster
 *
 * @details Loops through all Drones and finds the minimum distance to Disaster
 *          
 * @param in: fleet
 *            Drone array of all Drones
 *
 *            dis
 *            Disaster to find closest Drone to      
 *        
 * @return int index of the closest Drone to Disaster
 *
 * @note None
 */
int findClosest( const Drone* fleet, const Disaster& dis )
{
    int index = 0;
    int closest = -1;
    int dist = 1000;
    int thisDist;
    
    for( index = 0; index < FLEET_SIZE; index++ )
    {
        if( fleet[index].inFlight )
        {
            thisDist = distance( fleet[index].xPos, fleet[index].yPos,
                                        dis.xPos, dis.yPos );
            if( thisDist < dist )
            {
                dist = thisDist;
                closest = index;
            }
        }
    }
    
    return closest;
}

/**
 * @brief Simulator Menu
 *
 * @details User has 5 choices: Generate a new Disaster, print relay network
 *          information, simulate x minutes of time passing, send a packet,
 *          or quit
 *          
 * @param in: None        
 *        
 * @return int User's choice
 *
 * @note None
 */
int getUserChoice()
{
    int choice = -1;
    
    while( choice < 0 || choice > 5 )
    {
        cout << endl << "Enter 1 to generate new disaster" << endl
             << "Enter 2 to print relay network information" << endl
             << "Enter 3 to simulate x minutes of time passing" << endl
             << "Enter 4 to simulate sending a packet from (x,y)" << endl
             << "Enter 5 to print relay map (x,y)" << endl
             << "Enter 0 to quit" << endl;
        
        cin >> choice;
    }
    
    return choice;
}
   
/**
 * @brief Simulates x minutes based on user input
 *
 * @details Calls updateBatteries for each simulated minute
 *          
 * @param in: fleet
 *            Drone array of all Drones        
 *        
 * @return Void 
 *
 * @note None
 */  
void simulateTime( Drone*& fleet )
{
    int currentTime, endTime;
    
    cout << "Enter number of minutes to simulate: ";
    cin >> endTime;
    cout << endl;
    
    for( currentTime = 0; currentTime < endTime; currentTime++ )
    {
        updateBatteries( fleet, currentTime );
    }
}

/**
 * @brief All inFlight drones have their battery drained 1% for each minute in
 *        flight. Charging drones gain 5% battery each minute
 *
 * @details Loops through for every Drone and either decreased or increases
 *          battery. Also calls replaceDrone if neccessary
 *          
 * @param in: fleet
 *            Drone array of all Drones
 *
 *            currentTime
 *            Needed for replaceDrone to have correct time reference      
 *        
 * @return void
 *
 * @note None
 */
void updateBatteries( Drone*& fleet, const int currentTime )
{
    int index;
    
    for( index = 0; index < FLEET_SIZE; index++ )
    {
        if( fleet[index].inFlight ) //-1% for each minute in flight
        {
            fleet[index].battery--;
            if( fleet[index].battery < 25 )
            {
                if( !replaceDrone( index, fleet, currentTime ) )
                {
                    cout << "No drones available to replace drone #" << fleet[index].droneID;
                }
            }
        }
        else //+5% for each minute charging
        {
            fleet[index].battery = min( 100, fleet[index].battery + 5 );
        }
    }

}

/**
 * @brief If neccessary and possible replaces a low battery Drone with a
 *        charged drone from the Command and Control Center
 *
 * @details If a Drone needs to be replaced the new Drone takes its place
 *          in the relay. Also the new Drone is given default values
 *          
 * @param in: index
 *            Index of Drone to check for replacement
 *
 *            fleet
 *            Drone Array of all Drones
 *
 *            currentTime
 *            Needed for reference when printing Drone replacement information   
 *        
 * @return bool Evidence of successful Drone replacement
 *
 * @note None
 */
bool replaceDrone( const int index, Drone*& fleet, const int currentTime )
{
    int newIndex = highestPower( fleet );
    Drone* tmp = (&fleet[index]);
    
    if( newIndex < 0 )
    {
        return false;
    }
    
    fleet[newIndex].previous = fleet[index].previous;
    fleet[newIndex].next = fleet[index].next;
    
    if( tmp->previous != NULL )
    {
        tmp = tmp->previous;
        if( tmp->next == &(fleet[index]) )
        {
            tmp->next = &(fleet[newIndex]);
        }
        
        tmp = &(fleet[index]);
    }
    
    if( tmp->next != NULL )
    {
        tmp = tmp->next;
        if( tmp->previous == &(fleet[index]) )
        {
            tmp->previous = &(fleet[newIndex]);
        }
        
        tmp = &(fleet[index]);
    }
    
    fleet[newIndex].inFlight = true;
    fleet[newIndex].xPos = fleet[index].xPos;
    fleet[newIndex].yPos = fleet[index].yPos;
    fleet[newIndex].disNum = fleet[index].disNum;
    fleet[newIndex].firstDrone = fleet[index].firstDrone;
    fleet[index].inFlight = false;
    fleet[index].xPos = 0;
    fleet[index].yPos = 0;
    fleet[index].disNum = -1;
    fleet[index].firstDrone = false;
    fleet[index].previous = NULL;
    fleet[index].next = NULL;
    
    cout << "Drone #" << fleet[index].droneID << " replaced by drone #"
         << fleet[newIndex].droneID << " at " << fleet[newIndex].xPos
         << ", " << fleet[newIndex].yPos;
         
    if( currentTime >= 0 )
    {      
        cout << " at time " << currentTime;
    }
    
    cout << endl;
         
    return true;
    
}

/**
 * @brief Simulates sending a packet across a relay from Disaster to Command
 *        and Control Center
 *
 * @details Gets user input for where packet needs to be sent from. If possible
 *          sends packet across Drones in relay to Command and Control Center.
 *          Replaces Drones as neccessary as sending packets drains battery
 *          
 * @param in: fleet
 *            Drone array of all Drones        
 *        
 * @return bool Evidence of successful packet send
 *
 * @note None
 */
bool sendPacket( Drone*& fleet )
{
    int xPos, yPos;
    int closestDrone;
    Disaster tmp; //used to hold position as args to findClosest
    int distToNet;
    Drone* tmpDrone;
    
    cout << "Enter x position to send packet from (1-100): ";
    cin >> tmp.xPos;
    while( tmp.xPos > 100 || tmp.xPos < 1 )
    {
        cout << "Must be between 1 and 100: ";
        cin >> tmp.xPos;
    }
    cout << "Enter y position to send packet from (1-100): ";
    cin >> tmp.yPos;
    while( tmp.yPos > 100 || tmp.yPos < 1 )
    {
        cout << "Must be between 1 and 100: ";
        cin >> tmp.yPos;
    }
    
    closestDrone = findClosest( fleet, tmp );
    
    distToNet = distance( fleet[closestDrone].xPos, fleet[closestDrone].yPos,
                    tmp.xPos, tmp.yPos );
                    
    if( distToNet > MAX_CONNECTION_DIST * .95 )
    {
        return false;
    }
    
    else
    {
        tmpDrone = &(fleet[closestDrone]);
        cout << endl;
        
        while( tmpDrone != NULL )
        {
            cout << "Packet at drone #" << tmpDrone->droneID << endl;
            tmpDrone->battery -= 1; //subtract 1% for sending a packet
            if( tmpDrone->battery < 25 )
            {
                replaceDrone( tmpDrone->droneID, fleet, -1 );
            }
            tmpDrone = tmpDrone->previous;
        }
        
        cout << "Packet received at Command and Control Center" << endl;
        return true;
    }
    
}

/**
 * @brief Displays the current state of the simulator
 *
 * @details Uses the terminal to display all data
 *          
 * @param in: fleet
 *            Drone array of all Drones
 *
 *            disasterSet
 *            Vector containing all disaster data        
 *        
 * @return void
 *
 * @note None
 */
void printMap(Drone* fleet, vector<Disaster> disasterSet)
{
    int index;
    int passIndex;
    int fleetIndex;
    int disIndex;
    
    cout << endl;

    cout << "UAV:" << endl;
    cout << "#<droneID> :<disaster-attached>" << endl;
    cout << endl;
    cout << "Disaster:";
    cout << "<disasterID> pos: (x,y)" << endl;
    cout << endl;
       
    for( index = 0; index < 101; index++ )
    {
        for( passIndex = 0; passIndex < 101; passIndex++ )
        {
            for( fleetIndex = 0; fleetIndex < FLEET_SIZE; fleetIndex++ )  
            {
                if(fleet[fleetIndex].xPos == index && fleet[fleetIndex].yPos == passIndex )
                {
                    if( fleet[fleetIndex].disNum != -1 )
                    {
                        cout << "#" << fleet[fleetIndex].droneID << " :" << fleet[fleetIndex].disNum;
                    }
                }
            }

            for( disIndex = 0; disIndex < disasterSet.size(); disIndex++ )
            {
                if( disasterSet[disIndex].xPos == index && disasterSet[disIndex].yPos == passIndex )
                {
                    cout << "DSTR:" << disasterSet[disIndex].disNum<< " pos:(" << disasterSet[disIndex].xPos << "," 
                         << disasterSet[disIndex].yPos << ")";
                }
            }

            if( index == 0 && passIndex == 0)
            {
                cout << "(0,0) " << "Base Station";
            }

            if( index == 100 && passIndex == 100 )
            {
                cout << "(100,100) " << "Max";
            }
            cout << " ";
        }
        cout << endl;
    }  
}

#endif
