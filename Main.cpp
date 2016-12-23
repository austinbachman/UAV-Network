// Program Information ////////////////////////////////////////////////////////
/**
 * @file Main.cpp
 *
 * @brief Main driver for CPE400 Project Simulator
 *        Project 2: Using an energy‐constrained relay network cluster to 
 *        transmit disaster area information to Command and Control Center
 * 
 * @details Contains struct definitions, function definitions and main function
 *
 * @version 1.5
 *          Newest Addition: printMap function
 *
 * @Note None
 */

// Header files ///////////////////////////////////////////////////////////////

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include "simulator.cpp"

using namespace std;

// Begin main /////////////////////////////////////////////////////////////////

int main()
{
    Drone* fleet;
    vector<Disaster> disasterSet;
    Disaster dis;
    int disCount = 0;
    int choice = 1;
    bool relayCreated;
    srand(time(NULL));
    
    generateFleet( fleet );
    while( choice != 0 )
    {
        if( choice == 1 )
        {
            makeDisaster( dis, disCount );
            relayCreated = generateRelay( dis, fleet );
            if( relayCreated )
            {
                disCount++;
                disasterSet.push_back(dis);
                cout << endl << "Relay network successfully established" << endl;
            }
            else
            {
                cout << endl 
                     << "Failed to generate relay network: not enough drones available"
                     << endl << endl;
            }
        }
        
        else if( choice == 2 )
        {
            displayRelay( fleet, disCount );
        }
        
        else if( choice == 3 )
        {
            simulateTime( fleet );
        }
        
        else if( choice == 4 )
        {
            if( !sendPacket( fleet ) )
            {
                cout << endl
                     << "Unable to connect to fleet from given coordinates" << endl;
            }
        }

        else if( choice == 5 )
        {
            printMap( fleet, disasterSet );            
        }
        
        choice = getUserChoice();
    }

    delete[] fleet;
    fleet = NULL;
    return 0;
}
