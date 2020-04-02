#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

/**
 * This code automatically collects game data in an infinite loop.
 * It uses the standard input to place data into the game variables such as x and y.
 * YOU DO NOT NEED TO MODIFY THE INITIALIZATION OF THE GAME VARIABLES.
 **/

int main()
{
    bool boostUsed = false;
    // game loop
    while (1) {
        int x; // x position of your pod
        int y; // y position of your pod
        int nextCheckpointX; // x position of the next check point
        int nextCheckpointY;// y position of the next check point
        int nextCheckpointDist;
        int nextCheckpointAngle;
        
        int opponentX;
        int opponentY;
        
        cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle;
        cin >> opponentX >> opponentY;
        
        int thrust = 100;
        if (nextCheckpointAngle > 90 || nextCheckpointAngle < -90) {
            thrust = 0;
        }
        if (nextCheckpointDist < 500) {
            thrust = 0;      
        }

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // Edit this line to output the target position
        // and thrust (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        if (nextCheckpointDist > 3000 && nextCheckpointAngle < 10 && nextCheckpointAngle > -10 && !boostUsed) {
            cout << nextCheckpointX << " " << nextCheckpointY << " BOOST" << endl;
            boostUsed = true;
        } else {
            cout << nextCheckpointX << " " << nextCheckpointY << " " << thrust << endl;
        }


    }
}