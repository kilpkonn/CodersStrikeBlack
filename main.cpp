#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#define CHECKPOINT_RADIUS 600
#define MAX_THRUST 100

using namespace std;

struct Checkpoint {
    int x, y;

    bool operator == (const Checkpoint& c) const {
        return x == c.x && y == c.y;
    }

    bool operator != (const Checkpoint& c) const {
        return x != c.x || y != c.y;
    }
};

class Track {

    void onNewCheckpoint(Checkpoint point) {
        if (allCheckpointsFound) return;

        if (checkpoints.empty()) {
            checkpoints.push_back(point);
        } else if (checkpoints.back() != point) {
            checkpoints.push_back(point);
            if (checkpoints.front() == point) {
                allCheckpointsFound = true;
            }
        }
    }

private:
    vector<Checkpoint> checkpoints;
    bool allCheckpointsFound = false;
};

bool canRam(int x, int y, int checkX, int checkY, int opponentX, int opponentY, int angle) {
    bool opponentInFront = abs(opponentX - checkX) < abs(x - checkX) && abs(opponentY < checkY) < abs(y - checkY);
    cerr << " In front:"<< opponentInFront;
    bool opponentClose = abs(opponentY - y) < 600 && abs(opponentX - x) < 600;
    cerr << " close: "<< opponentClose;
    bool opponentCloseToCheck = abs(opponentY - checkY) < CHECKPOINT_RADIUS && abs(opponentX - checkX) < CHECKPOINT_RADIUS;
    cerr << " close to check: " << opponentCloseToCheck;
    cerr << " angle: " << angle << endl;
    return opponentInFront && opponentClose && opponentCloseToCheck && abs(angle) < 30;
}

/**
 * This code automatically collects game data in an infinite loop.
 * It uses the standard input to place data into the game variables such as x and y.
 * YOU DO NOT NEED TO MODIFY THE INITIALIZATION OF THE GAME VARIABLES.
 **/
int main() {
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

        int thrust = MAX_THRUST;
        if ((nextCheckpointDist < 3000 && abs(nextCheckpointAngle) > 60) || abs(nextCheckpointAngle) > 100) {
            thrust = 0;
        }
        // canRam(x, y, nextCheckpointX, nextCheckpointY, opponentX, opponentY, nextCheckpointAngle);
        cerr << " dist: " << nextCheckpointDist;
        bool ram = canRam(x, y, nextCheckpointX, nextCheckpointY, opponentX, opponentY, nextCheckpointAngle);
        cerr << " can ram: " << canRam;
        if (nextCheckpointDist < CHECKPOINT_RADIUS && !ram) {
            thrust = 5;
        }

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // Edit this line to output the target position
        // and thrust (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        if (nextCheckpointDist > 5000 && nextCheckpointAngle < 10 && nextCheckpointAngle > -10 && !boostUsed) {
            cout << nextCheckpointX << " " << nextCheckpointY << " BOOST" << endl;
            boostUsed = true;
        } else {
            cout << nextCheckpointX << " " << nextCheckpointY << " " << thrust << endl;
        }
        cerr << endl;

    }
}