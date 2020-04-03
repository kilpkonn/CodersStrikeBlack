#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#define CHECKPOINT_RADIUS 600
#define MAX_THRUST 100
#define MAX_ANGLE_ALLOWED_BOOST 10
#define MIN_DISTANCE_ALLOWED_BOOST 6000
#define SHIP_RADIUS 400
#define CLOSE_PADDING 100
#define PI 3.14159265
#define TARGET_AHEAD_DISTANCE 5000

using namespace std;

struct Point2D {
    int x, y;

    Point2D(int x_, int y_) {
        x = x_;
        y = y_;
    }

    bool operator==(const Point2D &p) const {
        return x == p.x && y == p.y;
    }

    bool operator!=(const Point2D &p) const {
        return x != p.x || y != p.y;
    }
};

class Track {
public:
    bool allCheckpointsFound = false;

    void onNewCheckpoint(Point2D point) {
        /*for (int i = 0; i < checkpoints.size(); i++) {
            cerr << "Cp: " << i << " " << checkpoints[i].x << " - " << checkpoints[i].y << endl;
        }*/
        if (allCheckpointsFound) return;

        if (checkpoints.empty()) {
            checkpoints.push_back(point);
        } else if (checkpoints.back() != point) {
            checkpoints.push_back(point);
            if (checkpoints.front() == point) {
                allCheckpointsFound = true;
            }
            cerr << "New point: " << checkpoints.back().x << " - " << checkpoints.back().y;
        }
    }

    /**
     * @param x ship x
     * @param y ship y
     * @param currentCP
     * @return - means left, + means right
     */
    double angleToNextCp(Point2D &ship, Point2D &currentCP) {
        int index = getCPIndex(currentCP);
        if (index < 0 || !allCheckpointsFound)
            return 0;
        Point2D *nextCpPtr = &checkpoints[(index + 1) % checkpoints.size()];

        cerr << "Next CP: " << index << " (" << nextCpPtr->x << ", " << nextCpPtr->y << ")" << endl;

        int dx = nextCpPtr->x - currentCP.x;
        int dy = nextCpPtr->y - currentCP.y;
        double pointsAngle = atan2(dy, dx) * 180 / PI;
        double shipAngle = atan2(currentCP.y - ship.y, currentCP.x - ship.x) * 180 / PI;
        // cerr << " angle p: " << pointsAngle << endl;
        // cerr << " angle ship: " << shipAngle << endl;
        double angle = pointsAngle - shipAngle;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 260;
        return angle;
    }

    int getCPIndex(Point2D &cp) {
        if (checkpoints.empty()) return -1;
        auto itr = find_if(checkpoints.cbegin(), checkpoints.cend(), [&cp](Point2D const &ca) { return ca == cp; });
        if (itr != checkpoints.cend())
            return std::distance(checkpoints.cbegin(), itr);
        return -1;
    }

    Point2D getNextCp(Point2D &cp) {
        int idx = getCPIndex(cp);
        if (idx < 0) return cp;
        return checkpoints[(idx + 1) % checkpoints.size()];
    }

    double calcOptimalAngle(int distance, double nextAngle) {
        // x*x*x/8000 - x*x/60 + x/6+ 30
        if (abs(nextAngle) > 160) nextAngle = nextAngle > 0 ? 160 : -160;
        distance = distance / 100 - 70;
        if (distance > 75) distance = 75;
        cerr << "Optimal angle x: " << distance << endl;
        return -nextAngle * (distance * distance * distance / 6000.0 - distance * distance / 60.0 + distance / 6.0 + 30) / 170;
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-narrowing-conversions"

    Point2D calcOptimalTarget(Point2D &pos, Point2D &cp, double optimalAngle) {
        double angle = atan2(pos.y - cp.y, pos.x - cp.x) * 180 / PI;
        cerr << "CP angle: " << angle << endl;
        cerr << "Optimal angle: " << optimalAngle << endl;
        double radAngle = (optimalAngle + angle) / 180 * PI;
        cerr << "DX: " << TARGET_AHEAD_DISTANCE * cos(radAngle) << endl;
        cerr << "DY: " << TARGET_AHEAD_DISTANCE * sin(radAngle) << endl;
        return Point2D(pos.x - TARGET_AHEAD_DISTANCE * cos(radAngle), pos.y - TARGET_AHEAD_DISTANCE * sin(radAngle));
    }

#pragma clang diagnostic pop

private:
    vector<Point2D> checkpoints;
};

bool canRam(int x, int y, int checkX, int checkY, int opponentX, int opponentY, int angle) {
    bool opponentInFront = abs(opponentX - checkX) < abs(x - checkX) && abs(opponentY - checkY) < abs(y - checkY);
    cerr << " In front:" << opponentInFront;
    bool opponentClose = abs(opponentY - y) < 600 && abs(opponentX - x) < 600;
    cerr << " close: " << opponentClose;
    bool opponentCloseToCheck =
            abs(opponentY - checkY) < CHECKPOINT_RADIUS && abs(opponentX - checkX) < CHECKPOINT_RADIUS;
    cerr << " close to check: " << opponentCloseToCheck;
    cerr << " angle: " << angle << endl;
    return opponentInFront && opponentClose && opponentCloseToCheck && abs(angle) < 30;
}

bool opponentClose(int x, int y, int opponentX, int opponentY) {
    return sqrt((x - opponentX) * (x - opponentX) + (y - opponentY) * (y - opponentY)) <
           SHIP_RADIUS * 2 + CLOSE_PADDING;
}


/**
 * This code automatically collects game data in an infinite loop.
 * It uses the standard input to place data into the game variables such as x and y.
 * YOU DO NOT NEED TO MODIFY THE INITIALIZATION OF THE GAME VARIABLES.
 **/
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
    bool boostUsed = false;
    Track track;
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

        Point2D currentCP = Point2D(nextCheckpointX, nextCheckpointY);
        Point2D shipLoc = Point2D(x, y);
        Point2D target = Point2D(nextCheckpointX, nextCheckpointY);

        track.onNewCheckpoint(currentCP);
        double nextAngle = track.angleToNextCp(shipLoc, currentCP);

        string thrust = to_string(MAX_THRUST);

        cerr << "Dist: " << nextCheckpointDist << " next angle: " << nextAngle;
        bool ram = canRam(x, y, nextCheckpointX, nextCheckpointY, opponentX, opponentY, nextCheckpointAngle);
        //cerr << " can ram: " << &canRam;

        /*if (nextCheckpointDist < 2000) {
            if (nextAngle * nextCheckpointAngle > 0 || (nextCheckpointDist < 1000 && abs(nextCheckpointAngle) < 30)) {
                // Both on same side
                target = track.getNextCp(currentCP);
            } else if (abs(nextCheckpointAngle) > 45) {
                thrust = "5";
            }

            if (abs(nextAngle) > 130 && nextCheckpointDist < 1000) {
                thrust = "5";
            }
        } else {
            if (nextCheckpointDist > 3000) {
                target = Point2D()
            }
        }*/


        if (track.allCheckpointsFound) {
            double angle = track.calcOptimalAngle(nextCheckpointDist, nextAngle);
            cerr << "Smart angle: " << angle << endl;
            //if (abs(angle) < 160) {
                target = track.calcOptimalTarget(shipLoc, currentCP, angle);
            //}

            if (nextCheckpointAngle > 65 ) {
                cerr << "A" << endl;
                thrust = "10";
            }
            if (abs(nextAngle) > 150 && nextCheckpointDist < 1200) {
                cerr << "B" << endl;
                thrust = "10";
            }

            // Hax
            if (abs(currentCP.x - 10565) < 150 &&  abs(currentCP.y - 5973) < 150) {
                cerr << "XD" << endl;
                target = Point2D(11350, 6000);
            } else if (abs(currentCP.x - 13116) < 150 &&  abs(currentCP.y - 2317) < 150) {
                target = Point2D(12700, 2850);
            } else if (abs(currentCP.x - 7347) < 150 &&  abs(currentCP.y - 4943) < 150) {
                target = Point2D(6600, 3700);
            } else if (abs(currentCP.x - 12920) < 150 &&  abs(currentCP.y -7241) < 150) {
                target = Point2D(12500, 6700);
            } else if (abs(currentCP.x - 6300) < 150 &&  abs(currentCP.y -7700) < 150) {
                target = Point2D(6300, 6700);
            } else if (abs(currentCP.x - 5950) < 150 &&  abs(currentCP.y -4222) < 150) {
                target = Point2D(4700, 4200);
            } else if (abs(currentCP.x - 8700) < 150 &&  abs(currentCP.y -7440) < 150) {
                target = Point2D(9100, 7000);
            }

        } else {
            cerr << "CP real angle: " << nextCheckpointAngle << endl;
            /*if (nextCheckpointAngle < 60) {
                target = track.calcOptimalTarget(shipLoc, currentCP, nextCheckpointAngle / 1.5);
            }*/

            if (nextCheckpointDist < CHECKPOINT_RADIUS * 1.2) {
                thrust = "5";
            }
            if (abs(nextCheckpointAngle) > 75) {
                thrust = "5";
            }

            // Hax
            if (abs(currentCP.x - 11205) < 150 &&  abs(currentCP.y - 5421) < 150) {
                cerr << "XD" << endl;
                target = Point2D(10200, 5200);
            } else if (abs(currentCP.x - 6000) < 150 &&  abs(currentCP.y - 5350) < 150) {
                cerr << "XD" << endl;
                target = Point2D(6600, 5300);
            }
        }

        /*if (nextCheckpointDist < CHECKPOINT_RADIUS && !ram) {
            if (opponentClose(x, y, opponentX, opponentY)) {
                thrust = "SHIELD";
            } else if (!track.allCheckpointsFound) {
                thrust = "5";
            }
        }*/

        if (track.allCheckpointsFound && nextCheckpointDist > MIN_DISTANCE_ALLOWED_BOOST
        && abs(nextCheckpointAngle) < MAX_ANGLE_ALLOWED_BOOST
        && abs(nextAngle) < MAX_ANGLE_ALLOWED_BOOST
        && !boostUsed) {
            cout << target.x << " " << target.y << " BOOST" << endl;
            boostUsed = true;
        } else {
            cout << target.x << " " << target.y << " " << thrust << endl;
        }
        cerr << "Target: " << target.x << " - " << target.y << endl;
        cerr << endl;
    }
}

#pragma clang diagnostic pop