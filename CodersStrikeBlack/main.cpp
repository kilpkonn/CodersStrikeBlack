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

struct Vector2D {
    int x, y;

    Vector2D() = default;

    Vector2D(int x, int y) : x(x), y(y) {}

    bool operator==(const Vector2D &p) const {
        return x == p.x && y == p.y;
    }

    bool operator!=(const Vector2D &p) const {
        return x != p.x || y != p.y;
    }
};

inline double angle(const Vector2D &a, const Vector2D &b) {
    return atan2(b.y - a.y, b.x - a.x) * 180 / PI;
}

inline double length(const Vector2D &a, const Vector2D &b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

inline double normalize_angle(double angle) {
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    return angle;
}

struct Ship2D {
    int angle, cpId;
    Vector2D pos, velocity;

    Ship2D() = default;

    Ship2D(int x, int y, int vx, int vy, int angle, int cpId) : angle(angle), cpId(cpId) {
        pos = Vector2D(x, y);
        velocity = Vector2D(vx, vy);
    }
};

class Track {
public:
    bool allCheckpointsFound = false;

    void addNewCheckpoint(Vector2D point) {
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
     * @return - means left, + means right
     */
    double angleToNextCp(Vector2D &ship, Vector2D &currentCP, Vector2D &nextCp) {
        double pointsAngle = angle(currentCP, nextCp);
        double shipAngle = angle(ship, currentCP);
        double angle = pointsAngle - shipAngle;
        return normalize_angle(angle);
    }

    Vector2D getCp(int index) {
        return checkpoints[index];
    }

    double calcOptimalAngle(int distance, double nextAngle) {
        // x*x*x/8000 - x*x/60 + x/6+ 30
        if (abs(nextAngle) > 160) nextAngle = nextAngle > 0 ? 160 : -160;
        distance = distance / 100 - 70;
        if (distance > 75) distance = 75;
        cerr << "Optimal angle x: " << distance << endl;
        return -nextAngle *
               (distance * distance * distance / 6000.0 - distance * distance / 60.0 + distance / 6.0 + 30) / 170;
    }

    double calcOptimalAngle2(Vector2D speed, Vector2D &pos, Vector2D &cp, Vector2D &nextCP) {
        return 0;
    }

#pragma clang diagnostic push
#pragma ide diagnostic ignored "bugprone-narrowing-conversions"

    Vector2D calcOptimalTarget(Ship2D &ship) {
        Vector2D currentCP = getCp(ship.cpId);
        Vector2D nextCP = getCp((ship.cpId + 1) % checkpoints.size());
        double angleToCP = angle(ship.pos, currentCP); // Reverse?
        double velocityAngle = atan2(ship.velocity.y, ship.velocity.x) * 180 / PI;
        double contVelAngle = normalize_angle(angleToCP - velocityAngle);
        double contVelAngleWeight = 1 - length(ship.pos, currentCP) / 2500;
        if (contVelAngleWeight < 0) contVelAngleWeight = 0;

        // Fix standstill
        if (ship.velocity.x == 0 && ship.velocity.y == 0) contVelAngle = 0;

        cerr << "Distance: " <<  length(currentCP, ship.pos) << endl;
        cerr << "Velocity angle: " << velocityAngle << endl;
        cerr << "Continious vel angle:" << contVelAngle << endl;
        double optimalAngle = calcOptimalAngle(length(ship.pos, currentCP), angleToNextCp(ship.pos, currentCP, nextCP));
        cerr << "CP angle: " << angleToCP << endl;
        cerr << "Optimal angle: " << optimalAngle << endl;
        if (contVelAngleWeight > 0) optimalAngle *= (1 - contVelAngleWeight);
        double radAngle = (optimalAngle + angleToCP + contVelAngle * contVelAngleWeight) / 180 * PI;
        return Vector2D(ship.pos.x + TARGET_AHEAD_DISTANCE * cos(radAngle),
                       ship.pos.y + TARGET_AHEAD_DISTANCE * sin(radAngle));
    }

#pragma clang diagnostic pop

private:
    vector<Vector2D> checkpoints;
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
    Track track;

    int lapsCount;
    int checkpointsCount;
    cin >> lapsCount >> checkpointsCount;
    int cX, cY;
    for (int i = 0; i < checkpointsCount; i++) {
        cin >> cX >> cY;
        track.addNewCheckpoint(Vector2D(cX, cY));
    }

    Ship2D ship1, ship2, opponent1, opponent2;

    Vector2D target;
    Vector2D cp;
    string thrust;

    int x, y, vx, vy, angle, cpId;

    // game loop
    while (1) {
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ship1 = Ship2D(x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ship2 = Ship2D(x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        opponent1 = Ship2D(x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        opponent2 = Ship2D(x, y, vx, vy, angle, cpId);

        thrust = to_string(MAX_THRUST);

        target = track.calcOptimalTarget(ship1);
        cout << target.x << " " << target.y << " " << thrust << endl;


        thrust = to_string(MAX_THRUST);
        /*if (abs(ship2.angle) > 80) {
            thrust = "10";
        }*/
        target = track.calcOptimalTarget(ship2);
        cout << target.x << " " << target.y << " " << thrust << endl;

    }
}

#pragma clang diagnostic pop