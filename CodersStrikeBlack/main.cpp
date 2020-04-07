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

class PID {
public:
    PID(double dt, double max, double min, double Kp, double Kd, double Ki) :
            _dt(dt),
            _max(max),
            _min(min),
            _Kp(Kp),
            _Kd(Kd),
            _Ki(Ki),
            _pre_error(0),
            _integral(0) {
    }

    double calculate(double target, double currentValue) {
        // Calculate error
        double error = target - currentValue;

        // Proportional term
        double Pout = _Kp * error;

        // Integral term
        _integral += error * _dt;
        double Iout = _Ki * _integral;

        // Derivative term
        double derivative = (error - _pre_error) / _dt;
        double Dout = _Kd * derivative;

        // Calculate total output
        double output = Pout + Iout + Dout;

        // Restrict to max/min
        if (output > _max)
            output = _max;
        else if (output < _min)
            output = _min;

        // Save error to previous error
        _pre_error = error;

        return output;
    }

private:
    double _dt;
    double _max;
    double _min;
    double _Kp;
    double _Kd;
    double _Ki;
    double _pre_error;
    double _integral;
};

class Track {
public:
    bool allCheckpointsFound = false;

    PID pid1 = PID(1, 135, -135, 0.2, 0.001, 0.02);
    PID pid2 = PID(1, 135, -135, 0.2, 0, 0);

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

    double calcTurnAngle(Vector2D &ship, Vector2D &currentCP, Vector2D &nextCp) {
        double pointAngle = angle(ship, currentCP);
        // cerr << "x" << pointAngle << endl;
        double nextAngle = angle(currentCP, nextCp);
        // cerr << nextAngle << endl;
        double angle = nextAngle - pointAngle;
        return normalize_angle(angle);
    }

    double calcOptimalAngleOffset(Vector2D speed, Vector2D &pos, Vector2D &cp, Vector2D &nextCP) {
        double x = length(pos, cp) / 150; // Map 0 - 15000 to 0 - 100
        cerr << "x " << x << endl;
        double turnAngle = calcTurnAngle(pos, cp, nextCP);
        cerr << "a " << turnAngle << endl;
        // log(x + 1) * 80 - sqrt(x) * 16
        // log((x -10)/3 + 4) * 75 - sqrt((x)/2) * 16
        return -(log10((x - 10) / 3 + 4) * 75 - sqrt((x) / 2) * 16) * turnAngle / 100;
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

        cerr << "Distance: " << length(currentCP, ship.pos) << endl;
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

    Vector2D calcOptimalTarget2(Ship2D &ship, bool ship1) {
        Vector2D currentCP = getCp(ship.cpId);
        Vector2D nextCP = getCp((ship.cpId + 1) % checkpoints.size());
        double angleToCP = angle(ship.pos, currentCP); // Reverse?
        double velocityAngle = atan2(ship.velocity.y, ship.velocity.x) * 180 / PI;
        double contVelAngle = normalize_angle(angleToCP - velocityAngle);
        // Fix standstill
        if (ship.velocity.x == 0 && ship.velocity.y == 0) contVelAngle = 0;

        double preferredAngleOffset = calcOptimalAngleOffset(ship.velocity, ship.pos, currentCP, nextCP);

        cerr << "Distance: " << length(currentCP, ship.pos) << endl;
        cerr << "Velocity angle: " << velocityAngle << endl;
        cerr << "Continious vel angle:" << contVelAngle << endl;
        cerr << "CP angle: " << angleToCP << endl;
        cerr << "Pref angle offset: " << preferredAngleOffset << endl;
        double targetAngle = normalize_angle(angleToCP + preferredAngleOffset);
        cerr << "Target angle: " << targetAngle << endl;
        double error = normalize_angle(targetAngle - contVelAngle); // TODO: pid
        cerr << "Error: " << error << endl;
        double target = normalize_angle(targetAngle);
        if (contVelAngle != 0) {
            if (ship1) {
                error = pid1.calculate(0, contVelAngle);
            } else {
                error = pid2.calculate(0, contVelAngle);
            }
            cerr << "PID err: " << error << endl;
            target = normalize_angle(target + error);
        }
        cerr << "PID " << target << endl;
        double radAngle = target / 180 * PI;
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

        cerr << "============== SHIP 1 ==============" << endl;
        thrust = to_string(MAX_THRUST);
        target = track.calcOptimalTarget2(ship1, true);
        cout << target.x << " " << target.y << " " << thrust << endl;


        cerr << "============== SHIP 2 ==============" << endl;
        thrust = to_string(MAX_THRUST);
        target = track.calcOptimalTarget2(ship2, false);
        cout << target.x << " " << target.y << " " << thrust << endl;

    }
}

#pragma clang diagnostic pop