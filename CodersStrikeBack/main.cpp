#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>

#define CHECKPOINT_RADIUS 600
#define MAX_THRUST 100
#define BOOST_THRUST 650
#define SHIELD_COOL_DOWN 3
#define MAX_ANGLE_ALLOWED_BOOST 10
#define MIN_DISTANCE_ALLOWED_BOOST 6000
#define POD_RADIUS 400
#define PI 3.14159265
#define TARGET_AHEAD_DISTANCE 5000
#define DRAG 0.85
#define MAX_CP_DISTANCE 16000

#define SIMULATION_CHILD_COUNT 6
#define SIMULATION_DEPTH 6
#define SIMULATION_ANGLE_DIFF 60

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

inline double angle(const Vector2D &a) {
    return atan2(a.y, a.x) * 180 / PI;
}

inline double length(const Vector2D &a, const Vector2D &b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

inline double length(const Vector2D &a) {
    return sqrt(a.x * a.x + a.y * a.y);
}

inline double normalize_angle(double angle) {
    if (angle > 180) angle -= 360;
    if (angle < -180) angle += 360;
    return angle;
}

struct Ship2D {
    int angle, cpId;
    Vector2D pos, velocity;
    int thrust = MAX_THRUST;
    bool boostUsed = false;
    int shieldCoolDown = 0;
    Vector2D target = Vector2D(0, 0);

    Ship2D() : pos(Vector2D(0, 0)), velocity(Vector2D(0, 0)), angle(0), cpId(0) {};

    Ship2D(const Vector2D &pos,
           const Vector2D &velocity,
           int angle,
           int cpId,
           int thrust,
           bool boostUsed,
           int shieldCoolDown,
           const Vector2D &target) :
            angle(angle),
            cpId(cpId),
            pos(pos),
            velocity(velocity),
            thrust(thrust),
            boostUsed(boostUsed),
            shieldCoolDown(shieldCoolDown),
            target(target) {}
};

class Track {
public:
    Ship2D pod1 = Ship2D();
    Ship2D pod2 = Ship2D();
    Ship2D opponent1 = Ship2D();
    Ship2D opponent2 = Ship2D();
    int lapsCount = 3;

    void updatePod(Ship2D *pod, int x, int y, int vx, int vy, int angle, int cpId) {
        pod->pos = Vector2D(x, y);
        pod->velocity = Vector2D(vx, vy);
        pod->angle = angle;
        pod->cpId = cpId;
    }

    void addNewCheckpoint(Vector2D point) {
        /*for (int i = 0; i < checkpoints.size(); i++) {
            cerr << "Cp: " << i << " " << checkpoints[i].x << " - " << checkpoints[i].y << endl;
        }*/
        checkpoints.push_back(point);
        cerr << "New point: " << checkpoints.back().x << " - " << checkpoints.back().y;
    }

    Vector2D getCp(unsigned index) {
        return checkpoints[index % checkpoints.size()];
    }

    double calcTurnAngle(Vector2D &ship, Vector2D &currentCP, Vector2D &nextCp) {
        double pointAngle = angle(ship, currentCP);
        // cerr << "x" << pointAngle << endl;
        double nextAngle = angle(currentCP, nextCp);
        // cerr << nextAngle << endl;
        double angle = nextAngle - pointAngle;
        return normalize_angle(angle);
    }

    Vector2D calcOptimalCpPos(Ship2D *ship, Vector2D &cp, Vector2D &nextCp) {
        Vector2D medianPoint = Vector2D((ship->pos.x + nextCp.x), (ship->pos.y + nextCp.y) / 2);
        double radAngleFromCp = angle(cp, medianPoint) / 180 * PI;
        return {(int) (cp.x + CHECKPOINT_RADIUS * 0.5 * cos(radAngleFromCp)),
                (int) (cp.y + CHECKPOINT_RADIUS * 0.5 * radAngleFromCp)};
    }

    double calcOptimalAngleOffset(Vector2D speed, Vector2D &pos, Vector2D &cp, Vector2D &nextCP) {
        double x = length(pos, cp) / 150; // Map 0 - 15000 to 0 - 100
        // cerr << "x " << x << endl;
        double turnAngle = calcTurnAngle(pos, cp, nextCP);
        // cerr << "a " << turnAngle << endl;
        // log(x + 1) * 80 - sqrt(x) * 16
        // log((x -10)/3 + 4) * 75 - sqrt((x)/2) * 16
        // return -(log10((x - 10) / 3 + 4) * 70 - sqrt((x + 2) / 2) * 16 + x / 10) * turnAngle / 50;

        // 20 * cos(log(x + 36) * 8) - x/18
        // return 20 * cos(log10(x + 36) * 8) - x / 18 * turnAngle / 100;

        // (x-50)*(x-50)*(x-50)/6000 - (x-50)*(x-50)/60 + (x-50)/6+ 30
        return -((x - 60) * (x - 60) * (x - 60) / 6000 - (x - 60) * (x - 60) / 60 + (x - 60) / 6 + 30) * turnAngle /
               180;
    }

    void plan() {
        // TODO: Simulation for some AI
        simulate();
        cerr << "============== SHIP 1 ==============" << endl;
        pod1.shieldCoolDown--;
        calcOptimalTarget(&pod1);
        evaluateShield(&pod1);
        evaluateBoost(&pod1);
        cerr << "============== SHIP 2 ==============" << endl;
        pod2.shieldCoolDown--;
        calcOptimalTarget(&pod2);
        evaluateShield(&pod2);
        evaluateBoost(&pod2);
    }

    double calcOptimalTargetAngle(Ship2D *ship) {
        Vector2D nextCP = getCp(ship->cpId + 1);
        Vector2D tmpCP = getCp(ship->cpId);
        Vector2D currentCP = calcOptimalCpPos(ship, tmpCP, nextCP);
        double angleToCP = angle(ship->pos, currentCP);
        double velocityAngle = atan2(ship->velocity.y, ship->velocity.x) * 180 / PI;
        double contVelAngle = normalize_angle(angleToCP - velocityAngle);
        // Fix standstill
        if (ship->velocity.x == 0 && ship->velocity.y == 0) contVelAngle = 0;

        double preferredAngleOffset = calcOptimalAngleOffset(ship->velocity, ship->pos, currentCP, nextCP);
        double targetAngle = normalize_angle(angleToCP + preferredAngleOffset);
        double error = normalize_angle(targetAngle - velocityAngle);
        double target = normalize_angle(targetAngle + error * 0.2);

        if (contVelAngle == 0 || abs(error) > 25) {
            target = targetAngle;
        }

        Vector2D newPos = Vector2D(ship->pos.x + ship->velocity.x, ship->pos.y + ship->velocity.y);

        double newAngleToCP = angle(newPos, currentCP);
        double newPreferredOffset = calcOptimalAngleOffset(ship->velocity, newPos, currentCP, nextCP);
        double newTargetAngle = normalize_angle(newAngleToCP + newPreferredOffset);

        return (target + newTargetAngle) / 2;
    }

    void calcOptimalTarget(Ship2D *ship) {
        Vector2D nextCP = getCp(ship->cpId + 1);
        Vector2D tmpCP = getCp(ship->cpId);
        Vector2D currentCP = calcOptimalCpPos(ship, tmpCP, nextCP);
        double angleToCP = angle(ship->pos, currentCP);
        double velocityAngle = atan2(ship->velocity.y, ship->velocity.x) * 180 / PI;
        double contVelAngle = normalize_angle(angleToCP - velocityAngle);
        // Fix standstill
        if (ship->velocity.x == 0 && ship->velocity.y == 0) contVelAngle = 0;

        double preferredAngleOffset = calcOptimalAngleOffset(ship->velocity, ship->pos, currentCP, nextCP);

        cerr << "Distance: " << length(currentCP, ship->pos) << endl;
        cerr << "Velocity angle: " << velocityAngle << endl;
        cerr << "Continuous vel angle:" << contVelAngle << endl;
        cerr << "CP angle: " << angleToCP << endl;
        cerr << "Pref angle offset: " << preferredAngleOffset << endl;
        double targetAngle = normalize_angle(angleToCP + preferredAngleOffset);
        cerr << "Target angle: " << targetAngle << endl;
        double error = normalize_angle(targetAngle - velocityAngle); // TODO: pid
        cerr << "Error: " << error << endl;
        double target = normalize_angle(targetAngle + error * 0.2);
        cerr << "Corrected target:  " << target << endl;

        if (contVelAngle == 0 || abs(error) > 25) {
            target = targetAngle;
        }

        Vector2D newPos = Vector2D(ship->pos.x + ship->velocity.x,
                                   ship->pos.y + ship->velocity.y); // Work on this

        double newAngleToCP = angle(newPos, currentCP);
        double newPreferredOffset = calcOptimalAngleOffset(ship->velocity, newPos, currentCP, nextCP);
        double newTargetAngle = normalize_angle(newAngleToCP + newPreferredOffset);

        cerr << "New target angle: " << newTargetAngle << endl;
        cerr << "Turn angle: " << newTargetAngle - velocityAngle << endl;

        if (contVelAngle != 0
            && abs(normalize_angle((newTargetAngle + target) / 2 - velocityAngle)) > 90
            && abs(normalize_angle(newTargetAngle + target) / 2 - velocityAngle) < 135 // Reverse lol
            && length(ship->velocity) > 150
            && length(ship->pos, currentCP) < 2500) { // speed

            ship->thrust = 0;
        } else {
            ship->thrust = MAX_THRUST;
        }
        double combinedTargetAngle = (target + newTargetAngle) / 2;
        cerr << "Combined target angle: " << combinedTargetAngle << endl;

        double radAngle = combinedTargetAngle / 180 * PI;
        ship->target = {(int) (ship->pos.x + TARGET_AHEAD_DISTANCE * cos(radAngle)),
                        (int) (ship->pos.y + TARGET_AHEAD_DISTANCE * sin(radAngle))};
    }

    void evaluateShield(Ship2D *ship) {
        Vector2D newPos = Vector2D(ship->pos.x + ship->velocity.x,
                                   ship->pos.y + ship->velocity.y);
        Vector2D newOpponet1 = Vector2D(opponent1.pos.x + opponent1.velocity.x,
                                        opponent1.pos.y + opponent1.velocity.y);
        Vector2D newOpponet2 = Vector2D(opponent2.pos.x + opponent2.velocity.x,
                                        opponent2.pos.y + opponent2.velocity.y);

        if (length(newPos, newOpponet1) < POD_RADIUS * 2.2) {
            if (abs(angle(ship->velocity) - angle(newOpponet1)) > 45) {
                ship->shieldCoolDown = SHIELD_COOL_DOWN;
            }
        }

        if (length(newPos, newOpponet2) < POD_RADIUS * 2.2) {
            if (abs(angle(ship->velocity) - angle(newOpponet2)) > 45) {
                ship->shieldCoolDown = SHIELD_COOL_DOWN;
            }
        }
    }

    void evaluateBoost(Ship2D *ship) {
        if (ship->boostUsed) return;

        if (length(ship->pos, getCp(ship->cpId)) > MIN_DISTANCE_ALLOWED_BOOST
            && abs(angle(ship->pos, getCp(ship->cpId))) < MAX_ANGLE_ALLOWED_BOOST) {
            ship->thrust = BOOST_THRUST;
            ship->boostUsed = true;
        }
    }

    void simulate() {
        return;
    }


private:
    vector<Vector2D> checkpoints;
};

class SimulationNode {
public:
    Track *track;
    Ship2D pod1, pod2, opponent1, opponent2;
    double score = 0;

    SimulationNode() {}

    SimulationNode(Track *track,
                   const Ship2D &pod1, const Ship2D &pod2, const Ship2D &opponent1, const Ship2D &opponent2) :
            track(track), pod1(pod1), pod2(pod2), opponent1(opponent1), opponent2(opponent2) {}

    SimulationNode evaluate(int depth = SIMULATION_DEPTH) {
        if (depth <= 0) {
            score = getNodeScore(this);
            return *this;
        }

        double pod1BaseAngle = track->calcOptimalTargetAngle(&pod1);
        double pod2BaseAngle = track->calcOptimalTargetAngle(&pod2);

        double pod1Angle;
        double pod2Angle;

        SimulationNode node, best;
        double bestScore = -1; // Negative start?

        for (short d1 = -SIMULATION_ANGLE_DIFF / 2;
             d1 < SIMULATION_ANGLE_DIFF / 2; d1 += SIMULATION_ANGLE_DIFF / SIMULATION_CHILD_COUNT) {
            for (short d2 = -SIMULATION_ANGLE_DIFF / 2;
                 d2 < SIMULATION_ANGLE_DIFF / 2; d2 += SIMULATION_ANGLE_DIFF / SIMULATION_CHILD_COUNT) {

                double pod1Angle = pod1BaseAngle + d1;
                double pod2Angle = pod2BaseAngle + d2;
                Ship2D newPod1 = calculateNewPodLocation(&pod1, pod1Angle);
                Ship2D newPod2 = calculateNewPodLocation(&pod2, pod2Angle);
                // TODO: Opponents and collisions

                node = SimulationNode(track, newPod1, newPod2, opponent1, opponent2).evaluate(depth - 1);

                if (node.score > score) {
                    score = node.score;
                    best = node; // Unnecessary?
                }
            }
        }
        cerr << "Best score: " << best.score << endl;
        return best;
    }

private:
    Ship2D calculateNewPodLocation(Ship2D *pod, double podAngle) {
        podAngle = podAngle / 180 * PI;
        Vector2D newVelocity = Vector2D((pod->velocity.x + cos(podAngle) * pod->thrust) * DRAG,
                                        (pod->velocity.y + sin(podAngle) * pod->thrust) * DRAG);
        Vector2D newPos = Vector2D(pod->pos.x + newVelocity.x, pod->pos.y + newVelocity.y); // Use average instead?
        int newCpId = length(pod->pos, track->getCp(pod->cpId)) < CHECKPOINT_RADIUS ? pod->cpId + 1 : 0;
        double newAngle = angle(newPos, track->getCp(newCpId));
        return Ship2D(newPos, newVelocity, newAngle, newCpId, pod->thrust, pod->boostUsed, pod->shieldCoolDown - 1,
                      pod->target); // Remove target?
    }

    double getNodeScore(SimulationNode *node) {
        int pod1CpId = node->pod1.cpId;
        int pod2CpId = node->pod2.cpId;

        double pod1CpDistance = length(node->pod1.pos, track->getCp(pod1CpId));
        double pod2CpDistance = length(node->pod2.pos, track->getCp(pod2CpId));

        pod1CpDistance += pod1CpId * MAX_CP_DISTANCE;
        pod2CpDistance += pod1CpId * MAX_CP_DISTANCE;
        // TODO: Subtract opponents
        return pod1CpDistance + pod2CpDistance;
    }
};

class AI {

};

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
    track.lapsCount = lapsCount;
    int cX, cY;
    for (int i = 0; i < checkpointsCount; i++) {
        cin >> cX >> cY;
        track.addNewCheckpoint(Vector2D(cX, cY));
    }

    int x, y, vx, vy, angle, cpId;


    // game loop
    while (1) {
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        track.updatePod(&track.pod1, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        track.updatePod(&track.pod2, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        track.updatePod(&track.opponent1, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        track.updatePod(&track.opponent2, x, y, vx, vy, angle, cpId);

        // cerr << "DT" << dt<<endl;

        track.plan();

        cout << track.pod1.target.x << " " << track.pod1.target.y << " ";
        if (track.pod1.thrust > MAX_THRUST) {
            cout << "BOOST" << endl;
        } else if (track.pod1.shieldCoolDown == SHIELD_COOL_DOWN) {
            cout << "SHIELD" << endl;
        } else {
            cout << track.pod1.thrust << endl;
        }

        cout << track.pod2.target.x << " " << track.pod2.target.y << " ";
        if (track.pod2.thrust > MAX_THRUST) {
            cout << "BOOST" << endl;
        } else if (track.pod2.shieldCoolDown == SHIELD_COOL_DOWN) {
            cout << "SHIELD" << endl;
        } else {
            cout << track.pod2.thrust << endl;
        }

    }
}

#pragma clang diagnostic pop