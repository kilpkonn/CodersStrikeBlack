#include <iostream>
#include <vector>
#include <cmath>

#define CHECKPOINT_RADIUS 600
#define POD_RADIUS 400
#define MAX_CP_DISTANCE 16000

#define MAX_THRUST 100
#define BOOST_THRUST 650
#define SHIELD_COOL_DOWN 3
#define DRAG 0.85

#define MAX_ANGLE_ALLOWED_BOOST 15
#define MIN_DISTANCE_ALLOWED_BOOST 6000
#define TARGET_AHEAD_DISTANCE 5000
#define MIN_SHIELD_IMPULSE 200

#define SIMULATION_CHILD_COUNT 7
#define SIMULATION_DEPTH 4
#define SIMULATION_ANGLE_DIFF 110
#define SIMULATION_STEP_LENGTH 1

#define PI 3.14159265

using namespace std;

struct Vector2D {
    double x, y;

    Vector2D() = default;

    Vector2D(double x, double y) : x(x), y(y) {}

    bool operator==(const Vector2D &v) const {
        return x == v.x && y == v.y;
    }

    bool operator!=(const Vector2D &v) const {
        return x != v.x || y != v.y;
    }

    Vector2D operator+(const Vector2D &v) const {
        return {x + v.x, y + v.y};
    }

    Vector2D operator-(const Vector2D &v) const {
        return {x - v.x, y - v.y};
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
    int cpId;
    double angle;
    Vector2D pos, velocity;
    int thrust = MAX_THRUST;
    bool boostUsed = false;
    int shieldCoolDown = 0;
    Vector2D target = Vector2D(0, 0);

    Ship2D() : pos(Vector2D(0, 0)), velocity(Vector2D(0, 0)), angle(0), cpId(0) {};

    Ship2D(const Vector2D &pos,
           const Vector2D &velocity,
           double angle,
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
    int lapsCount = 3;
    int checkpointsCount = 3;

    void addNewCheckpoint(const Vector2D point) {
        checkpoints.push_back(point);
        cerr << "New point: " << checkpoints.back().x << " - " << checkpoints.back().y;
    }

    Vector2D getCp(const unsigned index) {
        return checkpoints[index % checkpoints.size()];
    }

    static double calcTurnAngle(const Vector2D &ship, const Vector2D &currentCP, const Vector2D &nextCp) {
        // double pointAngle = angle(ship, currentCP);
        // double nextAngle = angle(currentCP, nextCp);
        return normalize_angle(angle(currentCP, nextCp) - angle(ship, currentCP));
    }

    static Vector2D calcOptimalCpPos(const Ship2D *ship, const Vector2D &cp, const Vector2D &nextCp) {
        Vector2D medianPoint = Vector2D((ship->pos.x + nextCp.x), (ship->pos.y + nextCp.y) / 2);
        double radAngleFromCp = angle(cp, medianPoint) / 180 * PI;
        return {cp.x + CHECKPOINT_RADIUS * 0.5 * cos(radAngleFromCp),
                cp.y + CHECKPOINT_RADIUS * 0.5 * radAngleFromCp};
    }

    static double calcRelativeCollisionImpulse(const Vector2D *base, const Vector2D *v) {
        //cerr << "Ipulse " << sin(angle(*base, *v) / 180 * PI) * length(*v) << endl;
        return sin(angle(*base, *v) / 180 * PI) * length(*v); // TODO: Validate if calculation is correct
    }

    static Vector2D calculateTarget(const Ship2D *ship, const double &angle) {
        double radAngle = angle / 180 * PI;
        return {ship->pos.x + TARGET_AHEAD_DISTANCE * cos(radAngle),
                ship->pos.y + TARGET_AHEAD_DISTANCE * sin(radAngle)};
    }

     void evaluateShield(Ship2D *ship, const Ship2D *opponent1, const Ship2D *opponent2, const Vector2D &cp) {

        // Track::calcRelativeCollisionImpulse(&ship->velocity, &opponent1->velocity);

        Ship2D newPod = calculateNewPodLocation(ship, angle(ship->target));
        Ship2D newOpponent1 = calculateNewPodLocation(opponent1, angle(opponent1->velocity), 0);
        Ship2D newOpponent2 = calculateNewPodLocation(opponent2, angle(opponent2->velocity), 0);

        if (willCollide(ship, opponent1, 2)) {
            if (length(opponent1->velocity) > 200
            && abs(normalize_angle(angle(ship->pos, cp) - angle(ship->pos, opponent1->pos))) < 45) {
                cerr << "Shield for 1!" << endl;
                ship->shieldCoolDown = SHIELD_COOL_DOWN;
            }
        }

        if (willCollide(ship, opponent2, 2)) {
            if (length(opponent2->velocity) > 200
            && abs(normalize_angle(angle(ship->pos, cp) - angle(ship->pos, opponent2->pos))) < 45) {
                ship->shieldCoolDown = SHIELD_COOL_DOWN;
                cerr << "Shield for 2!" << endl;
            }
        }
    }


    Ship2D calculateNewPodLocation(const Ship2D *pod, double podAngle, double steps = 1) {
        podAngle = podAngle / 180 * PI;
        Vector2D newVelocity = Vector2D((pod->velocity.x + cos(podAngle) * pod->thrust) * DRAG,
                                        (pod->velocity.y + sin(podAngle) * pod->thrust) * DRAG);
        Vector2D newPos = Vector2D(pod->pos.x + newVelocity.x * steps, pod->pos.y + newVelocity.y * steps); // Use average instead?
        int newCpId = length(pod->pos, getCp(pod->cpId)) < CHECKPOINT_RADIUS ? pod->cpId + 1 : pod->cpId;
        double newAngle = angle(newPos, getCp(newCpId));
        return {newPos, newVelocity, newAngle, newCpId, pod->thrust, pod->boostUsed, pod->shieldCoolDown - 1,
                pod->target}; // Remove target?
    }

    bool willCollide(const Ship2D* pod1, const Ship2D*pod2, int steps) {
        bool result = false;

        for (int i = 0; i < steps; i++) {
            Ship2D newPod1 = calculateNewPodLocation(pod1, angle(pod1->velocity), i + 1);
            Ship2D newPod2 = calculateNewPodLocation(pod2, angle(pod2->velocity), i + 1);

            result |= length(newPod1.pos, newPod2.pos) <= POD_RADIUS * 2;
        }

        return result;
    }

private:
    vector<Vector2D> checkpoints;
};

class SimulationNode {
public:
    Track *track;
    Ship2D pod1, pod2, opponent1, opponent2;
    double podBestAngle = 0, podBestThrust = 0;
    double score = 0;

    SimulationNode() : track(nullptr) {}

    SimulationNode(Track *track,
                   const Ship2D &pod1,
                   const Ship2D &pod2,
                   const Ship2D &opponent1,
                   const Ship2D &opponent2) :
            track(track),
            pod1(pod1),
            pod2(pod2),
            opponent1(opponent1),
            opponent2(opponent2) {}

    SimulationNode evaluate(int depth = SIMULATION_DEPTH) {
        if (depth <= 0) {
            score = getNodeScore(this);
            return *this;
        }

        double pod1BaseAngle = angle(pod1.pos, Track::calcOptimalCpPos(&pod1, track->getCp(pod1.cpId), track->getCp(pod1.cpId + 1)));

        double pod1Angle;
        // cerr << depth << " Cpids: " << pod1.cpId << " - " << pod2.cpId << endl;

        SimulationNode node, best;

        for (short d1 = -SIMULATION_ANGLE_DIFF / 2;
             d1 <= SIMULATION_ANGLE_DIFF / 2; d1 += SIMULATION_ANGLE_DIFF / (SIMULATION_CHILD_COUNT - 1)) {

            for (int thrust = 0; thrust <= MAX_THRUST; thrust += MAX_THRUST) {
                //cerr << thrust << endl;
                pod1Angle = pod1BaseAngle + d1;
                pod1.thrust = thrust;
                Ship2D newPod1 = track->calculateNewPodLocation(&pod1, pod1Angle, SIMULATION_STEP_LENGTH);
                Ship2D newPod2 = track->calculateNewPodLocation(&pod2, angle(pod2.velocity), SIMULATION_STEP_LENGTH);

                Ship2D newOpponent1 = track->calculateNewPodLocation(&opponent1, angle(opponent2.velocity), SIMULATION_STEP_LENGTH);
                Ship2D newOpponent2 = track->calculateNewPodLocation(&opponent2, angle(opponent2.velocity), SIMULATION_STEP_LENGTH);

                //if (depth == SIMULATION_DEPTH && d1 == -SIMULATION_ANGLE_DIFF / 2) {
                //    cerr << pod1.velocity.x << " - " << pod1.velocity.y << " o " << opponent1.velocity.x << " - " << opponent1.velocity.y << endl;
                    resolveCollisions(&newPod1, &newOpponent1);
                    resolveCollisions(&newPod1, &newOpponent2);
                //}

                // resolveCollisions(&newPod1, &newPod2, &newOpponent1, &newOpponent2);

                node = SimulationNode(track, newPod1, newPod2, opponent1, opponent2).evaluate(depth - 1);

                if (node.score > score) {
                    // cerr << "Depth " << depth << endl;
                    // cerr << "Pod1 angle " << pod1Angle;
                    score = node.score;
                    best = node;
                    // We need pod angles from here, not from leaf node
                    best.podBestAngle = pod1Angle;
                    best.podBestThrust = thrust;
                }
            }
        }
        return best;
    }

private:
    void resolveCollisions(Ship2D *pod1, Ship2D *pod2) {
        if (length(pod1->pos, pod2->pos) > POD_RADIUS * 2) return;

        double px = pod1->velocity.x + pod2->velocity.x;
        double py = pod1->velocity.y + pod2->velocity.y;

        double v1x = px - pod1->velocity.x;
        double v1y = px - pod1->velocity.x;

        // cerr << v1x << " | " << v1y << "  <> " << v2x <<  " | " << v2y << endl;
        pod1->velocity = Vector2D(px - pod1->velocity.x, px - pod1->velocity.x);
        pod2->velocity = Vector2D(px - pod2->velocity.x, py - pod2->velocity.y);
    }

    /**
     * Bigger better
     * @param node
     * @return score
     */
    double getNodeScore(SimulationNode *node) {
        int pod1CpId = node->pod1.cpId;

        double score = MAX_CP_DISTANCE - length(node->pod1.pos, Track::calcOptimalCpPos(&node->pod1, track->getCp(pod1CpId), track->getCp(pod1CpId + 1)));

        score += pod1CpId * MAX_CP_DISTANCE;

        double opp1Dist = MAX_CP_DISTANCE - length(node->opponent1.pos, track->getCp(node->opponent1.cpId));
        double opp2Dist = MAX_CP_DISTANCE - length(node->opponent2.pos, track->getCp(node->opponent2.cpId));

        if (node->opponent1.cpId * MAX_CP_DISTANCE + opp1Dist > node->opponent2.cpId*MAX_CP_DISTANCE + opp2Dist) {
            score -= node->opponent1.cpId * MAX_CP_DISTANCE / 2.0;
            score -= opp1Dist / 2.0;
        } else {
            score -= node->opponent2.cpId * MAX_CP_DISTANCE / 2.0;
            score -= opp2Dist / 2.0;
        }

        return score;
    }
};

class AI {
public:
    Track track = Track();

    Ship2D pod1 = Ship2D();
    Ship2D pod2 = Ship2D();
    Ship2D opponent1 = Ship2D();
    Ship2D opponent2 = Ship2D();

    AI() = default;

    void updatePod(Ship2D *pod, const int &x, const int &y, const int &vx, const int &vy, const int &angle, const int &cpId) {
        pod->pos = Vector2D(x, y);
        pod->velocity = Vector2D(vx, vy);
        pod->angle = angle;
        if (cpId != pod->cpId % track.checkpointsCount) {
            pod->cpId++;
            // cerr << "! ============= CP =============== !" << endl;
        }
    }

    void plan() {
        pod1.shieldCoolDown--;
        pod2.shieldCoolDown--;

        SimulationNode root = SimulationNode(&track, pod1, pod2, opponent1, opponent2);
        SimulationNode best = root.evaluate();

        pod1.target = Track::calculateTarget(&pod1, best.podBestAngle);
        pod1.thrust = best.podBestThrust;

        track.evaluateShield(&pod1, &opponent1, &opponent2, track.getCp(pod1.cpId));
        // track.evaluateBoost2(&pod1, &opponent1, &opponent2);

        //Track::evaluateShield(&pod2, &opponent1, &opponent2, track.getCp(pod2.cpId));
        //track.evaluateBoost(&pod2);
        planRam(&pod2);
    }

private:
    bool isReadyToRam = false;
    bool isRamming = false;

    void planRam(Ship2D *pod) {
        Ship2D opponentToRam = opponent1;
        Vector2D cp = track.getCp(opponent1.cpId);
        if (opponent2.cpId > opponent1.cpId ||
            (opponent1.cpId == opponent2.cpId && length(opponent1.pos, cp) > length(opponent2.pos, cp) * 1.4)) {
            opponentToRam = opponent2;
            cp = track.getCp(opponent2.cpId);
        }

        if (!isRamming) {
            // Setup
            Vector2D torrentCp = track.getCp(opponentToRam.cpId);
            if (length(pod->pos, torrentCp) * 1.5 < length(opponentToRam.pos, torrentCp)) {
                cerr << "Lets just ram!" << endl;
                isRamming = true;
            } else {
                torrentCp = track.getCp(opponentToRam.cpId + 1);
            }

            if (length(pod->pos, torrentCp) > length(opponentToRam.pos, cp) + length(cp, torrentCp)) {
                cerr << "Gotta take shortcut" << endl;
                torrentCp = track.getCp(opponentToRam.cpId + 2);
            }

            // Drive to torrent cp
            pod->target = torrentCp;

            // Geometric array sum
            if (length(pod->pos, torrentCp) > length(pod->velocity) / (1 - DRAG) && !track.willCollide(&pod1, &pod2, 7)) {
                pod->thrust = MAX_THRUST;
            } else {
                pod->thrust = 0;
            }
            cerr << "Dist till cp: " << length(pod->pos, torrentCp) << endl;
            if (length(pod->pos, torrentCp) < CHECKPOINT_RADIUS * 2) {
                cerr << "Lets go ram!" << endl;
                isRamming = true;
            }
        } else {
            // Ram
            Vector2D target = Vector2D(opponentToRam.pos.x + opponentToRam.velocity.x * 4,
                                       opponentToRam.pos.y + opponentToRam.velocity.y * 4);
            if (track.willCollide(&pod1, &pod2, 7)) {
                cerr << "Friendly fire!!!" << endl;
                pod->target = Track::calculateTarget(pod, angle(pod->pos, pod1.pos) + 90);

            } else {
                pod->target = target;
            }
            pod->thrust = MAX_THRUST;
            if (abs(normalize_angle(pod->angle - angle(pod->pos, target))) > 45) {
                pod->thrust = 0;
            }

            cerr << "angle " << pod->angle << " vs " << angle(pod->pos, target) << endl;
            cerr << "Dist till collision" << length(pod->pos, opponentToRam.pos) << endl;

            Ship2D newPod = track.calculateNewPodLocation(pod, angle(pod->target));
            Ship2D newOpponent1 = track.calculateNewPodLocation(&opponent1, angle(opponent1.velocity), 1.5);
            Ship2D newOpponent2 = track.calculateNewPodLocation(&opponent2, angle(opponent2.velocity), 1.5);

            if (length(newPod.pos, newOpponent1.pos) < POD_RADIUS * 2 || length(newPod.pos, newOpponent2.pos) < POD_RADIUS * 2) {
                cerr << "SHIELD!" << endl;
                pod->shieldCoolDown = SHIELD_COOL_DOWN;
                // isRamming = false;
            }

            if (length(pod->pos, track.getCp(opponentToRam.cpId)) > length(opponentToRam.pos, track.getCp(opponentToRam.cpId)) + POD_RADIUS * 2) {
                isRamming = false;
            }
        }
    }
};

void write_pod(const Ship2D &pod);

/**
 * This code automatically collects game data in an infinite loop.
 * It uses the standard input to place data into the game variables such as x and y.
 * YOU DO NOT NEED TO MODIFY THE INITIALIZATION OF THE GAME VARIABLES.
 **/
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

int main() {
    AI ai = AI();

    int lapsCount = 0;
    int checkpointsCount = 0;
    cin >> lapsCount >> checkpointsCount;
    ai.track.lapsCount = lapsCount;
    ai.track.checkpointsCount = checkpointsCount;
    int cX = 0, cY = 0;
    for (int i = 0; i < checkpointsCount; i++) {
        cin >> cX >> cY;
        ai.track.addNewCheckpoint(Vector2D(cX, cY));
    }

    int x = 0, y = 0, vx = 0, vy = 0, angle = 0, cpId = 0;


    // game loop
    while (true) {
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ai.updatePod(&ai.pod1, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ai.updatePod(&ai.pod2, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ai.updatePod(&ai.opponent1, x, y, vx, vy, angle, cpId);
        cin >> x >> y >> vx >> vy >> angle >> cpId;
        ai.updatePod(&ai.opponent2, x, y, vx, vy, angle, cpId);

        // cerr << "DT" << dt<<endl;

        ai.plan();

        write_pod(ai.pod1);
        write_pod(ai.pod2);

    }
}

void write_pod(const Ship2D &pod) {
    cout << (int) pod.target.x << " " << (int) pod.target.y << " ";
    if (pod.thrust > MAX_THRUST) {
        cout << "BOOST" << endl;
    } else if (pod.shieldCoolDown == SHIELD_COOL_DOWN) {
        cout << "SHIELD" << endl;
    } else {
        cout << pod.thrust << endl;
    }
}

#pragma clang diagnostic pop