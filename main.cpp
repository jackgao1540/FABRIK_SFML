#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <iostream>

const int WIDTH = 1000, HEIGHT = 700;
const float JOINT_RADIUS = 5.0f;  
const int MAXJOINTS = 7;

using namespace sf;

class Arm {
    public:
        Arm(sf::RenderWindow& window) : window(window) {
            // Initialize the base of the arm
            joints.push_back(Vector2f(WIDTH / 2, HEIGHT/2));
            isMarkerSet = false;
        }

        void addJoint(Vector2f position) {
            if (joints.size() >= MAXJOINTS) return;

            joints.push_back(position);

            if (joints.size() > 1) {
                // Calculate the length and angle of the new segment
                Vector2f prevJoint = joints[joints.size() - 2];
                Vector2f newSegment = position - prevJoint;
                lengths.push_back(sqrt(newSegment.x * newSegment.x + newSegment.y * newSegment.y));
                angles.push_back(atan2(newSegment.y, newSegment.x));

                lines.push_back(Vertex(prevJoint));
                lines.push_back(Vertex(position));
            }

            if (isMarkerSet) {
                inverseKinematicsStep();
            }

            std::cout << "Added Joint " << joints.size() << " with length " << lengths[lengths.size()-1] << std::endl;
        }

        void removeMarker() {
            isMarkerSet = false;
            optimalAngles.clear();
        }

        void removeJoint() {
            if(joints.size()<=1)return;
            // pop the latest element of all associated variables
            joints.pop_back();
            lines.pop_back();
            lines.pop_back();
            lengths.pop_back();
            angles.pop_back();
            optimalAngles.clear();
            // std::cout<<lines.size()<<std::endl;
            // std::cout<<lengths.size()<<std::endl;
            if (isMarkerSet) {
                inverseKinematicsStep();
            }
        }

        void setMarker(Vector2f position) {
            markerPosition = position;
            isMarkerSet = true;
            inverseKinematicsStep();
        }

        void updateBasedOnJoints(std::vector<Vector2f>j, std::vector<float>a){
            for (size_t i = 0; i < j.size() - 1; i++) {
                Vector2f segment = j[i + 1] - j[i];
                // lengths[i] = sqrt(segment.x * segment.x + segment.y * segment.y);
                a[i] = atan2(segment.y, segment.x);
            }

            // for (size_t i = 0; i < lines.size(); i += 2) {
            //     lines[i].position = joints[i / 2];
            //     lines[i + 1].position = joints[i / 2 + 1];
            // }
        }

        void inverseKinematicsStep() {
            if (!isMarkerSet) {
                optimalAngles.clear();
                return;
            }

            Vector2f target = markerPosition;

            std::vector<Vector2f> _j = joints;
            std::vector<float> _a = angles;

            if (distance(_j[0], target) > totalArmLength()) {
                for (size_t i = 1; i < _j.size(); i++) {
                    Vector2f direction = normalize(target - _j[i - 1]);
                    _j[i] = _j[i - 1] + direction * lengths[i - 1];
                }
                // updateBasedOnJoints(_j, _a);
            } else {
                Vector2f base = _j[0];
                float difference;

                _j.back() = target;
                for (int i = _j.size() - 2; i >= 0; i--) {
                    Vector2f direction = normalize(_j[i] - _j[i + 1]);
                    _j[i] = _j[i + 1] + direction * lengths[i];
                }
                
                _j[0] = base;
                for (size_t i = 1; i < _j.size(); i++) {
                    Vector2f direction = normalize(_j[i] - _j[i - 1]);
                    _j[i] = _j[i - 1] + direction * lengths[i - 1];
                }

                difference = distance(_j.back(), target);

                // updateBasedOnJoints(_j, _a);
            }
            for (size_t i = 0; i < _j.size() - 1; i++) {
                Vector2f segment = _j[i + 1] - _j[i];
                // lengths[i] = sqrt(segment.x * segment.x + segment.y * segment.y);
                _a[i] = atan2(segment.y, segment.x);
            }
            optimalAngles = _a;
        }

        void moveTowardsOptimalAngles() {
            if(optimalAngles.empty() || optimalAngles.size() != angles.size()) {
                std::cout<<"Error, optimalAngles empty or size doesn't match."<<std::endl;
                return;
            }
            for(size_t i = 0; i < angles.size(); i++){
                if(std::abs(angles[i] - optimalAngles[i]) > SPEED) {
                    // see which direction is faster
                    optimalAngles[i] = adjustAngle(optimalAngles[i]);
                    angles[i] = adjustAngle(angles[i]);
                    float diff = optimalAngles[i] - angles[i];
                    if(diff > M_PI) {
                        // negative
                        angles[i] -= SPEED;
                    } else if (diff < -M_PI){
                        // positive
                        angles[i] += SPEED;
                    } else {
                        angles[i] += SPEED * (diff >= 0.0f ? 1 : -1);
                    }
                    
                } else {
                    angles[i] = optimalAngles[i];
                }
            }
            std::cout<<"updated angles" << std::endl;
            printDebugInfo();
            // recalculatePositions();
        }

        float adjustAngle(float angle) {
            float twoPi = 2 * M_PI;

            // First, bring the angle into the range [-2π, 2π]
            angle = fmod(angle, twoPi);

            // Now, adjust negative values to be within [0, 2π]
            if (angle < 0) {
                angle += twoPi;
            }

            return angle;
        }

        void recalculatePositions() {
            for (size_t i = 1; i < joints.size(); ++i) {
                Vector2f prevJoint = joints[i - 1];
                joints[i] = Vector2f(prevJoint.x + lengths[i - 1] * cos(angles[i - 1]),
                                    prevJoint.y + lengths[i - 1] * sin(angles[i - 1]));
            }

            // Update the lines to reflect the new positions
            for (size_t i = 0; i < lines.size(); i += 2) {
                lines[i].position = joints[i / 2];
                lines[i + 1].position = joints[i / 2 + 1];
            }
        }

        void draw() {
            recalculatePositions();
            if (!lines.empty()) {
                window.draw(&lines[0], lines.size(), sf::Lines);
            }

            for (size_t i = 0; i < joints.size(); ++i) {
                CircleShape circle(JOINT_RADIUS);
                circle.setPosition(joints[i].x - JOINT_RADIUS, joints[i].y - JOINT_RADIUS);
                circle.setFillColor(i == joints.size() - 1 ? Color::Green : (i == 0 ? Color::Blue : Color::Red));
                window.draw(circle);
            }
            if (isMarkerSet) {
                CircleShape marker(JOINT_RADIUS);
                marker.setPosition(markerPosition.x - JOINT_RADIUS, markerPosition.y - JOINT_RADIUS);
                marker.setFillColor(Color::Cyan);
                window.draw(marker);
            }
        }

        bool hasTarget(){
            return isMarkerSet;
        }

        int numJoints() {
            return joints.size();
        }

        void randMove() {
            float change = 0.00013;
            float rate = -0.9;
            for(int i = 0; i < angles.size(); i++){
                angles[i] += change;
                change *= rate;
            }
        }

    private:
        RenderWindow& window;
        std::vector<Vector2f> joints;
        std::vector<Vertex> lines;
        std::vector<float> lengths;
        std::vector<float> angles;
        std::vector<float> optimalAngles;
        sf::Vector2f markerPosition;
        bool isMarkerSet;
        const float SPEED = 0.0002;

        void printDebugInfo() {
            // std::cout << "Joints:\n";
            // for (size_t i = 0; i < joints.size(); ++i) {
            //     std::cout << "Joint " << i << ": (" << joints[i].x << ", " << joints[i].y << ")\n";
            // }

            // std::cout << "Lengths:\n";
            // for (size_t i = 0; i < lengths.size(); ++i) {
            //     std::cout << "Segment " << i << ": " << lengths[i] << "\n";
            // }

            std::cout << "Angles (in degrees):\n";
            for (size_t i = 0; i < angles.size(); ++i) {
                std::cout << "Segment " << i << ": " << angles[i] * 180.0f / M_PI << "\n";
            }

            std::cout << "Optimal Angles (in degrees):\n";
            for (size_t i = 0; i < optimalAngles.size(); ++i) {
                std::cout << "Segment " << i << ": " << optimalAngles[i] * 180.0f / M_PI << "\n";
            }

            // std::cout << "Lines:\n";
            // for (size_t i = 0; i < lines.size(); i += 2) {
            //     std::cout << "Line " << i/2 << ": From (" << lines[i].position.x << ", " << lines[i].position.y << ") to (" << lines[i + 1].position.x << ", " << lines[i + 1].position.y << ")\n";
            // }

            std::cout << std::endl;
        }
        float distance(const Vector2f& a, const Vector2f& b) {
            Vector2f diff = a - b;
            return sqrt(diff.x * diff.x + diff.y * diff.y);
        }
        Vector2f normalize(const Vector2f& v) {
            float len = distance(v, Vector2f(0, 0));
            if (len == 0) return Vector2f(0, 0);
            return v / len;
        }
        float totalArmLength() {
            float totalLength = 0.0;
            for (auto length : lengths) {
                totalLength += length;
            }
            return totalLength;
        }
};

int main() {
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "FABRIK");
    Arm arm(window);

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::MouseButtonPressed) {
                if (event.mouseButton.button == sf::Mouse::Left && arm.numJoints() != MAXJOINTS) {
                    arm.addJoint(window.mapPixelToCoords(sf::Mouse::getPosition(window)));
                } else if (event.mouseButton.button == sf::Mouse::Right) {
                    arm.setMarker(window.mapPixelToCoords(sf::Mouse::getPosition(window)));
                }
            }
            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::X) {
                    arm.removeMarker();
                } else if(event.key.code == sf::Keyboard::D) {
                    arm.removeJoint();
                }
            }
        }



        // adding movement
        if (arm.hasTarget()) {
            arm.moveTowardsOptimalAngles();
        } else {
            arm.randMove();
        }

        window.clear();
        
        arm.draw();

        window.display();
    }

    return 0;
}
