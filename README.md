# FABRIK_SFML
This project uses the FABRIK algorithm and the SFML C++ library to allow the user to:
- Dynamically ADD (left-click) and REMOVE (press 'D') joints to/from an arm segment, automatically calculating limb lengths
- Randomly moves arm when no target is set
- Allow for the setting of a target, which the arm will immediately attempt to position itself into reaching
- Robustly recalculating optimal arm angles whenever joints are added/deleted
