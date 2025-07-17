# Get current heading from the IMU.
# Get current position(X,Y,Z) from the DVL
# Set target position()
# Using the current heading, calulate the angle from current position and target position
# Using the pythagoran theorem, break the distance down into X and Y components to find the distance to move in each direction
# Move in the X direction until the target X is reached
# Move in the Y direction until the target Y is reached
# Can use a PID loop to adjust the movement based on the current position