# AutonomyLeg.msg - Nelson Durrant, Feb 2025
# Used to define a leg of the rover's autonomy task

string name
string type       # type of leg ("gps", "aruco", "obj")
float64 latitude
float64 longitude

uint8 tag_id      # tag number (1, 2, 3, etc)
string object_id  # which object to find ("mallet", "bottle")