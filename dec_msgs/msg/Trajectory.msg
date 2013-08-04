Header header

# ID of this trajectory, to identify controller status messages
# Must be positive.
int32 id

# If preempt is true, halts the current trajectory immediately
# WARNING: this will cause a jerk in the control inputs, because the
# velocities and accelerations are forced to zero
# Note: dimension_names and points can be left empty.
bool preempt

string[] dimension_names
TrajectoryPoint[] points
