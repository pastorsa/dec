int32 id
float64 percent_complete
int32 status
int32 IN_PROGRESS=0
int32 FINISHED=1
int32 PREEMPTED=2
time start_time
time end_time

# current trajectory point:
string[] dimension_names
TrajectoryPoint point
