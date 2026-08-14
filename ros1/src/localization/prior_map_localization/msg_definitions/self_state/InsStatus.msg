int8 INS_INACTIVE       =  0 # IMU logs are present, but the alignment routine has not started; INS is inactive.
int8 INS_ALIGNING       =  1 # INS is in alignment mode
int8 INS_HIGH_VARIANCE  =  2 # The INS solution is in navigation mode but the azimuth solution
                             # uncertainty has exceeded the threshold. The default threshold
                             # is 2 degrees for most IMUs. a The solution is still valid but you
                             # should monitor the solution uncertainty in the INSCOV log.
                             # You may encounter this state during times when the GNSS, used to aid the INS, is absent
int8 INS_SOLUTION_GOOD = 3   # Singularity at parameters matrix
int8 INS_SOLUTION_FREE = 6   # The INS filter is in navigation mode and the GNSS solution is suspected to be in error.
                             # This may be due to multipath or limited satellite visibility. The
                             # inertial filter has rejected the GNSS position and is waiting for the solution quality to improve
int8 INS_ALIGNMENT_COMPLETE = 7  # The INS filter is in navigation mode, but not enough vehicle dynamics have 
                                 # been experienced for the system to be withinspecifications
int8 DETERMINING_ORIENTATION = 8 # INS is determining the IMU axis aligned with gravity.
int8 WAITING_INITIALPOS      = 9 # The INS filter has determined the IMU orientation and is awaiting an initial position estimate to begin the alignment process..

int8   ins_status

