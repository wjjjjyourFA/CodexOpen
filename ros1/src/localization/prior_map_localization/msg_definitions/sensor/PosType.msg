int8 NONE         = 0   # No solution
int8 FIXEDPOS     = 1   # Position has been fixed by the FIX POSITION command or by position averaging
int8 FIXEDHEIGHT  = 2   # Position has been fixed by the FIX HEIGHT, or FIX AUTO, command or by position averaging
int8 FLOATCONV    = 4   # Solution from floating point carrier phase ambiguities
int8 WIDELANE     = 5   # Solution from wide-lane ambiguities
int8 NARROWLANE   = 6   # Solution from narrow-lane ambiguities
# 7 Reserved
int8 DOPPLER_VELOCITY  = 8   # Velocity computed using instantaneous Doppler
int8 SINGLE         = 16  # Single point position
int8 PSRDIFF        = 17  # Pseudorange differential solution
int8 WAAS           = 18  # Solution calculated using corrections from an SBAS
int8 PROPOGATED     = 19  # Propagated by a Kalman filter without new observations
int8 OMNISTAR       = 20  # OmniSTAR VBS position (L1 sub-meter)
#21-31 Reserved
int8 L1_FLOAT       = 32  # Floating L1 ambiguity solution
int8 IONOFREE_FLOAT = 33  # Floating ionospheric free ambiguity solution
int8 NARROW_FLOAT   = 34  # Floating narrow-lane ambiguity solution
# 35-47 Reserved
int8 L1_INT             = 48  # Integer L1 ambiguity solution
int8 WIDE_INT           = 49  # Integer wide-lane ambiguity solution
int8 NARROW_INT         = 50  # Integer narrow-lane ambiguity solution
int8 RTK_DIRECT_INS     = 51  # RTK status where the RTK filter is directly initialized from the INS filter
int8 INS_SBAS           = 52  # INS calculated position corrected for the antenna
int8 INS_PSRSP          = 53  # INS pseudorange single point solution – no DGPS corrections
int8 INS_PSRDIFF        = 54  # INS pseudorange differential solution
int8 INS_RTKFLOAT       = 55  # INS RTK floating point ambiguities solution
int8 INS_RTKFIXED       = 56  # INS RTK fixed ambiguities solution
int8 INS_OMNISTAR       = 57  # INS OmniSTAR VBS position (L1 sub-meter)
int8 INS_OMNISTAR_HP    = 58  # INS OmniSTAR high precision solution
int8 INS_OMNISTAR_XP    = 59  # INS OmniSTAR extra precision solution
# 60-63 Reserved
int8 OMNISTAR_HP        = 64  # OmniSTAR high precision
int8 OMNISTAR_XP        = 65  # OmniSTAR extra precision
# 66-67 Reserved
int8 PPP_CONVERGING     = 68  # Precise Point Positioning (PPP) solution converging
int8 PPP                = 69  # Precise Point Positioning (PPP) solution
# 70-72 Reserved
int8 INS_PPP_Converging = 73  # INS NovAtel CORRECT Precise Point Positioning (PPP) solution converging
int8 INS_PPP            = 74  # INS NovAtel CORRECT Precise Point Positioning (PPP) solution

int8 pos_type

