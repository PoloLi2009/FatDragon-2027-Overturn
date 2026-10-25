# Public-project vision localization

Source: FTC-16095-FatDragon-2027-Public, subsystems/vision.

The localization update and solveHiveFlatPose mathematics are extracted from
the public project. Camera connection, pipeline, frame age/frozen-frame,
alliance, tag-area and minimum-two-tag checks are preserved.
The original solver accepts tilt magnitudes up to 35 degrees and level error
up to 10 degrees, with pivot height 43.95 inches. It does not impose 30 +/- 5 degrees.
Camera mounting pose must be configured in Limelight.

The Limelight localization itself remains isolated from shooter and intake logic.
The competition TeleOps use the user's tuned Shooter Test PID values through
separate command-based Shooter and Feeder subsystems.
getCorrectedRobotFieldPose() returns Optional<Pose2D>; empty means unavailable.

Run "Test: Limelight Position Only" for a camera-only position display.
It requires only "limelight"; X switches blue pipeline 0 / red pipeline 1.
Run "Test: Limelight + Odometry" for driving and rear aiming:
- Each valid new corrected vision frame directly replaces Pinpoint X/Y/heading.
  No weighted averaging, A button, distance threshold or correction cooldown.
  The existing public vision validity checks remain in effect.
- A physical frame is applied only once; between frames or without vision,
  Pinpoint continues tracking. Before the first valid field pose, aiming is disabled.
- X switches alliance (blue pipeline 0, red pipeline 1).
- Hold RIGHT trigger (>0.1) to point the robot rear at the selected Cell.
  Left-stick field-centric translation remains active; release for manual rotation.
- Targets below are copied from the public project's nominal Cell aim points,
  not newly measured from STEP. X >= 0 chooses audience-side, X < 0 opposite.

| Cell | X (inches) | Y (inches) |
| --- | ---: | ---: |
| Red opposite audience | -17.7 | -12.75 |
| Red audience | 17.7 | -12.75 |
| Blue audience | 17.7 | 12.75 |
| Blue opposite audience | -17.7 | 12.75 |

Rear aim uses atan2(targetY - robotY, targetX - robotX) - 180 degrees
as the desired robot heading, with shortest-angle error. A simple proportional
turn controller is limited to 0.40 power with a 2-degree tolerance; these are
initial bench-test settings, not verified on this robot. Positive turn power
is clockwise. Telemetry displays the target, error, aim state and vision update.
The Pinpoint X and Y encoder direction reversals from robot testing are retained.

## Competition TeleOp

Select Competition: BLUE or Competition: RED; that choice fixes the alliance
and Limelight pipeline for the match.

- RT simultaneously runs rear auto-aim and the tuned firing sequence.
- LT runs intake and transit intake.
- D-pad left/right changes shooter target by 50 ticks/s.
- D-pad down/up changes pitch by 0.02.
- Pressing the left stick resets only the driver-centric heading reference.
  It never calls Pinpoint setHeading/resetPosAndIMU and does not alter the field pose.
- Every valid new corrected Limelight frame directly replaces Pinpoint X/Y/heading.
  It is automatic and has no operator confirmation button or weighted average.

The mechanism values copied from the supplied test include 1800 ticks/s initial
speed, 900 ticks/s idle, PIDF 9.6/0/0.65/12.5, 500 ms stability, 350 ticks/s
boost for 250 ms, limit positions 0.4/0.7, and 10 ms transit retraction.
The 500 ms timer is retained with a plus/minus 7.5 percent flywheel-speed
window. During continuous fire, feeding pauses below 92.5 percent of target
speed and resumes after recovery to 97.5 percent. The flywheel PIDF is
14.0/0.20/0.65/12.5.

After firing begins, a severe-drop controller can temporarily take over from
RUN_USING_ENCODER velocity PIDF. It enters when measured speed is at least
7.5 percent below the active target, switches the motors to direct power, and
ramps power proportionally from a 0.60 baseline to 1.00 at a 25 percent drop.
It exits after recovery to within 2.5 percent and restores RUN_USING_ENCODER
plus the configured PIDF coefficients. Telemetry reports VELOCITY PIDF or
POWER TAKEOVER with the applied power.

Rear aim enters aligned at 3.5 degrees and stays aligned until error exceeds
5.5 degrees. PID is P=0.35, I=0, D=0.01. Turn output scales to a 0.65 maximum
at large errors; inside 20 degrees it has a 0.12 minimum until alignment.

LB reverses intake only and leaves transit stopped. RB reverses both intake and
transit. These bumper overrides close the limit and take priority over trigger
feeding for as long as the bumper is held.

During firing, intake and transit feed power is scaled to 0.75 so balls enter
the flywheel more slowly. Normal LT intake and bumper reverse power are unchanged.

## Distance-adaptive shot map

The distance-based mechanism is normally called a shot lookup table, shot map,
or LUT. Each calibrated row contains field distance, shooter speed, and pitch
servo position. At runtime the robot computes distance from its field pose to
the selected Cell, finds the two surrounding rows, and linearly interpolates
both outputs. Values outside the measured range should clamp to the nearest
calibrated row rather than extrapolate. This competition code does not enable
that behavior until real calibration points are supplied.

In either Competition TeleOp, press A once to append the current point to
shot-calibration.csv in the Robot Controller settings directory. Each CSV row
contains timestamp, alliance, selected Cell, target distance in cm, preset
shooter speed in ticks/s, and pitch position. Holding A records only once per
press. Recording is rejected until a valid field pose and Cell are available.
Telemetry reports the result, number of distinct distance points, and full file
path. ShotLookupTable can load this file, clamp outside the measured range, and
linearly interpolate between points; automatic application remains disabled.

## Coordinates

The source project uses the 19725 flat-HIVE map frame:
- Origin at field center.
- +X toward audience; -X away from audience.
- +Y toward blue HIVE; -Y toward red HIVE.
- Heading 0 along +X, positive toward +Y (counterclockwise viewed from above).
- Public solver output uses inches; the camera-only telemetry displays cm.
- A nominal 144-inch square spans approximately -72 to +72 inches on X/Y.

This replaces the old, incorrect audience-edge-origin Y convention.
Actual map files were not verified against the camera in this change.
