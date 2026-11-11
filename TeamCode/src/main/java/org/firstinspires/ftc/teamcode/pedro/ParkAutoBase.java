package org.firstinspires.ftc.teamcode.pedro;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Shared code for "Red Park Auto" and "Blue Park Auto". No shooting: the robot
 * just drives from its starting spot to its alliance's parking corner.
 *
 * ------------------------------------------------------------------------
 * FIELD COORDINATES (inches), as seen by someone standing in the AUDIENCE:
 *   X = 0   is the RED alliance wall (left side)
 *   X = 144 is the BLUE alliance wall (right side)
 *   Y = 0   is the audience wall (nearest the audience)
 *   Y = 144 is the far wall
 * Heading 0 = facing toward the blue wall (+X). Heading 180 = facing red wall.
 * The field is a 6 x 6 grid of 24 in tiles, so tile seams are at 24, 48, 72...
 * ------------------------------------------------------------------------
 *
 * THE PLAN (written for RED; Blue is the mirror image):
 *   START: robot's BACK touching the red alliance wall, centered on the middle
 *          seam of that wall (Y = 72, between the 3rd and 4th tile from the
 *          audience), FRONT facing into the field.
 *   LEG 1: drive straight forward until the back is WALL_GAP in off the wall.
 *   LEG 2: strafe along the wall to the parking corner, stopping WALL_GAP in
 *          short of the end wall. The robot keeps facing into the field.
 *
 * SAFETY (distance sensors): every loop we look at the sensors that point in
 * the direction the robot is moving. If one reads less than SAFE_DISTANCE_IN
 * the robot stops and holds its spot. When the reading stays clear for
 * CLEAR_TIME_S, it starts again from wherever it is. The robot has no FRONT
 * sensor, so the plan never drives forward toward anything.
 */
public abstract class ParkAutoBase extends OpMode {

    // ---------------- Things you may want to change ----------------

    /** Park at the corner nearest the AUDIENCE (true) or the far corner (false). */
    public static boolean PARK_AT_AUDIENCE_END = true;

    /** Half of the robot's front-to-back length, in inches. 18 in robot -> 9. */
    public static double ROBOT_HALF_LENGTH = 9.0;
    /** Half of the robot's side-to-side width, in inches. */
    public static double ROBOT_HALF_WIDTH = 9.0;

    /** Stop if anything gets closer than this (inches). */
    public static double SAFE_DISTANCE_IN = 6.0;
    /** How far off the walls we park (inches). A bit more than SAFE_DISTANCE_IN
     *  so the sensors don't stop the robot right at the end. */
    public static double WALL_GAP = 7.0;
    /** How long a blocked sensor must read clear before driving again (seconds). */
    public static double CLEAR_TIME_S = 0.5;

    /** Robot configuration names. Match these in the Driver Station config. */
    public static String BACK_SENSOR = "distBack";
    public static String LEFT_SENSOR = "distLeft";
    public static String RIGHT_SENSOR = "distRight";

    // ---------------- Alliance-specific part ----------------

    /** true for the Red OpMode, false for Blue. */
    protected abstract boolean isRed();

    // ---------------- Internals ----------------

    private static final double FIELD = 144.0;

    private enum State { LEG1, LEG2, PARKED }

    private State state = State.LEG1;
    private Follower follower;

    private Pose startPose, leg1End, leg2End;
    private Pose currentTarget;

    private DistanceSensor distBack, distLeft, distRight;
    private boolean blocked = false;
    private String blockedBy = "";
    private final ElapsedTime clearTimer = new ElapsedTime();

    /** Builds a pose using RED coordinates and mirrors it for Blue. */
    private Pose redPose(double x, double y, double headingDeg) {
        if (isRed()) return new Pose(x, y, Math.toRadians(headingDeg));
        return new Pose(FIELD - x, y, Math.toRadians(180 - headingDeg));
    }

    private void buildPoses() {
        double parkY = PARK_AT_AUDIENCE_END
                ? WALL_GAP + ROBOT_HALF_WIDTH
                : FIELD - WALL_GAP - ROBOT_HALF_WIDTH;
        double offWallX = WALL_GAP + ROBOT_HALF_LENGTH;

        startPose = redPose(ROBOT_HALF_LENGTH, 72, 0);
        leg1End   = redPose(offWallX, 72, 0);
        leg2End   = redPose(offWallX, parkY, 0);
    }

    private DistanceSensor getSensor(String name) {
        DistanceSensor s = hardwareMap.tryGet(DistanceSensor.class, name);
        if (s == null) telemetry.addLine("WARNING: no distance sensor named \"" + name + "\" (safety off for that side)");
        return s;
    }

    /** Reads a sensor in inches; returns a huge number if missing or out of range. */
    private double read(DistanceSensor s) {
        if (s == null) return Double.MAX_VALUE;
        double d = s.getDistance(DistanceUnit.INCH);
        if (Double.isNaN(d) || d <= 0) return Double.MAX_VALUE;
        return d;
    }

    /** Starts (or restarts) a straight line from where the robot is now to the target. */
    private void driveTo(Pose target) {
        currentTarget = target;
        Pose here = follower.pose();
        follower.follow(line(here, target).constant(target.heading()));
    }

    /**
     * Checks the sensors that face the way we are going.
     * Returns the name of the sensor that is too close, or null if all clear.
     */
    private String checkObstacles() {
        Pose here = follower.pose();
        double dx = currentTarget.x() - here.x();
        double dy = currentTarget.y() - here.y();
        double len = Math.hypot(dx, dy);
        if (len < 1.0) return null; // basically there

        // Turn the field direction of travel into robot directions
        // (robot +X = forward, robot +Y = left).
        double h = here.heading();
        double fwd  = ( dx * Math.cos(h) + dy * Math.sin(h)) / len;
        double left = (-dx * Math.sin(h) + dy * Math.cos(h)) / len;

        // A sensor "watches" if we are moving mostly toward the side it faces.
        if (-fwd > 0.5 && read(distBack) < SAFE_DISTANCE_IN) return BACK_SENSOR;
        if (left > 0.5 && read(distLeft) < SAFE_DISTANCE_IN) return LEFT_SENSOR;
        if (-left > 0.5 && read(distRight) < SAFE_DISTANCE_IN) return RIGHT_SENSOR;
        return null;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        distBack = getSensor(BACK_SENSOR);
        distLeft = getSensor(LEFT_SENSOR);
        distRight = getSensor(RIGHT_SENSOR);
        buildPoses();
        follower.setPose(startPose);

        telemetry.addLine(isRed() ? "RED PARK AUTO" : "BLUE PARK AUTO");
        telemetry.addLine("Back of robot touching the " + (isRed() ? "RED" : "BLUE")
                + " alliance wall, centered on the middle tile seam, facing into the field.");
        telemetry.addLine("Parks in the " + (PARK_AT_AUDIENCE_END ? "audience" : "far")
                + " corner on our alliance wall.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Back (in)", "%.1f", read(distBack));
        telemetry.addData("Left (in)", "%.1f", read(distLeft));
        telemetry.addData("Right (in)", "%.1f", read(distRight));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(startPose);
        state = State.LEG1;
        driveTo(leg1End);
    }

    @Override
    public void loop() {
        follower.update();

        if (state != State.PARKED) {
            String tooClose = checkObstacles();
            if (tooClose != null) {
                if (!blocked) {
                    // Something is within 6 in in the direction we're moving: stop here.
                    blocked = true;
                    follower.hold(follower.pose());
                }
                blockedBy = tooClose;
                clearTimer.reset();
            } else if (blocked && clearTimer.seconds() > CLEAR_TIME_S) {
                // Path has been clear long enough: keep going.
                blocked = false;
                blockedBy = "";
                driveTo(currentTarget);
            }
        }

        if (!blocked) {
            switch (state) {
                case LEG1:
                    if (follower.atParametricEnd()) {
                        state = State.LEG2;
                        driveTo(leg2End);
                    }
                    break;
                case LEG2:
                    if (follower.atParametricEnd()) {
                        state = State.PARKED;
                    }
                    break;
                case PARKED:
                    // Pedro keeps holding the final spot until Stop.
                    break;
            }
        }

        Pose pose = follower.pose();
        telemetry.addData("State", blocked ? "STOPPED: " + blockedBy + " < " + SAFE_DISTANCE_IN + " in" : state);
        telemetry.addData("X / Y (in)", "%.1f / %.1f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Back / Left / Right (in)", "%.1f / %.1f / %.1f",
                Math.min(read(distBack), 999), Math.min(read(distLeft), 999), Math.min(read(distRight), 999));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (follower != null) follower.stop();
    }
}
