package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled // Remove only after route, rules and mechanism validation.
@Autonomous(
    name = "BIOBUZZ Blue Right Autonomous",
    group = "Autonomous"
)
public class BioBuzzAutonomous extends LinearOpMode {

    public BioBuzzRobot robot;
    public static boolean ROUTE_CALIBRATED = false;

    // Unverified pickup positions: each row below is ONE expected ball.
    // An empty robot is required; no sensor confirms intake or discharge.
    // CAD uses meters.
    public static final double INCHES_PER_METER = 39.37007874;

    // Positive X = blue side.
    // Positive Z = audience side.
    // Starting-side interpretation must be confirmed
    // against the Driver Station orientation.

    // Proposed blue-right starting pose.
    public static double START_X_M = 1.45;
    public static double START_Z_M = -1.40;
    public static double START_HEADING = 0;

    // CAD landmark: blue NECTAR staging near
    // X = 1.87 m, Z = 0 m.
    // The points below are robot-center approach
    // positions, NOT exact game-piece centers.

    public static double[][] COLLECTION_ROUTE_M = {
        {1.20, -1.00},  // Proposed pickup 1: measure actual ball position
        {1.20, -0.30},  // Proposed pickup 2: measure actual ball position
        {1.48,  0.00}   // Proposed pickup 3: measure actual ball position
    };

    // Navigation ONLY: these two waypoints must not increment inventory.
    public static double[][] FINAL_APPROACH_M = {
        {1.20, -0.30},
        {1.48, 0.00}
    };
    // Proposed final pickup, not a CAD-confirmed ball position.
    public static double FINAL_BALL_X_M = 1.48;
    public static double FINAL_BALL_Z_M = 0.30;

    // Proposed shooting location, not an official
    // shooting coordinate.
    public static double[][] SHOOT_ROUTE_M = {
        {1.20, -0.30},
        {1.15, -0.85}
    };

    // Approximate blue HIVE pivot from CAD.
    public static double GOAL_X_M = 0.3241;
    public static double GOAL_Z_M = 0.0;

    private double inches(double meters) {
        return meters * INCHES_PER_METER;
    }

    private boolean goToMeters(double x, double z) {
        return robot.driveTo(inches(x), inches(z));
    }

    private boolean followRoute(
        double[][] route,
        String stage
    ) {
        robot.stopMechanisms();
        for (int i = 0; i < route.length; i++) {
            if (!opModeIsActive()) return false;

            telemetry.addData("Stage", stage);
            telemetry.addData("Waypoint", i + 1);
            telemetry.addData("Target X (m)", route[i][0]);
            telemetry.addData("Target Z (m)", route[i][1]);
            telemetry.update();

            if (!goToMeters(route[i][0], route[i][1])) {
                telemetry.addLine("Movement failed or timed out");
                telemetry.update();
                return false;
            }
        }
        return true;
    }

    private boolean shootAtGoal() {
        if (!followRoute(SHOOT_ROUTE_M, "Shooting position")) return false;
        if (!robot.facePoint(inches(GOAL_X_M), inches(GOAL_Z_M))) return false;
        return robot.shootExpectedBalls();
    }

    // Package-visible for the hardware-free regression check.
    boolean runSequence() {
        if (COLLECTION_ROUTE_M.length != 3) return false;
        for (double[] pickup : COLLECTION_ROUTE_M) {
            if (!robot.collectOneAt(inches(pickup[0]), inches(pickup[1]))) return false;
        }
        if (!shootAtGoal()) return false;
        // Restore the calibrated collection heading after aiming at the HIVE.
        if (!robot.faceHeading(START_HEADING)) return false;
        if (!followRoute(FINAL_APPROACH_M, "Final ball approach (no collection)")) return false;
        if (!robot.collectOneAt(inches(FINAL_BALL_X_M), inches(FINAL_BALL_Z_M))) return false;
        return shootAtGoal();
    }

    @Override
    public void runOpMode() {
        robot = new BioBuzzRobot(this);
        try {
            robot.init(hardwareMap);
            robot.setPose(inches(START_X_M), inches(START_Z_M), START_HEADING);
            telemetry.addLine("BIOBUZZ BLUE RIGHT - calibration draft");
            telemetry.addLine("Start EMPTY. Expected counts only; maximum planned capacity 4.");
            telemetry.addLine("endFlip shooting disabled; route/rules unverified.");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            if (!ROUTE_CALIBRATED) {
                telemetry.addLine("Route disabled: measure pickup positions and clearances first.");
                telemetry.update();
                return;
            }
            boolean completed = runSequence();
            telemetry.addData("Sequence", completed ? "Complete" : "Aborted / disabled");
            telemetry.addData("Expected balls (NOT sensed)", robot.getEstimatedBalls());
            telemetry.update();
        } finally {
            robot.stop();
        }
    }
}
