package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(
    name = "BIOBUZZ Blue Right Autonomous",
    group = "Autonomous"
)
public class BioBuzzAutonomous extends LinearOpMode {

    public BioBuzzRobot robot;

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
        {1.20, -1.00},  // Approach corridor
        {1.20, -0.30},  // Move along blue side
        {1.48,  0.00}   // Approach blue NECTAR staging
    };

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

    @Override
    public void runOpMode() {

        robot = new BioBuzzRobot(this);
        robot.init(hardwareMap);

        // ===== ADJUSTABLE MOTOR STRENGTH =====
        robot.leftFrontStrength = 1.0;
        robot.leftBackStrength = 1.0;
        robot.rightBackStrength = 1.0;
        robot.rightFrontStrength = 1.0;

        // ===== ADJUSTABLE MOTOR POWER =====
        robot.drivePower = 0.45;
        robot.strafePower = 0.35;
        robot.turnPower = 0.30;

        // ===== HARDWARE CALIBRATION =====
        // Example motor specifications.
        robot.ticksPerRev = 537.7;
        robot.wheelDiameterIn = 4.0;
        robot.gearRatio = 1.0;

        robot.strafeCorrection = 1.15;
        robot.turnCorrection = 1.0;
        robot.turnRadiusIn = 14.0;

        // ===== SET INITIAL POSITION =====
        robot.setPose(
            inches(START_X_M),
            inches(START_Z_M),
            START_HEADING
        );

        telemetry.addLine("BIOBUZZ BLUE RIGHT");
        telemetry.addLine("Preset CAD-based route loaded");
        telemetry.addLine("Drive motors only");
        telemetry.addLine("Verify start pose and clearance");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        try {
            // STEP 1: MOVE TO COLLECTION AREA
            if (!followRoute(
                    COLLECTION_ROUTE_M,
                    "Blue Collection Approach")) {
                return;
            }

            // No intake motor is configured.
            // This only drives to the staging area.

            // STEP 2: MOVE TO SHOOTING POSITION
            if (!followRoute(
                    SHOOT_ROUTE_M,
                    "Shooting Position")) {
                return;
            }

            // STEP 3: FACE BLUE HIVE
            if (!robot.facePoint(
                    inches(GOAL_X_M),
                    inches(GOAL_Z_M))) {
                telemetry.addLine("Alignment failed");
                telemetry.update();
                return;
            }

            robot.stop();

            telemetry.addLine("Autonomous route complete");
            telemetry.addData("Estimated X (in)", robot.x);
            telemetry.addData("Estimated Z (in)", robot.z);
            telemetry.addData("Estimated heading", robot.heading);
            telemetry.update();

        } finally {
            robot.stop();
        }
    }
}
