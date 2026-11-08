package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Sequential blue-right route from the referenced conversation. */
@Disabled // Retained from original: remove after calibration and safe testing.
@Autonomous(name = "BIOBUZZ Blue Right Auto", group = "Autonomous")
public class BioBuzzAutonomous extends LinearOpMode {
    private BioBuzzRobot robot;
    private static final double IN_PER_M = 39.37007874;

    // ALL coordinates below are PROVISIONAL METERS from the prior conversation.
    // They are not surveyed field positions. Verify origin, axes, clearances,
    // intake/shooter alignment and total runtime before enabling this OpMode.
    // Heading: 0 faces +z; positive rotation turns toward +x (clockwise).
    private static final double START_X = 1.45, START_Z = -1.40;
    private static final double START_HEADING = 0;
    private static final double BALL_1_X = 1.20, BALL_1_Z = -1.00;
    private static final double BALL_2_X = 1.20, BALL_2_Z = -0.30;
    private static final double BALL_3_X = 1.48, BALL_3_Z = 0.00;
    // UNVERIFIED FOURTH BALL: duplicate placeholder, NOT a confirmed extra ball.
    // Replace with the actual fourth-ball waypoint before running the route.
    private static final double BALL_4_X = 1.48, BALL_4_Z = 0.00;
    private static final double SHOOT_X = 1.15, SHOOT_Z = -0.85;
    private static final double GOAL_X = 0.3241, GOAL_Z = 0.00;
    private static final long COLLECT_MS = 500;

    private double inches(double meters) { return meters * IN_PER_M; }

    private boolean goTo(double xMeters, double zMeters) {
        return robot.driveTo(inches(xMeters), inches(zMeters));
    }

    private void step(String message) {
        telemetry.addLine(message);
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        robot = new BioBuzzRobot(this);
        // Enclose initialization and waitForStart too, so every exit shuts down.
        try {
            robot.drivePower = 0.35;
            robot.strafePower = 0.30;
            robot.turnPower = 0.25;
            robot.intakePower = 0.55;
            robot.outakePower = 0.90;
            robot.shooterSpinupMs = 3000;
            robot.kickerRest = 0.20; // PROVISIONAL physical rest position.
            robot.kickerMovement = 0.30; // Relative push = 0.50.
            robot.kickerPushMs = 220;
            robot.kickerResetMs = 350;
            robot.init(hardwareMap);
            robot.setPose(inches(START_X), inches(START_Z), START_HEADING);

            telemetry.addLine("BIOBUZZ BLUE RIGHT: collect 3, shoot 3, collect 1, shoot 1");
            telemetry.addLine("PROVISIONAL route; fourth-ball waypoint UNVERIFIED.");
            telemetry.addLine("Ball counts and pose are estimates; no ball sensor.");
            telemetry.update();
            waitForStart();
            if (isStopRequested() || !opModeIsActive()) return;

            step("1: Drive to ball 1");
            if (!goTo(BALL_1_X, BALL_1_Z)) return;
            step("2: Collect estimated ball 1");
            if (!robot.collectOne(COLLECT_MS)) return;

            step("3: Drive to ball 2");
            if (!goTo(BALL_2_X, BALL_2_Z)) return;
            step("4: Collect estimated ball 2");
            if (!robot.collectOne(COLLECT_MS)) return;

            step("5: Drive to ball 3");
            if (!goTo(BALL_3_X, BALL_3_Z)) return;
            step("6: Collect estimated ball 3");
            if (!robot.collectOne(COLLECT_MS)) return;

            step("7: Drive to shooting position");
            if (!goTo(SHOOT_X, SHOOT_Z)) return;
            step("8: Aim at goal using estimated pose");
            if (!robot.facePoint(inches(GOAL_X), inches(GOAL_Z))) return;
            step("9: Spin wheels at 0.90 for 3000 ms, then attempt 3 shots");
            if (!robot.shootExpectedBalls()) return;

            step("10: Drive to UNVERIFIED fourth-ball waypoint");
            if (!goTo(BALL_4_X, BALL_4_Z)) return;
            step("11: Collect estimated ball 4");
            if (!robot.collectOne(COLLECT_MS)) return;

            step("12: Return to shooting position");
            if (!goTo(SHOOT_X, SHOOT_Z)) return;
            step("13: Aim at goal using estimated pose");
            if (!robot.facePoint(inches(GOAL_X), inches(GOAL_Z))) return;
            step("14: Spin wheels at 0.90 for 3000 ms, then attempt final shot");
            if (!robot.shootExpectedBalls()) return;
            step("15: Autonomous sequence complete (shots not sensor-confirmed)");
        } finally {
            robot.stopAll();
        }
    }
}
