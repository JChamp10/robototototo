
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BIOBUZZ Collect and Position")
public class BioBuzzCollectAuto extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;

    // ===== ADJUSTABLE MOTOR POWER =====
    private static final double DRIVE_POWER = 0.45;
    private static final double STRAFE_POWER = 0.40;
    private static final double TURN_POWER = 0.30;

    // Individual motor strength multipliers
    private static final double LF_STRENGTH = 1.0;
    private static final double LB_STRENGTH = 1.0;
    private static final double RB_STRENGTH = 1.0;
    private static final double RF_STRENGTH = 1.0;

    // ===== ENCODER SETTINGS =====
    // Example values; replace with your drivetrain specs.
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER = 4.0;
    private static final double GEAR_RATIO = 1.0;

    private static final double TICKS_PER_INCH =
            TICKS_PER_REV * GEAR_RATIO /
            (Math.PI * WHEEL_DIAMETER);

    private static final double STRAFE_CORRECTION = 1.15;
    private static final double TURN_CORRECTION = 1.0;

    private static final double HALF_TRACK = 7.0;
    private static final double HALF_WHEELBASE = 7.0;

    // ===== ADJUSTABLE BALL ROUTE =====
    // Each waypoint is relative to the previous position.
    // Positive forward = drive forward
    // Positive strafe = move right

    private static final double[][] BALL_ROUTE = {
        {24, 0},     // Collection location 1
        {0, 18},     // Collection location 2
        {18, 0}      // Collection location 3
    };

    // Route from third collection location to shooting area
    private static final double SHOOT_FORWARD = -12;
    private static final double SHOOT_STRAFE = -18;
    private static final double SHOOT_TURN = 90;

    @Override
    public void runOpMode() {

        leftFront = hardwareMap.get(
                DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(
                DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(
                DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(
                DcMotor.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : motors()) {
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setTargetPosition(0);
            motor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("BIOBUZZ Route Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        try {

            // ===== COLLECT BALL POSITIONS =====
            for (int i = 0; i < BALL_ROUTE.length; i++) {

                if (!opModeIsActive()) break;

                telemetry.addData(
                        "Target Ball Location", i + 1);
                telemetry.update();

                double forward = BALL_ROUTE[i][0];
                double sideways = BALL_ROUTE[i][1];

                // Drive through collection location
                drive(forward, DRIVE_POWER);
                strafe(sideways, STRAFE_POWER);

                // Brief pause at location
                pause(150);
            }

            // ===== GO TO SHOOTING POSITION =====
            if (opModeIsActive()) {

                telemetry.addLine(
                        "Moving to shooting position");
                telemetry.update();

                drive(SHOOT_FORWARD, DRIVE_POWER);
                strafe(SHOOT_STRAFE, STRAFE_POWER);

                // Face goal
                turn(SHOOT_TURN, TURN_POWER);

                stopMotors();
            }

        } finally {
            stopMotors();
        }

        telemetry.addLine(
                "At Shooting Position - Autonomous Complete");
        telemetry.update();
    }

    // ===== DRIVE MOVEMENT =====

    private void drive(double inches, double power) {
        int ticks = (int) Math.round(
                inches * TICKS_PER_INCH);

        moveRelative(
                ticks, ticks, ticks, ticks, power);
    }

    // Positive = right, negative = left
    private void strafe(double inches, double power) {
        int ticks = (int) Math.round(
                inches * TICKS_PER_INCH
                * STRAFE_CORRECTION);

        moveRelative(
                ticks, -ticks, -ticks, ticks, power);
    }

    // Positive = clockwise, negative = counterclockwise
    private void turn(double degrees, double power) {
        double arc = Math.toRadians(degrees) *
                (HALF_TRACK + HALF_WHEELBASE);

        int ticks = (int) Math.round(
                arc * TICKS_PER_INCH * TURN_CORRECTION);

        moveRelative(
                ticks, ticks, -ticks, -ticks, power);
    }

    // ===== MOTOR CONTROLLER =====

    private void moveRelative(
            int lf, int lb, int rb, int rf,
            double power) {

        if (!opModeIsActive()) return;

        DcMotor[] motors = motors();
        int[] offsets = {lf, lb, rb, rf};

        double[] strengths = {
                LF_STRENGTH,
                LB_STRENGTH,
                RB_STRENGTH,
                RF_STRENGTH
        };

        for (int i = 0; i < 4; i++) {
            motors[i].setTargetPosition(
                    motors[i].getCurrentPosition()
                    + offsets[i]);

            motors[i].setMode(
                    DcMotor.RunMode.RUN_TO_POSITION);
        }

        double p = Math.max(
                0, Math.min(1, Math.abs(power)));

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(
                    p * strengths[i]);
        }

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && timer.seconds() < 5.0) {

            boolean busy = false;

            for (DcMotor motor : motors) {
                if (motor.isBusy()) busy = true;
            }

            if (!busy) break;

            telemetry.addData(
                    "Movement time", "%.2f",
                    timer.seconds());
            telemetry.update();

            idle();
        }

        stopMotors();

        for (DcMotor motor : motors) {
            motor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private DcMotor[] motors() {
        return new DcMotor[]{
                leftFront,
                leftBack,
                rightBack,
                rightFront
        };
    }

    private void stopMotors() {
        for (DcMotor motor : motors()) {
            motor.setPower(0);
        }
    }

    private void pause(long milliseconds) {
        if (opModeIsActive()) {
            sleep(milliseconds);
        }
    }
}
