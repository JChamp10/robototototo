
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BIOBUZZ Full Autonomous", group = "Autonomous")
public class BioBuzzAutonomous extends LinearOpMode {

    // ===== DRIVETRAIN MOTORS =====
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor rightFront;

    // ===== ADJUSTABLE SETTINGS =====

    private static final double DRIVE_POWER = 0.50;
    private static final double STRAFE_POWER = 0.45;
    private static final double TURN_POWER = 0.35;

    // Individual motor power multipliers
    private static final double LF_STRENGTH = 1.0;
    private static final double LB_STRENGTH = 1.0;
    private static final double RB_STRENGTH = 1.0;
    private static final double RF_STRENGTH = 1.0;

    // Example motor/encoder specifications.
    // Replace with your actual motor and gearing values.
    private static final double TICKS_PER_REV = 537.7;
    private static final double WHEEL_DIAMETER_IN = 4.0;
    private static final double GEAR_RATIO = 1.0;

    // Calibrate using measured movement.
    private static final double STRAFE_MULTIPLIER = 1.15;
    private static final double TURN_MULTIPLIER = 1.0;

    // Approximate distance from robot center to wheel
    // along each axis, in inches.
    private static final double HALF_TRACK_WIDTH = 7.0;
    private static final double HALF_WHEELBASE = 7.0;

    private static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) /
            (Math.PI * WHEEL_DIAMETER_IN);

    // ===== INITIALIZATION =====

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

        // Motor direction settings
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Brake when stopped
        for (DcMotor motor : motors()) {
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE);

            motor.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            motor.setTargetPosition(0);

            motor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addLine("BIOBUZZ Autonomous Ready");
        telemetry.addData("Ticks/Inch", TICKS_PER_INCH);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        try {

            // =================================
            // COMPLETE EXAMPLE AUTONOMOUS ROUTE
            // =================================

            // Step 1: Drive forward 24 inches
            drive(24, DRIVE_POWER);

            pause(200);

            // Step 2: Strafe right 18 inches
            strafe(18, STRAFE_POWER);

            pause(200);

            // Step 3: Turn right 90 degrees
            turn(90, TURN_POWER);

            pause(200);

            // Step 4: Drive forward 12 inches
            drive(12, DRIVE_POWER);

            pause(200);

            // Step 5: Strafe left 8 inches
            strafe(-8, STRAFE_POWER);

            pause(200);

            // Step 6: Move backward 6 inches
            drive(-6, DRIVE_POWER);

        } finally {
            stopMotors();
        }

        telemetry.addLine("Autonomous Complete");
        telemetry.update();
    }

    // ===== MOVEMENT FUNCTIONS =====

    // Positive distance = forward
    // Negative distance = backward
    private void drive(double inches, double power) {

        int ticks = (int) Math.round(
                inches * TICKS_PER_INCH);

        moveRelative(
                ticks,
                ticks,
                ticks,
                ticks,
                power,
                5.0
        );
    }

    // Positive = right, negative = left
    private void strafe(double inches, double power) {

        int ticks = (int) Math.round(
                inches * TICKS_PER_INCH
                * STRAFE_MULTIPLIER);

        moveRelative(
                ticks,
                -ticks,
                -ticks,
                ticks,
                power,
                5.0
        );
    }

    // Positive = clockwise/right
    // Negative = counterclockwise/left
    private void turn(double degrees, double power) {

        double radians = Math.toRadians(degrees);

        double arcDistance =
                radians *
                (HALF_TRACK_WIDTH + HALF_WHEELBASE);

        int ticks = (int) Math.round(
                arcDistance *
                TICKS_PER_INCH *
                TURN_MULTIPLIER);

        moveRelative(
                ticks,
                ticks,
                -ticks,
                -ticks,
                power,
                5.0
        );
    }

    // ===== ENCODER MOVEMENT =====

    private void moveRelative(
            int lfTicks,
            int lbTicks,
            int rbTicks,
            int rfTicks,
            double power,
            double timeoutSeconds) {

        if (!opModeIsActive()) {
            return;
        }

        DcMotor[] motors = motors();

        int[] deltas = {
                lfTicks,
                lbTicks,
                rbTicks,
                rfTicks
        };

        double[] strengths = {
                LF_STRENGTH,
                LB_STRENGTH,
                RB_STRENGTH,
                RF_STRENGTH
        };

        int[] targets = new int[4];

        for (int i = 0; i < 4; i++) {

            targets[i] =
                    motors[i].getCurrentPosition()
                    + deltas[i];

            motors[i].setTargetPosition(targets[i]);

            motors[i].setMode(
                    DcMotor.RunMode.RUN_TO_POSITION);
        }

        double basePower = Math.max(
                0.0,
                Math.min(1.0, Math.abs(power)));

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(
                    Math.max(0.0,
                    Math.min(1.0,
                    basePower * strengths[i])));
        }

        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive()
                && timer.seconds() < timeoutSeconds) {

            boolean anyBusy = false;

            for (DcMotor motor : motors) {
                if (motor.isBusy()) {
                    anyBusy = true;
                }
            }

            if (!anyBusy) {
                break;
            }

            telemetry.addData(
                    "Elapsed", "%.2f s", timer.seconds());

            telemetry.addData(
                    "LF", "%d / %d",
                    leftFront.getCurrentPosition(),
                    targets[0]);

            telemetry.addData(
                    "LB", "%d / %d",
                    leftBack.getCurrentPosition(),
                    targets[1]);

            telemetry.addData(
                    "RB", "%d / %d",
                    rightBack.getCurrentPosition(),
                    targets[2]);

            telemetry.addData(
                    "RF", "%d / %d",
                    rightFront.getCurrentPosition(),
                    targets[3]);

            telemetry.update();

            idle();
        }

        stopMotors();

        for (DcMotor motor : motors) {
            motor.setMode(
                    DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // ===== HELPERS =====

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
