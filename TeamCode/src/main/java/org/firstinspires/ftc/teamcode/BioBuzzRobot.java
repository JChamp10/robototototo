package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class BioBuzzRobot {

    public DcMotor leftFront, leftBack;
    public DcMotor rightBack, rightFront;
    public DcMotor intake, endFlip;
    public static final int MAX_BALLS = 4;
    private int estimatedBalls;
    public double intakePower = 0.55;
    public long intakeSettleMs = 250;

    public double leftFrontStrength = 1.0;
    public double leftBackStrength = 1.0;
    public double rightBackStrength = 1.0;
    public double rightFrontStrength = 1.0;

    public double drivePower = 0.45;
    public double strafePower = 0.35;
    public double turnPower = 0.30;

    // Replace with measured robot specifications.
    public double ticksPerRev = 537.7;
    public double wheelDiameterIn = 4.0;
    public double gearRatio = 1.0;
    public double strafeCorrection = 1.15;
    public double turnCorrection = 1.0;
    public double turnRadiusIn = 14.0;

    // Field pose in inches:
    // X points toward blue, Z points toward audience.
    // Heading 0 = +Z; heading 90 = +X.
    public double x, z, heading;

    private final LinearOpMode op;

    public BioBuzzRobot(LinearOpMode op) {
        this.op = op;
    }

    public void init(HardwareMap map) {
        leftFront = map.get(DcMotor.class, "leftFront");
        leftBack = map.get(DcMotor.class, "leftBack");
        rightBack = map.get(DcMotor.class, "rightBack");
        rightFront = map.get(DcMotor.class, "rightFront");

        intake = map.get(DcMotor.class, "intake");
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        endFlip = map.get(DcMotor.class, "endFlip");
        endFlip.setPower(0); // Unknown mechanism: never energize or reset its encoder.
        estimatedBalls = 0; // Requires an EMPTY robot at start.

        // Verify directions with the actual drivetrain.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : motors()) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
            );
            motor.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
            );
            motor.setTargetPosition(0);
            motor.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
            );
        }
    }

    public DcMotor[] motors() {
        return new DcMotor[]{
            leftFront, leftBack, rightBack, rightFront
        };
    }

    public void setPose(double x, double z, double heading) {
        this.x = x;
        this.z = z;
        this.heading = normalize(heading);
    }

    public double ticksPerInch() {
        return ticksPerRev * gearRatio /
            (Math.PI * wheelDiameterIn);
    }

    public void stopDrive() {
        for (DcMotor motor : motors()) {
            if (motor != null) motor.setPower(0);
        }
    }

    public void stopMechanisms() {
        if (intake != null) intake.setPower(0);
        if (endFlip != null) endFlip.setPower(0);
    }

    public void stop() {
        stopDrive();
        stopMechanisms();
    }

    public int getEstimatedBalls() {
        return estimatedBalls;
    }

    // One completed pickup pass = ONE expected ball, never sensor confirmation.
    // A failed pass may still pick up a ball: abort the routine, do not retry.
    public boolean collectOneAt(double targetX, double targetZ) {
        if (!op.opModeIsActive() || estimatedBalls >= MAX_BALLS
                || intakeSettleMs < 0 || intakeSettleMs > 2000
                || !Double.isFinite(intakePower) || intakePower <= 0 || intakePower > 1) {
            stopMechanisms();
            return false;
        }
        try {
            intake.setPower(intakePower);
            if (!driveTo(targetX, targetZ)) return false;
            ElapsedTime timer = new ElapsedTime();
            while (op.opModeIsActive() && timer.milliseconds() < intakeSettleMs) {
                op.idle();
            }
            if (!op.opModeIsActive()) return false;
            estimatedBalls++;
            op.telemetry.addData("Expected balls (NOT sensed)", estimatedBalls);
            op.telemetry.update();
            return true;
        } finally {
            stopMechanisms();
        }
    }

    public boolean shootExpectedBalls() {
        stop();
        // Deliberately disabled. endFlip may be a flipper/catapult, not a flywheel.
        // Replace with a measured, bounded, STOP-aware mechanism cycle.
        // Only reduce estimatedBalls after successful discharge; on failure retain it.
        op.telemetry.addLine("SHOOTING DISABLED: identify and calibrate endFlip");
        op.telemetry.update();
        return false; // Never claim a shot or continue collecting after this placeholder.
    }

    private double normalize(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) angle += 360;
        return angle - 180;
    }

    public boolean moveTicks(
        int lf, int lb, int rb, int rf,
        double power, double timeout
    ) {
        if (!op.opModeIsActive()) return false;

        DcMotor[] m = motors();
        int[] delta = {lf, lb, rb, rf};

        double[] strengths = {
            leftFrontStrength, leftBackStrength,
            rightBackStrength, rightFrontStrength
        };

        int[] targets = new int[4];
        double p = Math.max(0, Math.min(1, Math.abs(power)));

        for (int i = 0; i < 4; i++) {
            targets[i] = m[i].getCurrentPosition() + delta[i];
            m[i].setTargetPosition(targets[i]);
            m[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        for (int i = 0; i < 4; i++) {
            m[i].setPower(Math.max(0,
                Math.min(1, p * strengths[i])));
        }

        ElapsedTime timer = new ElapsedTime();
        boolean completed = false;

        try {
            while (op.opModeIsActive()
                    && timer.seconds() < timeout) {

                boolean busy = false;
                for (DcMotor motor : m) {
                    if (motor.isBusy()) busy = true;
                }

                if (!busy) {
                    completed = true;
                    break;
                }

                op.telemetry.addData("Move time",
                    "%.2f", timer.seconds());
                op.telemetry.update();
                op.idle();
            }
        } finally {
            stopDrive();
            for (DcMotor motor : m) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        return completed;
    }

    public boolean drive(double inches) {
        int ticks = (int)Math.round(
            inches * ticksPerInch()
        );

        boolean ok = moveTicks(
            ticks, ticks, ticks, ticks, drivePower, 6.0
        );

        if (ok) {
            double a = Math.toRadians(heading);
            x += inches * Math.sin(a);
            z += inches * Math.cos(a);
        }
        return ok;
    }

    public boolean strafe(double inches) {
        int ticks = (int)Math.round(
            inches * ticksPerInch() * strafeCorrection
        );

        boolean ok = moveTicks(
            ticks, -ticks, ticks, -ticks,
            strafePower, 6.0
        );

        if (ok) {
            double a = Math.toRadians(heading);
            x += inches * Math.cos(a);
            z -= inches * Math.sin(a);
        }
        return ok;
    }

    public boolean turn(double degrees) {
        double arcIn = Math.toRadians(degrees)
            * turnRadiusIn;

        int ticks = (int)Math.round(
            arcIn * ticksPerInch() * turnCorrection
        );

        boolean ok = moveTicks(
            ticks, ticks, -ticks, -ticks,
            turnPower, 5.0
        );

        if (ok) heading = normalize(heading + degrees);
        return ok;
    }

    public boolean faceHeading(double degrees) {
        double difference = normalize(degrees - heading);
        if (Math.abs(difference) < 1.0) return true;
        return turn(difference);
    }

    public boolean facePoint(double targetX, double targetZ) {
        double dx = targetX - x;
        double dz = targetZ - z;

        if (Math.hypot(dx, dz) < 0.5) return true;

        return faceHeading(
            Math.toDegrees(Math.atan2(dx, dz))
        );
    }

    // Uses robot-relative strafe followed by forward travel.
    public boolean driveTo(double targetX, double targetZ) {
        double dx = targetX - x;
        double dz = targetZ - z;
        double a = Math.toRadians(heading);

        double sideways = dx * Math.cos(a)
                         - dz * Math.sin(a);

        double forward = dx * Math.sin(a)
                       + dz * Math.cos(a);

        if (Math.abs(sideways) > 0.25) {
            if (!strafe(sideways)) return false;
        }

        if (Math.abs(forward) > 0.25) {
            if (!drive(forward)) return false;
        }

        return true;
    }
}
