package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Encoder-based estimates only: no ball sensor, IMU, or field localization. */
public class BioBuzzRobot {
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public DcMotor intake, leftOutake, rightOutake;
    public Servo endFlip;
    private final LinearOpMode op;

    // PROVISIONAL calibration: verify gearing, wheel size, directions and slip.
    public double ticksPerRev = 537.7;
    public double wheelDiameterIn = 4.0;
    public double strafeCorrection = 1.15;
    public double turnRadiusIn = 14.0;
    public double turnCorrection = 1.0;
    public double drivePower = 0.35, strafePower = 0.30, turnPower = 0.25;
    public double intakePower = 0.55;
    public double outakePower = 0.90;
    public long shooterSpinupMs = 3000;
    // PROVISIONAL servo rest; push is relative: 0.20 + 0.30 = 0.50.
    public double kickerRest = 0.20, kickerMovement = 0.30;
    public long kickerPushMs = 220, kickerResetMs = 350;
    public static final int MAX_BALLS = 4;
    private int estimatedBalls;

    // Inches; heading degrees clockwise from +z toward +x.
    // Commanded pose estimate, NOT measured localization.
    public double x, z, heading;

    public BioBuzzRobot(LinearOpMode op) { this.op = op; }

    public void init(HardwareMap map) {
        leftFront = map.get(DcMotor.class, "leftFront");
        leftBack = map.get(DcMotor.class, "leftBack");
        rightFront = map.get(DcMotor.class, "rightFront");
        rightBack = map.get(DcMotor.class, "rightBack");
        intake = map.get(DcMotor.class, "intake");
        leftOutake = map.get(DcMotor.class, "leftOutake");
        rightOutake = map.get(DcMotor.class, "rightOutake");
        endFlip = map.get(Servo.class, "endFlip");
        stopAll();
        // Assumes conventional mecanum mounting. Verify on your robot.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        leftOutake.setDirection(DcMotor.Direction.FORWARD);
        rightOutake.setDirection(DcMotor.Direction.REVERSE);
        for (DcMotor motor : driveMotors()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        for (DcMotor motor : new DcMotor[] {intake, leftOutake, rightOutake}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        estimatedBalls = 0;
    }

    // Preserve the original public moveTicks parameter order: LF, LB, RB, RF.
    private DcMotor[] driveMotors() {
        return new DcMotor[] {leftFront, leftBack, rightBack, rightFront};
    }

    private boolean active() {
        return !Thread.currentThread().isInterrupted()
                && !op.isStopRequested() && op.opModeIsActive();
    }

    /** Wait in short intervals so STOP aborts collection, spin-up and kicks. */
    private boolean waitActive(long milliseconds) {
        ElapsedTime timer = new ElapsedTime();
        while (active() && timer.milliseconds() < milliseconds) {
            op.sleep(10);
        }
        return active();
    }

    public void stopDrive() {
        for (DcMotor motor : driveMotors()) {
            if (motor != null) motor.setPower(0);
        }
    }

    /** Null-safe even when hardware initialization only partly succeeded. */
    public void stopAll() {
        // Attempt every shutdown even if one hardware command fails.
        RuntimeException failure = null;
        for (DcMotor motor : new DcMotor[] {leftFront, leftBack, rightFront,
                rightBack, intake, leftOutake, rightOutake}) {
            if (motor == null) continue;
            try { motor.setPower(0); }
            catch (RuntimeException e) { if (failure == null) failure = e; }
        }
        try { if (endFlip != null) endFlip.setPosition(kickerRest); }
        catch (RuntimeException e) { if (failure == null) failure = e; }
        if (failure != null) throw failure;
    }

    public void setPose(double startX, double startZ, double startHeading) {
        x = startX;
        z = startZ;
        heading = normalize(startHeading);
    }

    private double normalize(double angle) {
        angle = (angle + 180) % 360;
        if (angle < 0) angle += 360;
        return angle - 180;
    }

    private double ticksPerInch() {
        return ticksPerRev / (Math.PI * wheelDiameterIn);
    }

    public boolean moveTicks(int lf, int lb, int rb, int rf,
                             double power, double timeoutSeconds) {
        if (!active()) return false;
        DcMotor[] motors = driveMotors();
        int[] delta = {lf, lb, rb, rf};
        try {
            for (int i = 0; i < motors.length; i++) {
                motors[i].setTargetPosition(motors[i].getCurrentPosition() + delta[i]);
                motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (!active()) return false;
            for (DcMotor motor : motors) {
                motor.setPower(Math.min(1.0, Math.abs(power)));
            }
            ElapsedTime timer = new ElapsedTime();
            while (active() && timer.seconds() < timeoutSeconds) {
                boolean busy = false;
                for (DcMotor motor : motors) busy |= motor.isBusy();
                if (!busy) return active();
                op.idle();
            }
            op.telemetry.addLine("Move aborted: STOP or encoder timeout; route ends.");
            op.telemetry.update();
            return false;
        } finally {
            stopDrive();
            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public boolean drive(double inches) {
        int ticks = (int) Math.round(inches * ticksPerInch());
        if (!moveTicks(ticks, ticks, ticks, ticks, drivePower, 8.0)) return false;
        double a = Math.toRadians(heading);
        x += inches * Math.sin(a);
        z += inches * Math.cos(a);
        return true;
    }

    public boolean strafe(double inches) {
        int ticks = (int) Math.round(inches * ticksPerInch() * strafeCorrection);
        // Right strafe: LF+, LB-, RB+, RF- (corrected from previous source).
        if (!moveTicks(ticks, -ticks, ticks, -ticks, strafePower, 8.0)) return false;
        double a = Math.toRadians(heading);
        x += inches * Math.cos(a);
        z -= inches * Math.sin(a);
        return true;
    }

    public boolean turn(double degrees) {
        int ticks = (int) Math.round(Math.toRadians(degrees) * turnRadiusIn
                * ticksPerInch() * turnCorrection);
        if (!moveTicks(ticks, ticks, -ticks, -ticks, turnPower, 6.0)) return false;
        heading = normalize(heading + degrees);
        return true;
    }

    public boolean facePoint(double targetX, double targetZ) {
        if (!active()) return false;
        double dx = targetX - x, dz = targetZ - z;
        if (Math.hypot(dx, dz) < 0.5) return true;
        return turn(normalize(Math.toDegrees(Math.atan2(dx, dz)) - heading));
    }

    public boolean driveTo(double targetX, double targetZ) {
        if (!active()) return false;
        double dx = targetX - x, dz = targetZ - z;
        double a = Math.toRadians(heading);
        double sideways = dx * Math.cos(a) - dz * Math.sin(a);
        double forward = dx * Math.sin(a) + dz * Math.cos(a);
        if (Math.abs(sideways) > 0.25 && !strafe(sideways)) return false;
        if (Math.abs(forward) > 0.25 && !drive(forward)) return false;
        return active();
    }

    /** Timed intake attempt; increment is an estimate, never sensor-confirmed. */
    public boolean collectOne(long durationMs) {
        if (!active() || estimatedBalls >= MAX_BALLS) return false;
        try {
            intake.setPower(intakePower);
            if (!waitActive(durationMs)) return false;
            estimatedBalls++;
            op.telemetry.addData("Estimated balls (no sensor)", estimatedBalls);
            op.telemetry.update();
            return true;
        } finally {
            intake.setPower(0);
        }
    }

    public double kickerPushPosition() {
        return Math.max(0.0, Math.min(1.0, kickerRest + kickerMovement));
    }

    private boolean kickOne() {
        if (!active()) return false;
        try {
            endFlip.setPosition(kickerPushPosition());
            if (!waitActive(kickerPushMs)) return false;
        } finally {
            // Retract even if STOP arrives while the kicker is extended.
            endFlip.setPosition(kickerRest);
        }
        return waitActive(kickerResetMs);
    }

    public boolean shootExpectedBalls() {
        if (!active() || estimatedBalls <= 0) return false;
        int shots = estimatedBalls;
        try {
            endFlip.setPosition(kickerRest);
            leftOutake.setPower(outakePower);
            rightOutake.setPower(outakePower);
            // Each batch gets a full spin-up; wheels stay on between kicks.
            if (!waitActive(shooterSpinupMs)) return false;
            for (int i = 0; i < shots; i++) {
                if (!kickOne()) return false;
                estimatedBalls--;
                op.telemetry.addData("Attempted shot in batch", i + 1);
                op.telemetry.addData("Estimated remaining (no sensor)", estimatedBalls);
                op.telemetry.update();
            }
            return active();
        } finally {
            stopAll();
        }
    }
}
