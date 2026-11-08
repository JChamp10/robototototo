package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/** Four-wheel mecanum driving. Ball-handling mechanisms are not controlled. */
@TeleOp(name = "Photo Robot - Drive", group = "Team")
public class PhotoRobotTeleOp extends LinearOpMode {
    // Match these names to Configure Robot on your Driver Station.
    // Left/right are from the robot's viewpoint looking toward its front.
    private static final String FRONT_LEFT = "leftFront";
    private static final String FRONT_RIGHT = "rightFront";
    private static final String BACK_LEFT = "leftBack";
    private static final String BACK_RIGHT = "rightBack";

    // Initial direction assumptions from your previous BasicTeleop.java.
    // Verify each wheel with the robot lifted before driving on the floor.
    private static final DcMotor.Direction FL_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction FR_DIRECTION = DcMotor.Direction.FORWARD;
    private static final DcMotor.Direction BL_DIRECTION = DcMotor.Direction.REVERSE;
    private static final DcMotor.Direction BR_DIRECTION = DcMotor.Direction.FORWARD;
    private static final double NORMAL_POWER = 0.45;
    private static final double SLOW_POWER = 0.20;
    private static final double DEADBAND = 0.06;

    @Override
    public void runOpMode() {
        DcMotor[] motors = new DcMotor[4];
        try {
            motors[0] = hardwareMap.get(DcMotor.class, FRONT_LEFT);
            motors[1] = hardwareMap.get(DcMotor.class, FRONT_RIGHT);
            motors[2] = hardwareMap.get(DcMotor.class, BACK_LEFT);
            motors[3] = hardwareMap.get(DcMotor.class, BACK_RIGHT);
            DcMotor.Direction[] directions = {
                FL_DIRECTION, FR_DIRECTION, BL_DIRECTION, BR_DIRECTION
            };
            for (int i = 0; i < motors.length; i++) {
                motors[i].setPower(0);
                motors[i].setDirection(directions[i]);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            telemetry.addLine("Drive only: ball mechanisms are not controlled.");
            telemetry.addLine("Left stick: move. Right stick X: turn.");
            telemetry.addLine("Hold right bumper: slow. B: stop this OpMode.");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.b) {
                    requestOpModeStop();
                    break;
                }
                double forward = deadband(-gamepad1.left_stick_y);
                double strafe = deadband(gamepad1.left_stick_x);
                double turn = deadband(gamepad1.right_stick_x);
                double scale = gamepad1.right_bumper ? SLOW_POWER : NORMAL_POWER;
                double denominator = Math.max(1.0,
                        Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
                double fl = (forward + strafe + turn) / denominator * scale;
                double fr = (forward - strafe - turn) / denominator * scale;
                double bl = (forward - strafe + turn) / denominator * scale;
                double br = (forward + strafe - turn) / denominator * scale;
                motors[0].setPower(fl);
                motors[1].setPower(fr);
                motors[2].setPower(bl);
                motors[3].setPower(br);

                telemetry.addData("Mode", gamepad1.right_bumper ? "SLOW" : "NORMAL");
                telemetry.addData("Front L / R", "%.2f / %.2f", fl, fr);
                telemetry.addData("Back L / R", "%.2f / %.2f", bl, br);
                telemetry.update();
                idle();
            }
        } finally {
            for (DcMotor motor : motors) {
                if (motor != null) motor.setPower(0);
            }
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) <= DEADBAND ? 0.0
                : Math.copySign((Math.abs(value) - DEADBAND) / (1.0 - DEADBAND), value);
    }
}
