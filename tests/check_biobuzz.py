"""Hardware-free regression check. Run with Python 3 and a JDK on PATH.
Compiles the production classes against minimal FTC doubles, NOT the Android SDK.
"""
from pathlib import Path
import subprocess
import tempfile
import shutil

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "TeamCode/src/main/java/org/firstinspires/ftc/teamcode"
if not SOURCE.exists():
    SOURCE = Path(__file__).resolve().parent  # Local review workspace.

STUBS = {
"com/qualcomm/robotcore/eventloop/opmode/Autonomous.java":
'package com.qualcomm.robotcore.eventloop.opmode; public @interface Autonomous { String name(); String group(); }',
"com/qualcomm/robotcore/eventloop/opmode/Disabled.java":
'package com.qualcomm.robotcore.eventloop.opmode; public @interface Disabled {}',
"com/qualcomm/robotcore/eventloop/opmode/LinearOpMode.java":
"""package com.qualcomm.robotcore.eventloop.opmode;
import com.qualcomm.robotcore.hardware.HardwareMap;
public abstract class LinearOpMode {
 public boolean active=true;
 public HardwareMap hardwareMap=new HardwareMap();
 public Telemetry telemetry=new Telemetry();
 public abstract void runOpMode();
 public boolean opModeIsActive(){return active;}
 public boolean isStopRequested(){return !active;}
 public void waitForStart(){}
 public void idle(){}
 public static class Telemetry {
  public void addData(String k,Object v){}
  public void addData(String k,String f,Object... v){}
  public void addLine(String s){}
  public void update(){}
 }
}""",
"com/qualcomm/robotcore/hardware/HardwareMap.java":
"""package com.qualcomm.robotcore.hardware;
public class HardwareMap {
 public <T> T get(Class<T> cls,String name) {
  try { return cls.getDeclaredConstructor().newInstance(); }
  catch(Exception e){throw new RuntimeException(e);}
 }
}""",
"com/qualcomm/robotcore/hardware/DcMotor.java":
"""package com.qualcomm.robotcore.hardware;
public class DcMotor {
 public enum Direction {FORWARD,REVERSE}
 public enum ZeroPowerBehavior {BRAKE,FLOAT}
 public enum RunMode {STOP_AND_RESET_ENCODER,RUN_USING_ENCODER,RUN_WITHOUT_ENCODER,RUN_TO_POSITION}
 public double power; public int target;
 public void setPower(double p){power=p;}
 public void setMode(RunMode m){}
 public void setDirection(Direction d){}
 public void setZeroPowerBehavior(ZeroPowerBehavior z){}
 public void setTargetPosition(int p){target=p;}
 public int getCurrentPosition(){return target;}
 public boolean isBusy(){return false;}
}""",
"com/qualcomm/robotcore/util/ElapsedTime.java":
"""package com.qualcomm.robotcore.util;
public class ElapsedTime {
 private final long start=System.nanoTime();
 public double seconds(){return (System.nanoTime()-start)/1e9;}
 public double milliseconds(){return seconds()*1000;}
}""",
"org/firstinspires/ftc/teamcode/BioBuzzCheck.java":
"""package org.firstinspires.ftc.teamcode;
import java.util.*;
public class BioBuzzCheck {
 static void check(boolean b,String message){if(!b)throw new AssertionError(message);}
 static class FakeRobot extends BioBuzzRobot {
  List<String> events=new ArrayList<>();
  boolean failPickup, stopPickup, allowShot=true;
  BioBuzzAutonomous op;
  FakeRobot(BioBuzzAutonomous op){super(op);this.op=op;init(op.hardwareMap);intakeSettleMs=0;}
  @Override public boolean driveTo(double x,double z){
   events.add("navigate");
   if(stopPickup)op.active=false;
   return !failPickup;
  }
  @Override public boolean facePoint(double x,double z){return true;}
  @Override public boolean faceHeading(double h){return true;}
  @Override public boolean collectOneAt(double x,double z){
   events.add("pickup");return super.collectOneAt(x,z);
  }
  @Override public boolean shootExpectedBalls(){
   events.add("shoot");
   return allowShot; // Sequence-only double: deliberately does not clear inventory.
  }
  boolean realShot(){return super.shootExpectedBalls();}
 }
 public static void main(String[] args){
  BioBuzzAutonomous op=new BioBuzzAutonomous();
  FakeRobot r=new FakeRobot(op);op.robot=r;
  check(op.runSequence(),"planned sequence");
  List<String> actions=new ArrayList<>();
  for(String e:r.events)if(!e.equals("navigate"))actions.add(e);
  check(actions.equals(Arrays.asList("pickup","pickup","pickup","shoot","pickup","shoot")),"3/shoot/1/shoot");
  check(r.getEstimatedBalls()==4,"navigation must not count balls");
  check(!r.collectOneAt(0,0),"capacity rejects fifth pickup");
  check(r.intake.power==0,"capacity stops intake");
  check(!r.realShot(),"real shooter stays disabled");
  check(r.getEstimatedBalls()==4 && r.endFlip.power==0,"disabled shot preserves inventory");
  op=new BioBuzzAutonomous();r=new FakeRobot(op);op.robot=r;r.allowShot=false;
  check(!op.runSequence() && r.getEstimatedBalls()==3,"failed shot prevents final collection");
  op=new BioBuzzAutonomous();r=new FakeRobot(op);r.failPickup=true;
  check(!r.collectOneAt(0,0) && r.getEstimatedBalls()==0 && r.intake.power==0,"failed pickup cleanup");
  r.failPickup=false;r.stopPickup=true;
  check(!r.collectOneAt(0,0) && r.getEstimatedBalls()==0 && r.intake.power==0,"STOP pickup cleanup");
  op=new BioBuzzAutonomous();op.runOpMode();
  check(op.robot.getEstimatedBalls()==0 && op.robot.endFlip.power==0,"default route disabled");
  op=new BioBuzzAutonomous();BioBuzzRobot drive=new BioBuzzRobot(op);drive.init(op.hardwareMap);
  check(drive.strafe(1),"strafe completes");
  check(drive.leftFront.target>0 && drive.leftBack.target<0
    && drive.rightBack.target>0 && drive.rightFront.target<0,"mecanum strafe wheel order");
  System.out.println("PASS: sequence, final count, capacity, disabled shooter, failure and STOP cleanup, default gate");
 }
}"""
}

with tempfile.TemporaryDirectory() as directory:
    root = Path(directory)
    for name, content in STUBS.items():
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")
    for name in ("BioBuzzRobot.java", "BioBuzzAutonomous.java"):
        shutil.copyfile(SOURCE / name, root / "org/firstinspires/ftc/teamcode" / name)
    sources = [str(p) for p in root.rglob("*.java")]
    subprocess.run(["javac", "-d", str(root / "classes"), *sources], check=True)
    subprocess.run(["java", "-cp", str(root / "classes"), "org.firstinspires.ftc.teamcode.BioBuzzCheck"], check=True)
