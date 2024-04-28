package org.firstinspires.ftc.teamcode.drive.OGCode;



import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1.autoControllerStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.HALF_WAY;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.THREE_WAY;
import static org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController.ServoStatus.STACK_POSITION;
import static org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController.pozTurnClaw_COLLECT;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import java.util.List;

@TeleOp(name="DistanceOPMODE", group="Linear Opmode")

public class DistanceOPMODE extends  LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime() , timeGetVoltage = new ElapsedTime();
    public static int salut=0;
    double pozInit4Bar = 0, pozInter4Bar= 0.4, pozPlace4Bar = 0.7;
    double pozCloseClaw=0.8, pozOpenClaw=0.2;
    double kp =0, ki=0, kd=0;
    boolean isDown = true, isClosed=false, isTurned = false, isExtended = false;
    public static boolean oklow=false;
    public static double  PrecisionDenominator=1, PrecisionDenominator2=1.25;

    public void robotCentricDrive(DcMotor leftFront,DcMotor leftBack,DcMotor rightFront,DcMotor rightBack, double  lim, boolean StrafesOn , double LeftTrigger,  double RightTrigger)
    {
        double y = -gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.right_stick_x*1.1;
        if (StrafesOn == false)
        {
            x=0;
        }
        double rx = gamepad1.left_stick_x*1 - LeftTrigger + RightTrigger;

        rx/=PrecisionDenominator2;
        x/=PrecisionDenominator;
        y/=PrecisionDenominator;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,lim);
        backLeftPower = Clip(backLeftPower,lim);
        frontRightPower = Clip(frontRightPower,lim);
        backRightPower = Clip(backRightPower,lim);

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }
    public void robotCentricDriveABSO(DcMotor leftFront,DcMotor leftBack,DcMotor rightFront,DcMotor rightBack, double  lim, boolean StrafesOn , double LeftTrigger,  double RightTrigger)
    {
        double y = gamepad1.right_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x*1.1;
        double rx = gamepad1.right_stick_x*1 - LeftTrigger + RightTrigger;

        rx/=PrecisionDenominator2;
        x/=PrecisionDenominator;
        y/=PrecisionDenominator;
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeftPower = Clip(frontLeftPower,lim);
        backLeftPower = Clip(backLeftPower,lim);
        frontRightPower = Clip(frontRightPower,lim);
        backRightPower = Clip(backRightPower,lim);

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }
    double Clip(double Speed,double lim)
    {
        return Math.max(Math.min(Speed,lim),-lim);
    }
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {

        RobotMap robot=new RobotMap(hardwareMap);
        SigurantaLiftController sigurantaLiftController = new SigurantaLiftController();
        Angle4BarController angle4BarController = new Angle4BarController();
        Servo4BarController servo4BarController = new Servo4BarController();
        MotorColectareController motorColectareController = new MotorColectareController();
        CloseClawController closeClawController = new CloseClawController();
        TurnClawController turnClawController = new TurnClawController();
        RobotController robotController = new RobotController();
        BiggerController biggerController = new BiggerController();
        LiftController liftController = new LiftController();
        AutoController5_1 autoController51 = new AutoController5_1();
        AllCycleController allCycleController = new AllCycleController();



        double currentVoltage;
        VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        currentVoltage = batteryVoltageSensor.getVoltage();
        double x1=0,y1=0,x2=0;
        double loopTime = 0;
        boolean motorColectareExtension = false;
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
        angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
        robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
        autoController51.CurrentStatus = NOTHING;
        autoController51.PreviousStatus = NOTHING;
        allCycleController.CurrentStatus = AllCycleController.AllCycleControllerStatus.NOTHING;

        robotController.timerTransfer = 0.5;

        closeClawController.update(robot);
        robot.turnClaw.setPosition(pozTurnClaw_COLLECT);
        turnClawController.update(robot);
        angle4BarController.update(robot);
        servo4BarController.update(robot);
        sigurantaLiftController.update(robot);
        motorColectareController.update(robot,0, 0.6, currentVoltage);
        liftController.update(robot,0,sigurantaLiftController,currentVoltage);
        robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
        biggerController.update(robotController,closeClawController,motorColectareController);

        autoController51.Cone_Stack_Level  =5;
        autoController51.AutoLiftStatus = LiftController.LiftStatus.HIGH;
        autoController51.LimitLift = 0.85;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        imu.initialize(parameters);
        DcMotor rightFront = null;
        DcMotor rightBack = null;
        DcMotor leftFront = null;
        DcMotor leftBack = null;
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftFront = hardwareMap.get(DcMotor.class,"leftFront");
        rightBack = hardwareMap.get(DcMotor.class,"rightBack");
        leftBack = hardwareMap.get(DcMotor.class,"leftBack");


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        double lim = 1 ; /// limita vitezei la sasiu
        int pozServoStack = 6;
        boolean StrafesOn = false;
        boolean senzorOn = true;
        String typeOfDrive = "RobotCentric";
        while (opModeIsActive()) {
            if (isStopRequested()) return;

            double current4BarPosition = robot.left4Bar.getPosition();
            double currentAngle4BarPosition = robot.angle4Bar.getPosition();
            int ColectarePosition = robot.motorColectareStanga.getCurrentPosition();
            int LiftPosition = robot.dreaptaLift.getCurrentPosition(); /// folosesc doar encoderul de la dreaptaLift , celalalt nu exista.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            if (!previousGamepad1.touchpad && currentGamepad1.touchpad) {
                StrafesOn = !StrafesOn;
            }
            /// DRIVE
        double distance = robot.dsensor.getDistance(DistanceUnit.MM);
            robotCentricDrive(leftFront, leftBack, rightFront, rightBack, lim,StrafesOn , 0,0);

            if (timeGetVoltage.seconds() > 5) {
                timeGetVoltage.reset();
                currentVoltage = batteryVoltageSensor.getVoltage();
            }

            if ((!previousGamepad1.dpad_right && currentGamepad1.dpad_right)) {
                motorColectareController.CurrentStatus = HALF_WAY;

            }
            if ((!previousGamepad1.dpad_left && currentGamepad1.dpad_left)) {
                motorColectareController.CurrentStatus = THREE_WAY;
            }
            if ((!previousGamepad2.right_bumper && currentGamepad2.right_bumper)) {
                if (closeClawController.CurrentStatus == CloseClawController.closeClawStatus.CLOSED) {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                } else {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                }
            }

            if (currentGamepad2.left_trigger > 0) {
                if (!previousGamepad2.left_bumper && currentGamepad2.left_bumper) {
                    if (motorColectareController.CurrentStatus == MotorColectareController.MotorColectare.RETRACTED) {
                        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.EXTENDED_DRIVE;
                    } else {
                        motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
                    }
                }
                if ((!previousGamepad2.dpad_up && currentGamepad2.dpad_up)) {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                }
                if ((!previousGamepad2.dpad_down && currentGamepad2.dpad_down)) {
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.LOW_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus
                            .PLACE_LOW;
                }
                if ((!previousGamepad2.dpad_left && currentGamepad2.dpad_left)) {
                    robot.left4Bar.setPosition(servo4BarController.Fallen_Cones);
                    robot.right4Bar.setPosition(servo4BarController.Fallen_Cones);
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.COLLECT_CONES;
                    servo4BarController.CurrentStatus = STACK_POSITION;
                }
                if (!previousGamepad2.square && currentGamepad2.square) {
                    robot.left4Bar.setPosition(Servo4BarController.GroundPositions[2]);
                    robot.right4Bar.setPosition(Servo4BarController.GroundPositions[2]);
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    Servo4BarController.CurrentStatus = STACK_POSITION;
                }
                if (!previousGamepad2.triangle && currentGamepad2.triangle) {
                    robot.left4Bar.setPosition(Servo4BarController.GroundPositions[3]);
                    robot.right4Bar.setPosition(Servo4BarController.GroundPositions[3]);
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    Servo4BarController.CurrentStatus = STACK_POSITION;
                }
                if (!previousGamepad2.circle && currentGamepad2.circle) {
                    robot.left4Bar.setPosition(Servo4BarController.GroundPositions[4]);
                    robot.right4Bar.setPosition(Servo4BarController.GroundPositions[4]);
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    Servo4BarController.CurrentStatus = STACK_POSITION;
                }
                if (!previousGamepad2.cross && currentGamepad2.cross) {
                    robot.left4Bar.setPosition(Servo4BarController.GroundPositions[5]);
                    robot.right4Bar.setPosition(Servo4BarController.GroundPositions[5]);
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.PLACE_FIFTH_GROUND;
                    Servo4BarController.CurrentStatus = STACK_POSITION;
                }
            } else {
                if ((!previousGamepad2.dpad_down && currentGamepad2.dpad_down)) {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_COLLECT;
                    servo4BarController.Collect_Position = servo4BarController.Collect_Drive;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;

                }
                if ((!previousGamepad2.dpad_up && currentGamepad2.dpad_up)) {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE;
                }
                if ((!previousGamepad2.dpad_right && currentGamepad2.dpad_right)) {
                    robot.left4Bar.setPosition(servo4BarController.groundJunctionPosition);
                    robot.right4Bar.setPosition(servo4BarController.groundJunctionPosition);
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    servo4BarController.CurrentStatus = STACK_POSITION;
                }
                if ((!previousGamepad2.dpad_left && currentGamepad2.dpad_left)) {
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE_STACK;
                }
                if (!previousGamepad2.cross && currentGamepad2.cross) {
                    oklow = false;
                    if (liftController.CurrentStatus != LiftController.LiftStatus.HIGH_DRIVE) {
                        liftController.CurrentStatus = LiftController.LiftStatus.HIGH_DRIVE;
                    } else {
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    }
                }
                if (!previousGamepad2.square && currentGamepad2.square) {
                    if (liftController.CurrentStatus != LiftController.LiftStatus.BASE) {
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    } else {
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    }
                }
                if ((!previousGamepad2.triangle && currentGamepad2.triangle)) {
                    if (liftController.CurrentStatus!= LiftController.LiftStatus.LOW)
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.LOW;
                    }
                    else
                    {
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    }
                }
                if ((!previousGamepad2.circle && currentGamepad2.circle)) {
                    oklow = false;
                    if (liftController.CurrentStatus != LiftController.LiftStatus.MID) {
                        liftController.CurrentStatus = LiftController.LiftStatus.MID;
                    } else {
                        liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    }
                }
            }
            if (!previousGamepad1.left_bumper && currentGamepad1.left_bumper) {
                robot.left4Bar.setPosition(current4BarPosition - 0.05);
                robot.right4Bar.setPosition(current4BarPosition - 0.05);
                Servo4BarController.CurrentStatus = STACK_POSITION;
            }
            if (!previousGamepad1.right_bumper && currentGamepad1.right_bumper) {
                robot.left4Bar.setPosition(current4BarPosition + 0.05);
                robot.right4Bar.setPosition(current4BarPosition + 0.05);
                Servo4BarController.CurrentStatus = STACK_POSITION;
            }
            if ((!previousGamepad2.left_bumper && currentGamepad2.left_bumper)) {
                if (motorColectareController.CurrentStatus == MotorColectareController.MotorColectare.RETRACTED) {
                    motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.EXTENDED_DRIVE;
                } else {
                    motorColectareController.CurrentStatus = MotorColectareController.MotorColectare.RETRACTED;
                }
            }
            if (motorColectareController.CurrentStatus == MotorColectareController.MotorColectare.EXTENDED_DRIVE && servo4BarController.CurrentStatus == Servo4BarController.ServoStatus.COLLECT_DRIVE)
            {
                robot.left4Bar.setPosition(0.925);
                robot.right4Bar.setPosition(0.925);
            }
            if ((!previousGamepad1.dpad_down && currentGamepad1.dpad_down)) {
                robot.angle4Bar.setPosition(currentAngle4BarPosition + 0.05);
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.INCREMENT;
            }
            if ((!previousGamepad1.dpad_up && currentGamepad1.dpad_up)) {
                robot.angle4Bar.setPosition(currentAngle4BarPosition - 0.05);
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.INCREMENT;

            }
            if ((!previousGamepad1.cross && currentGamepad1.cross))
            {
                pozServoStack--;
                if (pozServoStack == 0)
                {
                    pozServoStack = 1 ;
                }
                robot.left4Bar.setPosition(Servo4BarController.StackPositions[pozServoStack]);
                robot.right4Bar.setPosition(Servo4BarController.StackPositions[pozServoStack]);
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                Servo4BarController.CurrentStatus = STACK_POSITION;
            }
            if ((!previousGamepad1.triangle && currentGamepad1.triangle))
            {
                if (1<=pozServoStack && pozServoStack<=5)
                {
                    pozServoStack++;
                    if (pozServoStack == 6)
                    {
                        pozServoStack = 5;
                    }
                    robot.left4Bar.setPosition(Servo4BarController.StackPositions[pozServoStack]);
                    robot.right4Bar.setPosition(Servo4BarController.StackPositions[pozServoStack]);
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    Servo4BarController.CurrentStatus = STACK_POSITION;
                }
            }
            if (gamepad1.left_trigger >0)
            {
                PrecisionDenominator = 2;
                PrecisionDenominator2 = 2.75;
            }
            else
            {
                if (servo4BarController.CurrentStatus == Servo4BarController.ServoStatus.COLLECT_DRIVE && angle4BarController.CurrentStatus == Angle4BarController.angle4BarStatus.VERTICAL)
                {
                    PrecisionDenominator2=2.75;
                    PrecisionDenominator = 1;
                }
                else if (liftController.CurrentStatus != LiftController.LiftStatus.BASE)
                {
                    PrecisionDenominator2=2.75;
                    PrecisionDenominator = 1.5;
                }
                else
                {
                    PrecisionDenominator = 1;
                    PrecisionDenominator2 = 1.5;
                }
            }

            biggerController.update(robotController,closeClawController,motorColectareController);
            robotController.update(robot,sigurantaLiftController,angle4BarController,servo4BarController,motorColectareController,closeClawController,turnClawController);
            closeClawController.update(robot);
            turnClawController.update(robot);
            servo4BarController.update(robot);
            sigurantaLiftController.update(robot);
            motorColectareController.update(robot,ColectarePosition, 1, currentVoltage);
            liftController.update(robot,LiftPosition,sigurantaLiftController,currentVoltage);
            autoController51.update(sigurantaLiftController,robot,angle4BarController,turnClawController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);
            allCycleController.update(robot, sigurantaLiftController, angle4BarController,turnClawController, liftController, servo4BarController, robotController, closeClawController, motorColectareController);
            angle4BarController.update(robot);

            double loop = System.nanoTime();

            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            telemetry.addData("distance", distance);

            loopTime = loop;
            //telemetry.addData("typeOfDrive",typeOfDrive);
        /*    drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            double yPosition = poseEstimate.getY();
            double intakeTicks = (63- Math.abs(Math.floor(poseEstimate.getY())))*15;
            if (yPosition >= -15) intakeTicks = 875;
            else intakeTicks = Math.max(0,(59- Math.abs(Math.floor(poseEstimate.getY())))*15);
            //if (intakeTicks)
            motorColectareController.extendedDrive = intakeTicks;*/

            // Read pose


            // Print pose to telemetry
            //telemetry.addData("x", poseEstimate.getX());
            //telemetry.addData("y", poseEstimate.getY());
            //telemetry.addData("heading", poseEstimate.getHeading());
            //telemetry.addData("ticksColectare", intakeTicks);
           /*telemetry.addData("AutoStatus", autoController51.CurrentStatus);
            telemetry.addData("RobotStatus",robotController.CurrentStatus);
            telemetry.addData("CurrentStatus",servo4BarController.CurrentStatus);
            telemetry.addData("salut", servo4BarController.salut);
            telemetry.addData("PreviousStatus",servo4BarController.PreviousStatus);
            telemetry.addData("WhereFromIntermediary",servo4BarController.WhereFromIntermediary);
            telemetry.addData("Timer4Bar",servo4BarController.time.seconds());
            telemetry.addData("salut",salut);
            telemetry.addData("4Barpos",robot.left4Bar.getPosition());
            telemetry.addData("CloseClawPosition",robot.closeClaw.getPosition());
            telemetry.addData("CurrentStatusCloseClaw",closeClawController.CurrentStatus);
            telemetry.addData("TurnClawPosition",robot.turnClaw.getPosition());
            telemetry.addData("Angle4BarPosition",robot.angle4Bar.getPosition());
            telemetry.addData("CurrentStatusTurnClawPosition",turnClawController.CurrentStatus);
            telemetry.addData("posColectare", ColectarePosition);
            telemetry.addData("posLift",LiftPosition);
            telemetry.addData("DreaptaPutereLift",robot.dreaptaLift.getPower());
            telemetry.addData("timpFSM", servo4BarController.time.seconds());
            telemetry.addData("SigurantaLiftStatus",sigurantaLiftController.CurrentStatus);
            telemetry.addData("sigurantaLiftPosition",robot.sigurantaLift.getPosition());*/
            telemetry.addData("M0 current", robot.motorColectareDreapta.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}