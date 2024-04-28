package org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers;

import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoLeftSouthHighJunction5_1.autoControllerSouthHigh.CLOSE_THE_CLAW;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoLeftSouthHighJunction5_1.autoControllerSouthHigh.FOURBAR_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoLeftSouthHighJunction5_1.autoControllerSouthHigh.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoLeftSouthHighJunction5_1.autoControllerSouthHigh.PLACE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoLeftSouthHighJunction5_1.autoControllerSouthHigh.RETRIEVE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_SOUTH;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_SOUTH_LEFT;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OGCode.Angle4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.CloseClawController;
import org.firstinspires.ftc.teamcode.drive.OGCode.LiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.SigurantaLiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController;

public class AutoLeftSouthHighJunction5_1 {
    public enum autoControllerSouthHigh
    {
        NOTHING,
        RETRIEVE_CONE,
        STACK_LEVEL,
        FOURBAR_DOWN,
        LIFT_CONE,
        INITIALIZE,
        CLOSE_THE_CLAW,
        PLACE_CONE,
        RETRACTED,
        GET_LIFT_DOWN,
    }
    public static autoControllerSouthHigh CurrentStatus = NOTHING, PreviousStatus = NOTHING;
    ElapsedTime timerFourBar = new ElapsedTime() ,timerStart = new ElapsedTime(),timerClaw = new ElapsedTime() , timerPlace_Cone = new ElapsedTime(), timerLift =new ElapsedTime();
    public static int Cone_Stack_Level=5;
    public static double LimitLift = 0.75;
    public static LiftController.LiftStatus AutoLiftStatus = LiftController.LiftStatus.HIGH;
    boolean moreThanOneStack = false;
    int ok=0;
    double timerInter = 2,timeStart=0;
    public void update(SigurantaLiftController sigurantaLiftController, RobotMap Robotel, Angle4BarController angle4BarController, TurnClawController turnClawController, LiftController liftController, Servo4BarController servo4BarController, RobotController robotController, CloseClawController closeClawController, MotorColectareController motorColectareController)
    {
        switch (CurrentStatus)
        {
            case INITIALIZE:
            {
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.INITIALIZE;
                motorColectareController.CurrentStatus = RETRACTED;
                closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                robotController.CurrentStatus = RobotController.RobotControllerStatus.START;
                liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                CurrentStatus = NOTHING;
                break;
            }
            case FOURBAR_DOWN:
            {
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                timerFourBar.reset();
                CurrentStatus = CLOSE_THE_CLAW;
                break;
            }
            case CLOSE_THE_CLAW:
            {
                if (timerFourBar.seconds()>0.85)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                    timerClaw.reset();
                    CurrentStatus = RETRIEVE_CONE;
                }
                break;
            }
            case RETRIEVE_CONE:
            {
                if (ok == 0 && timerClaw.seconds()>0.1)
                {
                    ok=1;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.RAISED;
                }
                if (timerClaw.seconds()>0.35)
                {
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.PLACE;
                    motorColectareController.CurrentStatus = RETRACTED;
                    timerPlace_Cone.reset();
                    ok=0;
                    CurrentStatus = PLACE_CONE;
                }
                break;
            }
            case PLACE_CONE:
            {
                if (ok == 0 && timerPlace_Cone.seconds()>0.55)
                {
                    Robotel.left4Bar.setPosition(servo4BarController.Place_Cone_Position);
                    Robotel.right4Bar.setPosition(servo4BarController.Place_Cone_Position);

                    ok = 1;
                }
                if (timerPlace_Cone.seconds()>0.7)
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                }
                if (timerPlace_Cone.seconds()>0.85)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    CurrentStatus = NOTHING;
                }
                break;
            }
            case GET_LIFT_DOWN:
            {
                if (timerLift.seconds()>LimitLift)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    CurrentStatus= NOTHING;
                }
                break;
            }
            case STACK_LEVEL:
            {
                ok=0;
                motorColectareController.CurrentStatus = EXTENDED_SOUTH_LEFT;
                closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN_CLAW_BIG;
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                if (Cone_Stack_Level==5)
                {
                    servo4BarController.Collect_Position = servo4BarController.Fifth_Cone_Position_South;
                    Cone_Stack_Level =4;
                }
                else if (Cone_Stack_Level==4)
                {
                    servo4BarController.Collect_Position = servo4BarController.Fourth_Cone_Position_South;
                    Cone_Stack_Level =3;
                }
                else if (Cone_Stack_Level==3)
                {
                    servo4BarController.Collect_Position = servo4BarController.Third_Cone_Position_South;
                    Cone_Stack_Level =2;
                }
                else if (Cone_Stack_Level==2)
                {
                    servo4BarController.Collect_Position = servo4BarController.Second_Cone_Position_South;
                    Cone_Stack_Level =1;
                }
                else if (Cone_Stack_Level==1)
                {
                    servo4BarController.Collect_Position = servo4BarController.Ground_Position_South;
                    Cone_Stack_Level =5;
                }
                timerStart.reset();
                CurrentStatus = FOURBAR_DOWN;
                break;
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
