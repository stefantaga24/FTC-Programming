package org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers;

import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1.autoControllerStatus.CLOSE_THE_CLAW;

import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1.autoControllerStatus.FOURBAR_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1.autoControllerStatus.GET_LIFT_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1.autoControllerStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1.autoControllerStatus.PLACE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1.autoControllerStatus.RETRIEVE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.OGCode.Angle4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.CloseClawController;
import org.firstinspires.ftc.teamcode.drive.OGCode.LiftController;
import org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotController;
import org.firstinspires.ftc.teamcode.drive.OGCode.RobotMap;
import org.firstinspires.ftc.teamcode.drive.OGCode.Servo4BarController;
import org.firstinspires.ftc.teamcode.drive.OGCode.TurnClawController;

public class AutoController10_1 {
    public enum autoControllerStatus
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
    public static autoControllerStatus CurrentStatus = NOTHING, PreviousStatus = NOTHING;
    ElapsedTime timerFourBar = new ElapsedTime() ,timerStart = new ElapsedTime(),timerClaw = new ElapsedTime() , timerPlace_Cone = new ElapsedTime(), timerLift =new ElapsedTime();
    public static int Cone_Stack_Level=5;
    public static int stackNumber = 0;
    public static double LimitLift = 0.75;
    public static LiftController.LiftStatus AutoLiftState = LiftController.LiftStatus.HIGH;
    int ok=0;
    double timerInter = 2,timeStart=0;
    public void update(RobotMap Robotel, Angle4BarController angle4BarController, TurnClawController turnClawController, LiftController liftController, Servo4BarController servo4BarController, RobotController robotController, CloseClawController closeClawController, MotorColectareController motorColectareController)
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
                if (Cone_Stack_Level ==4 || Cone_Stack_Level ==3)
                {
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.LIL_FRONT;
                }
                    timerFourBar.reset();
                    CurrentStatus = CLOSE_THE_CLAW;
                break;
            }
            case CLOSE_THE_CLAW:
            {
                if (timerFourBar.seconds()>0.5)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                    timerClaw.reset();
                    CurrentStatus = RETRIEVE_CONE;
                }
                break;
            }
            case RETRIEVE_CONE:
            {
                if (ok == 0 && timerClaw.seconds()>0.35)
                {
                    ok=1;
                    robotController.CurrentStatus = RobotController.RobotControllerStatus.GO_PLACE_FIRST_CONE_AUTO;
                }
                if (timerClaw.seconds()>0.6)
                {
                    motorColectareController.CurrentStatus = RETRACTED;
                    timerPlace_Cone.reset();
                    CurrentStatus = PLACE_CONE;
                }
                break;
            }
            case PLACE_CONE:
            {
                if (timerPlace_Cone.seconds()>1.8)
                {
                    if (stackNumber == 1 && Cone_Stack_Level == 5)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                        CurrentStatus = NOTHING;
                    }
                    else
                    {
                        timerLift.reset();
                        liftController.CurrentStatus = AutoLiftState;
                        if (Cone_Stack_Level != 5 )
                        {
                            if (stackNumber==1 && Cone_Stack_Level==3)
                            {
                                motorColectareController.CurrentStatus = RETRACTED;
                            }
                            else
                            {
                                motorColectareController.CurrentStatus = EXTENDED;
                            }
                        }
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                        CurrentStatus = GET_LIFT_DOWN;
                    }
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
                motorColectareController.CurrentStatus = EXTENDED;
                closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                robotController.timerTransfer = 1.1;
                if (Cone_Stack_Level==5)
                {
                    servo4BarController.Collect_Position = servo4BarController.Fifth_Cone_Position;
                    Cone_Stack_Level =4;
                }
                else if (Cone_Stack_Level==4)
                {
                    servo4BarController.Collect_Position = servo4BarController.Fourth_Cone_Position;
                    Cone_Stack_Level =3;
                }
                else if (Cone_Stack_Level==3)
                {
                    servo4BarController.Collect_Position = servo4BarController.Third_Cone_Position;
                    Cone_Stack_Level =2;
                }
                else if (Cone_Stack_Level==2)
                {
                    servo4BarController.Collect_Position = servo4BarController.Second_Cone_Position;
                    Cone_Stack_Level =1;
                }
                else if (Cone_Stack_Level==1)
                {
                    servo4BarController.Collect_Position = servo4BarController.Ground_Position;
                    Cone_Stack_Level =5;
                    stackNumber++;
                }
                timerStart.reset();
                CurrentStatus = FOURBAR_DOWN;
                break;
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
