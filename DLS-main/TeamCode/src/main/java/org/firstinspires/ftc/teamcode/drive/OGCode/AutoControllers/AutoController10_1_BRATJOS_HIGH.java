package org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers;


import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1_BRATJOS_HIGH.autoControllerStatus.CLOSE_THE_CLAW;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1_BRATJOS_HIGH.autoControllerStatus.FOURBAR_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1_BRATJOS_HIGH.autoControllerStatus.GET_LIFT_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1_BRATJOS_HIGH.autoControllerStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1_BRATJOS_HIGH.autoControllerStatus.PLACE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController10_1_BRATJOS_HIGH.autoControllerStatus.RETRIEVE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_10_1;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_2050;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_FAST;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.RETRACTED_0;

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

public class AutoController10_1_BRATJOS_HIGH {
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
    public static double LimitLift = 0.55;
    public static LiftController.LiftStatus AutoLiftState = LiftController.LiftStatus.HIGH;
    int ok=0,ok2=0;
    double timerInter = 2,timeStart=0,timerStartLimit = 0.5;
    public double positionFromState(int stackNumber, int number)
    {
        double finalPos = Servo4BarController.Fifth_Cone_Position_MID;
        if (stackNumber == 0)
        {
            switch (number)
            {
                case 5:
                {
                    finalPos =  Servo4BarController.Fifth_Cone_Position_MID;
                    break;
                }
                case 4:
                {
                    finalPos =  Servo4BarController.Fourth_Cone_Position;
                    break;
                }
                case 3:
                {
                    finalPos =  Servo4BarController.Third_Cone_Position;
                    break;
                }
                case 2:
                {
                    finalPos =  Servo4BarController.Second_Cone_Position;
                    break;
                }
                case 1:
                {
                    finalPos =  Servo4BarController.Ground_Position;
                    break;
                }
            }
        }
        else
        {
            switch (number)
            {
                case 5:
                {
                    finalPos =  Servo4BarController.Fifth_Cone_Position_BRATJOSHIGH_2;
                    break;
                }
                case 4:
                {
                    finalPos =  Servo4BarController.Fourth_Cone_Position_BRATJOSHIGH_2;
                    break;
                }
                case 3:
                {
                    finalPos =  Servo4BarController.Third_Cone_Position_BRATJOSHIGH_2;
                    break;
                }
                case 2:
                {
                    finalPos =  Servo4BarController.Second_Cone_Position_BRATJOSHIGH_2;
                    break;
                }
                case 1:
                {
                    finalPos =  Servo4BarController.Ground_Position_BRATJOSHIGH_2;
                    break;
                }
            }
        }
        return finalPos;
    }
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
            case CLOSE_THE_CLAW:
            {
                if (timerStart.seconds()>timerStartLimit)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                    timerClaw.reset();
                    CurrentStatus = RETRIEVE_CONE;
                }
                break;
            }
            case RETRIEVE_CONE:
            {
                if (ok == 0 && timerClaw.seconds()>0.05)
                {
                    ok=1;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.RAISED;
                }
                if (timerClaw.seconds()>0.2)
                {

                    motorColectareController.CurrentStatus = RETRACTED_0;
                    timerPlace_Cone.reset();
                    ok=0;
                    ok2=0;
                    CurrentStatus = PLACE_CONE;
                }
                break;
            }
            case PLACE_CONE:
            {
                if (ok2==0 && timerPlace_Cone.seconds()>0.05)
                {
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.PLACE;
                    ok2=1;
                }
                if (ok == 0 && timerPlace_Cone.seconds()>0.55)
                {
                    Robotel.left4Bar.setPosition(servo4BarController.Place_Cone_Position);
                    Robotel.right4Bar.setPosition(servo4BarController.Place_Cone_Position);
                    ok = 1;
                }
                if (timerPlace_Cone.seconds()>0.75)
                {
                    sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                }
                if (timerPlace_Cone.seconds()>0.8)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                }
                if (timerPlace_Cone.seconds()>0.8)
                {
                    servo4BarController.Collect_Position = positionFromState(stackNumber,Cone_Stack_Level);
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                    ok=0;
                    if (stackNumber == 1 && Cone_Stack_Level == 5)
                    {
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                        CurrentStatus = NOTHING;
                    }
                    else
                    {
                        timerLift.reset();
                        liftController.CurrentStatus = AutoLiftState;
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                        CurrentStatus = GET_LIFT_DOWN;
                    }
                }
                break;
            }
            case GET_LIFT_DOWN:
            {
                if (timerLift.seconds()>LimitLift-0.1)
                {
                    if (Cone_Stack_Level == 5)
                    {
                        motorColectareController.CurrentStatus = RETRACTED;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    }
                    else
                    {
                        motorColectareController.CurrentStatus = EXTENDED_10_1;
                    }
                }
                if (timerLift.seconds()>LimitLift)
                {
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    CurrentStatus= NOTHING;
                }
                break;
            }
            case STACK_LEVEL:
            {
                closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                if (Cone_Stack_Level==5)
                {
                    if (stackNumber == 0)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Fifth_Cone_Position_BRATJOSHIGH;
                    }
                    else
                    {
                        servo4BarController.Collect_Position = servo4BarController.Fifth_Cone_Position_BRATJOSHIGH_2;
                    }
                    Cone_Stack_Level =4;
                }
                else if (Cone_Stack_Level==4)
                {
                    if (stackNumber == 0)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Fourth_Cone_Position;
                    }
                    else
                    {
                        servo4BarController.Collect_Position = servo4BarController.Fourth_Cone_Position_BRATJOSHIGH_2;
                    }
                    Cone_Stack_Level =3;
                }
                else if (Cone_Stack_Level==3)
                {
                    if (stackNumber == 0)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Third_Cone_Position;
                    }
                    else
                    {
                        servo4BarController.Collect_Position = servo4BarController.Third_Cone_Position_BRATJOSHIGH_2;
                    }
                    Cone_Stack_Level =2;
                }
                else if (Cone_Stack_Level==2)
                {
                    if (stackNumber == 0)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Second_Cone_Position;
                    }
                    else
                    {
                        servo4BarController.Collect_Position = servo4BarController.Second_Cone_Position_BRATJOSHIGH_2;
                    }
                    Cone_Stack_Level =1;
                }
                else if (Cone_Stack_Level==1)
                {
                    if (stackNumber == 0)
                    {
                        servo4BarController.Collect_Position = servo4BarController.Ground_Position;
                    }
                    else
                    {
                        servo4BarController.Collect_Position = servo4BarController.Ground_Position_BRATJOSHIGH_2;
                    }
                    Cone_Stack_Level =5;
                    stackNumber++;
                }
                timerStart.reset();
                timerStartLimit = 0.4;
                CurrentStatus = CLOSE_THE_CLAW;
                break;
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
