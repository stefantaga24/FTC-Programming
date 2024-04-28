package org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers;


import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.CyclingSouth.autoControllerStatus.CLOSE_THE_CLAW;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.CyclingSouth.autoControllerStatus.FOURBAR_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.CyclingSouth.autoControllerStatus.GET_LIFT_DOWN;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.CyclingSouth.autoControllerStatus.NOTHING;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.CyclingSouth.autoControllerStatus.PLACE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.CyclingSouth.autoControllerStatus.RETRIEVE_CONE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_10_1;
import static org.firstinspires.ftc.teamcode.drive.OGCode.MotorColectareController.MotorColectare.EXTENDED_10_1_SOUTH;
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

public class CyclingSouth {
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
    public static double LimitLift = 1.2;
    public static LiftController.LiftStatus AutoLiftState = LiftController.LiftStatus.HIGH;
    int ok=0,ok2=0;
    double timerInter = 2,timeStart=0,timerStartLimit = 0.5;
    public double positionFromState(int stackNumber, int number)
    {
        return Servo4BarController.CyclingSouthPositions[number];
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
                if (timerStart.seconds()>1.5)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.CLOSED;
                    timerClaw.reset();
                    CurrentStatus = RETRIEVE_CONE;
                }
                break;
            }
            case RETRIEVE_CONE:
            {
                if (ok == 0 && timerClaw.seconds()>0)
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
                if (ok2 == 0 &&timerPlace_Cone.seconds()>0.15)
                {
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.PLACE;
                    ok2 = 1;
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
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN_CLAW_SMALL;
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
                if (timerLift.seconds()>0.4)
                {
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN_CLAW_BIG;
                }
                if (timerLift.seconds()>LimitLift)
                {
                    if (Cone_Stack_Level == 5)
                    {
                        motorColectareController.CurrentStatus = RETRACTED;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    }
                    else
                    {
                        motorColectareController.CurrentStatus = EXTENDED_10_1_SOUTH;
                    }
                    liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                    CurrentStatus= NOTHING;
                }
                break;
            }
            case STACK_LEVEL:
            {
                liftController.CurrentStatus = LiftController.LiftStatus.BASE;
                if (Cone_Stack_Level ==5 )
                {
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                }
                else
                {
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                }
                servo4BarController.Collect_Position = Servo4BarController.CyclingSouthPositions[Cone_Stack_Level];
                if (Cone_Stack_Level == 1)
                {
                    stackNumber++;
                    Cone_Stack_Level = 5;
                }
                else
                {
                    Cone_Stack_Level = Cone_Stack_Level -1;
                }
                timerStart.reset();
                CurrentStatus = CLOSE_THE_CLAW;
                break;
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
