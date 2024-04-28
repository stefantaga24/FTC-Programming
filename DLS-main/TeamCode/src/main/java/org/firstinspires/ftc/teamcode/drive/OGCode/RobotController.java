package org.firstinspires.ftc.teamcode.drive.OGCode;

import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_COLLECT;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_PLACE;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_PLACE_FIRST_CONE_AUTO;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.INTER_GO_PLACE_STACK;
import static org.firstinspires.ftc.teamcode.drive.OGCode.RobotController.RobotControllerStatus.START;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.Current;
import org.firstinspires.ftc.teamcode.drive.OGCode.AutoControllers.AutoController5_1;

public class RobotController {

    public enum RobotControllerStatus
    {
        START,
        GO_COLLECT,
        INTER_GO_COLLECT,
        GO_PLACE,
        INTER_GO_PLACE,
        GO_PLACE_STACK,
        INTER_GO_PLACE_STACK,
        GO_PLACE_FIRST_CONE_AUTO,
        INTER_GO_PLACE_FIRST_CONE_AUTO,
    }
    public double timerTransfer = 1.2;
    public static RobotControllerStatus CurrentStatus = START, PreviousStatus = START;
    ElapsedTime timerGO_COLLECT = new ElapsedTime() , timerGO_PLACE = new ElapsedTime() ,timeCOLLECT_RAPID_FIRE = new ElapsedTime();
    public void update(RobotMap Robot, SigurantaLiftController sigurantaLiftController,
                       Angle4BarController angle4BarController, Servo4BarController servo4BarController,
                       MotorColectareController motorColectareController, CloseClawController closeClawController, TurnClawController turnClawController)
    {
        if (PreviousStatus != CurrentStatus || CurrentStatus == INTER_GO_COLLECT || CurrentStatus == INTER_GO_PLACE || CurrentStatus == INTER_GO_PLACE_FIRST_CONE_AUTO || CurrentStatus == INTER_GO_PLACE_STACK)
        {
            switch (CurrentStatus)
            {

                case GO_COLLECT:
                {
                    timerGO_COLLECT.reset();
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.COLLECT_DRIVE;
                    angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.VERTICAL;
                    CurrentStatus = INTER_GO_COLLECT;
                    break;
                }
                case INTER_GO_COLLECT:
                {
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.COLLECT;
                    closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                    if (timerGO_COLLECT.seconds()>0.5)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN_COLLECT;
                        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.TRANSFER;
                        CurrentStatus = START;
                    }
                    break;
                }
                case GO_PLACE:
                {
                    timerGO_PLACE.reset();
                    angle4BarController.CurrentStatus= Angle4BarController.angle4BarStatus.PLACE;
                    turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.PLACE_CONE;
                    CurrentStatus = INTER_GO_PLACE;
                    break;
                }
                case INTER_GO_PLACE:
                {
                    if (sigurantaLiftController.CurrentStatus!= SigurantaLiftController.SigurantaLift.JUNCTION && timerGO_PLACE.seconds()>timerTransfer)
                    {
                        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    }
                    if (timerGO_PLACE.seconds()>timerTransfer+0.15)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        angle4BarController.CurrentStatus= Angle4BarController.angle4BarStatus.VERTICAL;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        CurrentStatus = START;
                    }
                    break;
                }
                case GO_PLACE_STACK:
                {
                    timerGO_PLACE.reset();
                    angle4BarController.CurrentStatus= Angle4BarController.angle4BarStatus.RAISED;
                    servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                    CurrentStatus = INTER_GO_PLACE_STACK;
                    break;
                }
                case INTER_GO_PLACE_STACK:
                {
                    if (turnClawController.CurrentStatus != TurnClawController.TurnClawStatus.PLACE && timerGO_PLACE.seconds()>0.15)
                    {
                        angle4BarController.CurrentStatus = Angle4BarController.angle4BarStatus.PLACE;
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    }
                    if (timerGO_PLACE.seconds()>0.45)
                    {
                        Robot.left4Bar.setPosition(Servo4BarController.Place_Cone_Position);
                        Robot.right4Bar.setPosition(Servo4BarController.Place_Cone_Position);
                    }
                    if (timerGO_PLACE.seconds()>0.7)
                    {
                        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    }
                    if (timerGO_PLACE.seconds()>1)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        angle4BarController.CurrentStatus= Angle4BarController.angle4BarStatus.VERTICAL;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        CurrentStatus = START;
                    }
                    break;
                }
                case GO_PLACE_FIRST_CONE_AUTO:
                {
                    timerGO_PLACE.reset();
                    servo4BarController.CurrentStatus=Servo4BarController.ServoStatus.PLACE_CONE;
                    angle4BarController.CurrentStatus= Angle4BarController.angle4BarStatus.RAISED;
                    CurrentStatus = INTER_GO_PLACE_FIRST_CONE_AUTO;
                    break;
                }
                case INTER_GO_PLACE_FIRST_CONE_AUTO:
                {
                    if (timerGO_PLACE.seconds()>0.5)
                    {

                        angle4BarController.CurrentStatus= Angle4BarController.angle4BarStatus.PLACE;
                        turnClawController.CurrentStatus = TurnClawController.TurnClawStatus.PLACE;
                    }
                    if (sigurantaLiftController.CurrentStatus!= SigurantaLiftController.SigurantaLift.JUNCTION && timerGO_PLACE.seconds()>timerTransfer)
                    {
                        sigurantaLiftController.CurrentStatus = SigurantaLiftController.SigurantaLift.JUNCTION;
                    }
                    if (timerGO_PLACE.seconds()>timerTransfer+0.35)
                    {
                        closeClawController.CurrentStatus = CloseClawController.closeClawStatus.OPEN;
                        angle4BarController.CurrentStatus= Angle4BarController.angle4BarStatus.VERTICAL;
                        servo4BarController.CurrentStatus = Servo4BarController.ServoStatus.DRIVE_POSITION;
                        CurrentStatus = START;
                    }
                    break;
                }
            }
        }
        PreviousStatus = CurrentStatus;
    }
}
