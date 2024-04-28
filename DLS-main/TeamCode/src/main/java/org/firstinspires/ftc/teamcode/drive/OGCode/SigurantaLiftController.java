package org.firstinspires.ftc.teamcode.drive.OGCode;


import static org.firstinspires.ftc.teamcode.drive.OGCode.SigurantaLiftController.SigurantaLift.START;

public class SigurantaLiftController {
    public enum SigurantaLift
    {
        START,
        TRANSFER,
        JUNCTION,
    }
    public static SigurantaLift CurrentStatus = START, PreviousStatus = START;
    double transfer_Position = 0.3 , junction_Position = 0.71;
    public void update(RobotMap Robotel)
    {
            switch (CurrentStatus)
            {
                case TRANSFER:
                {
                    Robotel.sigurantaLift.setPosition(transfer_Position);
                    break;
                }
                case JUNCTION:
                {
                    Robotel.sigurantaLift.setPosition(junction_Position);
                    break;
                }

            }
    }
}
