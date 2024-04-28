package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        double x_CYCLING_POSITION = 17, y_CYCLING_POSITION = -18, Angle_CYCLING_POSITION = 7.2;
        double x_CYCLING_POSITION_INTER = 37.5, y_CYCLING_POSITION_INTER = -12;
        double x_CYCLE_LEFT = -14.02679549603315,y_CYCLE_LEFT = -18 , Angle_CYCLE_LEFT = 170;
        double x_PLACE_SOUTH_HIGH_LEFT = -14.5, y_PLACE_SOUTH_HIGH_LEFT = -17.5, Angle_PLACE_SOUTH_HIGH_LEFT = 170;
        double x_PARK1 = -11.5, y_PARK1 = -14, Angle_PARK1 = 90;
        double x_PARK2 = -33, y_PARK2 = -17, Angle_PARK2 = 270;
        double x_PARK3 = 11.5, y_PARK3 = -20, Angle_PARK3 = 90;
        double x_PreloadIntf = 37, y_PrelaodIntf = 0, Angle_PreloadIntf = -1;
        double x_GoTo = 37, y_GoTo = -12.5, Angle_GoTo =-1;
        double x_COLLECT_POSITION = 34, y_COLLECT_POSITION = -15, Angle_COLLECT_POSITION = 45;
        double x_Slide = 37 , y_Slide= -5;
        Pose2d COLLECT_POSITION_5 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION));
        double x_PLACE_SOUTH_HIGH = 12.5, y_PLACE_SOUTH_HIGH = -16, Angle_PLACE_SOUTH_HIGH = 20;

        double y_LLH = -5.5, Angle_LLH = 192.5, x_LLH = -36;
        Pose2d StartPositionRight = new Pose2d(35,-63,Math.toRadians(270));
        Pose2d StartPositionLeft = new Pose2d(-35,-63,Math.toRadians(270));
        Pose2d PLACE = new Pose2d(x_PLACE_SOUTH_HIGH, y_PLACE_SOUTH_HIGH, Math.toRadians(Angle_PLACE_SOUTH_HIGH));
        Pose2d PLACE_SOUTH_HIGH_LEFT_10 = new Pose2d(x_PLACE_SOUTH_HIGH_LEFT-1,y_PLACE_SOUTH_HIGH_LEFT,Math.toRadians(Angle_PLACE_SOUTH_HIGH_LEFT));

        RoadRunnerBotEntity Autonomia1 = new DefaultBotBuilder(meepMeep)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(PLACE_SOUTH_HIGH_LEFT_10)
                                //.setTangent(70)
                                .lineToLinearHeading(new Pose2d(x_PARK3,y_PARK3,Math.toRadians(90)))
                                .build()
                );

        RoadRunnerBotEntity Autonomia2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
               // .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(33,-23,Math.toRadians(20)))
                                .build()
                );
        RoadRunnerBotEntity Autonomia3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .setReversed(true)
                                .back(10)
                                .splineToLinearHeading(new Pose2d(10,-30,Math.toRadians(315)), Math.toRadians(0))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-33,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(40,-33))
                                .splineToSplineHeading(new Pose2d(50,-30,Math.toRadians(45)),Math.toRadians(45))
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(40,-33,Math.toRadians(0)),Math.toRadians(180))
                                .lineTo(new Vector2d(25,-33))
                                .splineToSplineHeading(new Pose2d(10,-30,Math.toRadians(315)),Math.toRadians(135))
                                .build()
                );
        RoadRunnerBotEntity Autonomia4 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(35,-6,Math.toRadians(345)))
                                .lineTo(new  Vector2d(34, -7))
                                .splineToSplineHeading(new Pose2d(10,-12.5,Math.toRadians(180)),Math.toRadians(180))
                                .lineTo(new Vector2d(-30,-12.5))
                                .build()
                );
        RoadRunnerBotEntity Autonomia5 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(35,-20,Math.toRadians(10)))
                                .lineTo(new  Vector2d(33, -18))
                                .splineToSplineHeading(new Pose2d(-15,-17,Math.toRadians(180)),Math.toRadians(180))
                                .lineTo(new Vector2d(-30,-17))
                                .lineToLinearHeading(new Pose2d(-35,-20,Math.toRadians(167)))
                                .build()
                );
        RoadRunnerBotEntity Autonomia7 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionLeft)
                               // .setReversed(true)
                                .back(43)
                                .splineToSplineHeading(new Pose2d(-27.5,-5.5,Math.toRadians(225)),Math.toRadians(35))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-31,-7.5))
                                .splineToSplineHeading(new Pose2d(-55,-13,Math.toRadians(180)),Math.toRadians(180))
                               // .waitSeconds(3)
                                .lineTo(new Vector2d(-65,-13))
                                .waitSeconds(1)
                                .lineTo(new Vector2d(-40,-13))
                                .splineToSplineHeading(new Pose2d(-29,-20,Math.toRadians(125)),Math.toRadians(315))
                                .waitSeconds(1)
                                .build()
                );
        RoadRunnerBotEntity LeftAutonomySouthHigh5_1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionLeft)
                                .lineToLinearHeading(new Pose2d(-34.5,-14,Math.toRadians(217)))
                                .turn(Math.toRadians(-37))
                                .lineToLinearHeading(new Pose2d(-10,-14,Math.toRadians(150)))
                                .lineToLinearHeading(new Pose2d(-34.5,-14,Math.toRadians(180)))
                                .build()
                );
        RoadRunnerBotEntity RightAutonomySouthHigh5_1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                .lineToLinearHeading(new Pose2d(33,-10,Math.toRadians(315)))
                                .turn(Math.toRadians(45))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-10))
                                .splineToSplineHeading(new Pose2d(5,-15,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-10,Math.toRadians(0)),Math.toRadians(0))
                                /*.lineTo(new Vector2d(33,-10))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-9))
                                .splineToSplineHeading(new Pose2d(5,-14,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-9,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(33,-9))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-8.5))
                                .splineToSplineHeading(new Pose2d(5,-13.5,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-8.5,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(33,-8.5))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-8))
                                .splineToSplineHeading(new Pose2d(5,-13,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-8,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(33,-8))
                                .setReversed(true)
                                .lineTo(new Vector2d(25,-7.5))
                                .splineToSplineHeading(new Pose2d(5,-12.5,Math.toRadians(45)),Math.toRadians(225))
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(25,-7.5,Math.toRadians(0)),Math.toRadians(0))
                                .lineTo(new Vector2d(33,-7.5))*/
                                .build()
                );
        RoadRunnerBotEntity Carte = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionLeft)
                                .lineToLinearHeading(new Pose2d(-34.5,-14,Math.toRadians(217)))
                                .build()
                );
        RoadRunnerBotEntity Traiectorii = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                //.lineTo(new Vector2d(-33,-60))
                                .lineTo(new Vector2d(25,-59))
                                .splineToConstantHeading(new Vector2d(13,-50),Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(13,-35,Math.toRadians(13)),Math.toRadians(90))
                                .lineTo(new Vector2d(13,-20))
                                //.splineToConstantHeading(new Vector2d(17,-17),Math.toRadians(315))
                                //.splineToConstantHeading(new Vector2d(17,-17),Math.toRadians(0))
                                .build()
                );

        Pose2d PLACE_SOUTH_HIGH = new Pose2d(x_PLACE_SOUTH_HIGH,y_PLACE_SOUTH_HIGH,Math.toRadians(Angle_PLACE_SOUTH_HIGH));
        //double x_COLLECT_POSITION = 17, y_COLLECT_POSITION = -14, Angle_COLLECT_POSITION = -1;
        Pose2d COLLECT_POSITION_1 = new Pose2d(x_COLLECT_POSITION,y_COLLECT_POSITION,Math.toRadians(Angle_COLLECT_POSITION+1.2));

        double x_SWITCH_LEFT = -13.5, y_SWITCH_LEFT = -15.5, Angle_SIWTCH_LEFT = 160;
        Pose2d SWITCH = new Pose2d(x_SWITCH_LEFT, y_SWITCH_LEFT, Math.toRadians(Angle_SIWTCH_LEFT));
      //  Pose2d PLACE_SOUTH_HIGH_LEFT_10 = new Pose2d(x_PLACE_SOUTH_HIGH_LEFT,y_PLACE_SOUTH_HIGH_LEFT,Math.toRadians(Angle_PLACE_SOUTH_HIGH_LEFT));

        RoadRunnerBotEntity South_1_8 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
               // .setConstraints(60, 50, 5.69, 5.69, 7)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPositionRight)
                                //.setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(9,-20, Math.toRadians(90)),Math.toRadians(270))
                                //.setTangent(Math.toRadians(175))
                                //.splineToLinearHeading(SWITCH,Math.toRadians(210))
                                //.setTangent(165.5)
                                //.lineTo(new Vector2d(x_Slide,y_Slide))
                                //.lineToLinearHeading(new Pose2d(x_PreloadIntf,y_PrelaodIntf,Math.toRadians(Angle_PreloadIntf)))
                             //   .lineTo(new Vector2d(x_GoTo,y_GoTo))
                               // .lineToLinearHeading(COLLECT_POSITION_5)
                               // .lineToLinearHeading(new Pose2d(x_PLACE_SOUTH_HIGH, y_PLACE_SOUTH_HIGH, Math.toRadians(Angle_PLACE_SOUTH_HIGH)))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(South_1_8)
                .start();
    }
}