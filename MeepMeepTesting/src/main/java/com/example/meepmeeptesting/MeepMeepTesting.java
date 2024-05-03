package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width

                .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 14.75)
                .setDimensions(16.75, 17)
                .setStartPose(new Pose2d(12, 62, Math.toRadians(-90)))
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 62, Math.toRadians(-90)))
                                .splineToSplineHeading(new Pose2d(48, 44, Math.toRadians(0)), Math.toRadians(0))
                                .lineToLinearHeading(new Pose2d(41, 37, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(21, 40, Math.toRadians(-90)), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(21, 45, Math.toRadians(-90)))
                                .lineToLinearHeading(new Pose2d(12, 60, Math.toRadians(180)))
                                .splineToConstantHeading(new Vector2d(-50, 60), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-58, 35), Math.toRadians(180))
                                .lineToLinearHeading(new Pose2d(-62, 35, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-50, 60, Math.toRadians(0)))
                                .splineToConstantHeading(new Vector2d(12, 60), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(48, 44), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}