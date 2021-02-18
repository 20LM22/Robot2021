package frc.robot.commands.drivecommands;

import java.util.List;
import java.util.function.Supplier;

import java.nio.file.Path;
import java.io.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.ArduinoSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.CarouselConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.armcommands.BounceArmCommand;
import frc.robot.commands.carouselcommands.RunCarouselCommand;
import frc.robot.commands.intakecommands.IntakeCommand;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.DriverStation;

public class PixyGalacticCommand extends SequentialCommandGroup {

    private final ArduinoSubsystem m_arduinoSubsystem;
    private final DriveSubsystem m_driveSubsystem;
    private final Trajectory m_trajectory;

    // TODO- change values here
    double aredx = 103;
    double abluex = 200;
    double bredx = 100;
    double bbluex = 180;
    double threshhold = 20;

    public PixyGalacticCommand(ArduinoSubsystem arduinoSubsystem, DriveSubsystem driveSubsystem, Trajectory trajectoryBlue, Trajectory trajectoryRed) {
      
        m_arduinoSubsystem = arduinoSubsystem;
        m_driveSubsystem = driveSubsystem;

        m_arduinoSubsystem.update();
        // System.out.println("the x value is: " + m_arduinoSubsystem.getXValue());
        // if (m_arduinoSubsystem.getXValue() >= aredx - threshhold
        //         && m_arduinoSubsystem.getXValue() <= aredx + threshhold) { //the target is in the acceptable range --> run red path
        //     m_trajectory = trajectoryRed;
        // } else { //run the blue path
        //     m_trajectory = trajectoryRed;
        // }

        if (m_arduinoSubsystem.getTargetInView()) { //the target is in the acceptable range --> run red path
            m_trajectory = trajectoryRed;
            System.out.println("HERE");
        } else { //run the blue path
            m_trajectory = trajectoryBlue;
            System.out.println("This is what I'm running.");
        }
        
        addRequirements(m_driveSubsystem, m_arduinoSubsystem);
        addCommands(new TrajectoryFollowCommand(m_driveSubsystem, m_trajectory));
        
    }

    public void end() { // TODO this might cause problems so might need to take it out later
        m_driveSubsystem.tankDrive(0, 0);
    }

}