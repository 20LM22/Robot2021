// package frc.robot.commands.drivecommands;

// import java.util.List;
// import java.util.function.Supplier;

// import java.nio.file.Path;
// import java.io.*;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import frc.robot.subsystems.ArduinoSubsystem;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.CarouselSubsystem;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import frc.robot.Constants.CarouselConstants;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.commands.armcommands.BounceArmCommand;
// import frc.robot.commands.carouselcommands.RunCarouselCommand;
// import frc.robot.commands.intakecommands.IntakeCommand;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj.DriverStation;

// public class PixyGalacticCommand extends CommandBase {

//     private final ArduinoSubsystem m_arduinoSubsystem;
//     private final DriveSubsystem m_driveSubsystem;
//     private final IntakeSubsystem m_intakeSubsystem;
//     private final CarouselSubsystem m_carouselSubsystem;
//     private final ArmSubsystem m_armSubsystem;

//     // TODO- change values here
//     double aredx = 103;
//     double abluex = 200;
//     double bredx = 100;
//     double bbluex = 180;
//     double threshhold = 20;

//     String trajectoryJSONared = "paths/GalacticSearchARed.wpilib.json";
//     String trajectoryJSONablue = "paths/GalacticSearchABlue.wpilib.json";
//     String trajectoryJSONbred = "paths/GalacticSearchBRed.wpilib.json";
//     String trajectoryJSONbblue = "paths/GalacticSearchBBlue.wpilib.json";
//     String path = "";

//     Trajectory trajectory = new Trajectory();

//     public PixyGalacticCommand(ArduinoSubsystem arduinoSubsystem, DriveSubsystem driveSubsystem,
//             IntakeSubsystem intakeSubsystem, CarouselSubsystem carouselSubsystem, ArmSubsystem armSubsystem) {
//         m_arduinoSubsystem = arduinoSubsystem;
//         m_driveSubsystem = driveSubsystem;
//         m_intakeSubsystem = intakeSubsystem;
//         m_carouselSubsystem = carouselSubsystem;
//         m_armSubsystem = armSubsystem;
//         addRequirements(m_driveSubsystem, m_arduinoSubsystem);
//     }

//     public void initialize() {
//         m_arduinoSubsystem.update();
//         // if (m_arduinoSubsystem.getXValue() >= aredx - threshhold
//         //         && m_arduinoSubsystem.getXValue() <= aredx + threshhold) {
//         //     path = trajectoryJSONared;
//         // } else if (m_arduinoSubsystem.getXValue() >= abluex - threshhold
//         //         && m_arduinoSubsystem.getXValue() <= abluex + threshhold) {
//         //     path = trajectoryJSONablue;

//         // } else if (m_arduinoSubsystem.getXValue() >= bredx - threshhold
//         //         && m_arduinoSubsystem.getXValue() <= bredx + threshhold) {
//         //     path = trajectoryJSONbred;

//         // } else if (m_arduinoSubsystem.getXValue() >= bbluex - threshhold
//         //         && m_arduinoSubsystem.getXValue() <= bbluex + threshhold) {
//         //     path = trajectoryJSONbblue;
//         // }
//         // try {
//         //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
//         //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//         //     Command movePath = new TrajectoryFollowCommand(m_driveSubsystem, trajectory);

//         // } catch (IOException ex) {
//         //     DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
//         // }

//         // new ParallelDeadlineGroup(new TrajectoryFollowCommand(m_driveSubsystem, trajectory),
//         //         new ParallelCommandGroup(new IntakeCommand(m_intakeSubsystem),
//         //                 new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kIntakeVelocity),
//         //                 new BounceArmCommand(m_armSubsystem)));

//         // new ParallelDeadlineGroup( new TrajectoryFollowCommand(m_driveSubsystem, TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(
//         //     new Translation2d(.5, 0)), new Pose2d(1, 0, new Rotation2d()), DriveConstants.kTrajectoryConfig));,
//         //         new RunCarouselCommand(m_carouselSubsystem, 5));

//         if (m_arduinoSubsystem.getXValue() >= aredx - threshhold
//                 && m_arduinoSubsystem.getXValue() <= aredx + threshhold) {
//                     new ParallelDeadlineGroup( new TrajectoryFollowCommand(m_driveSubsystem, TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(
//                         new Translation2d(.5, 0)), new Pose2d(1, 0, new Rotation2d()), DriveConstants.kTrajectoryConfig));,
//                             new RunCarouselCommand(m_carouselSubsystem, 5));
        
//         } else if (m_arduinoSubsystem.getXValue() >= abluex - threshhold
//                 && m_arduinoSubsystem.getXValue() <= abluex + threshhold) {
//                     new ParallelDeadlineGroup( new TrajectoryFollowCommand(m_driveSubsystem, TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(
//                         new Translation2d(.5, 0)), new Pose2d(1, 0, new Rotation2d()), DriveConstants.kTrajectoryConfig));,
//                             new RunCarouselCommand(m_carouselSubsystem, 5));
            
//         } else if (m_arduinoSubsystem.getXValue() >= bredx - threshhold
//                 && m_arduinoSubsystem.getXValue() <= bredx + threshhold) {
//                     new ParallelDeadlineGroup( new TrajectoryFollowCommand(m_driveSubsystem, TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(
//                         new Translation2d(.5, 0)), new Pose2d(1, 0, new Rotation2d()), DriveConstants.kTrajectoryConfig));,
//                             new RunCarouselCommand(m_carouselSubsystem, 5));
            
//         } else if (m_arduinoSubsystem.getXValue() >= bbluex - threshhold
//                 && m_arduinoSubsystem.getXValue() <= bbluex + threshhold) {
//                     new ParallelDeadlineGroup( new TrajectoryFollowCommand(m_driveSubsystem, TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(
//                         new Translation2d(.5, 0)), new Pose2d(1, 0, new Rotation2d()), DriveConstants.kTrajectoryConfig));,
//                             new RunCarouselCommand(m_carouselSubsystem, 5));
//                     }

//     }

//     public void end() { // TODO this might cause problems so might need to take it out later
//         m_driveSubsystem.tankDrive(0, 0);
//     }

// }