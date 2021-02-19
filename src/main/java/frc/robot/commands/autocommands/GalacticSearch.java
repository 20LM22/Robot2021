// package frc.robot.commands.autocommands;

// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import frc.robot.Constants.CarouselConstants;
// import frc.robot.commands.armcommands.BounceArmCommand;
// import frc.robot.commands.carouselcommands.RunCarouselCommand;
// import frc.robot.commands.drivecommands.TrajectoryFollowCommand;
// import frc.robot.commands.intakecommands.IntakeCommand;
// import frc.robot.subsystems.ArduinoSubsystem;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.CarouselSubsystem;
// import frc.robot.subsystems.DriveSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;

// public class GalacticSearch extends CommandBase { //not using this command --> see pixy galatic command

//     private final DriveSubsystem m_driveSubsystem;
//     private final ArduinoSubsystem m_arduinoSubsystem;
//     private final IntakeSubsystem m_intakeSubsystem;
//     private final CarouselSubsystem m_carouselSubsystem;
//     private final ArmSubsystem m_armSubsystem;
//     private final Trajectory m_trajectoryBlue;
//     private final Trajectory m_trajectoryRed;

//     /**
//      * Shoot balls move forward off init
//      */
//     public GalacticSearch(DriveSubsystem driveSubsystem, ArduinoSubsystem arduinoSubsystem,
//             Trajectory trajectoryRed, Trajectory trajectoryBlue) {

//         m_arduinoSubsystem = arduinoSubsystem;
//         m_driveSubsystem = driveSubsystem;
//         m_trajectoryRed = trajectoryRed;
//         m_trajectoryBlue = trajectoryBlue;
//         addRequirements(m_driveSubsystem, m_arduinoSubsystem, m_intakeSubsystem, m_carouselSubsystem, m_armSubsystem);
//     }

//     public void execute() {
//         // m_driveSubsystem.tankDrive(.25, .25);
//         m_arduinoSubsystem.update();
//         // if target is in the camera's view
//         if (m_arduinoSubsystem.getTargetInView()) { // this means that the robot sees the red ball and therefore we
//                                                     // should run the red path
//             // TODO ask Chris or Andrew about how to find the size of the ball in the frame
//             // so that we only see a ball if it's quite close to the robot
//             new ParallelDeadlineGroup(new TrajectoryFollowCommand(m_driveSubsystem, m_trajectoryRed),
//                     new ParallelCommandGroup(new IntakeCommand(m_intakeSubsystem),
//                             new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kIntakeVelocity),
//                             new BounceArmCommand(m_armSubsystem)));

//             // new TrajectoryFollowCommand(m_driveSubsystem, m_trajectoryRed); //this is
//             // without the intake running
//         } else { // so if we didn't see the red ball then that means we must be running the blue
//                  // path
//             new ParallelDeadlineGroup(new TrajectoryFollowCommand(m_driveSubsystem, m_trajectoryBlue),
//                     new ParallelCommandGroup(new IntakeCommand(m_intakeSubsystem),
//                             new RunCarouselCommand(m_carouselSubsystem, CarouselConstants.kIntakeVelocity),
//                             new BounceArmCommand(m_armSubsystem)));

//             // new TrajectoryFollowCommand(m_driveSubsystem, m_trajectoryBlue);
//         }
//     }

//     public void end() { // TODO this might cause problems so might need to take it out later
//         m_driveSubsystem.tankDrive(0, 0);
//     }

// }
