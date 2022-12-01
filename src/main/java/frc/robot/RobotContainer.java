package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.SwerveCommand;
//import frc.robot.subsystems.Swervesubsystem;
import frc.robot.subsystems.sim.Swervesubsystem;

public class RobotContainer {
  private Swervesubsystem m_SwerveSubsystem;
  private XboxController m_controller;

  public RobotContainer() {
    m_controller = new XboxController(0);

    m_SwerveSubsystem = new Swervesubsystem();

    m_SwerveSubsystem.setDefaultCommand(new SwerveCommand(
      () -> m_controller.getLeftX(), 
      () -> m_controller.getLeftY(), 
      () -> m_controller.getRightX(), 
      m_SwerveSubsystem
      ));

    configureButtonBindings();
  }

  private void configureButtonBindings() {}
}