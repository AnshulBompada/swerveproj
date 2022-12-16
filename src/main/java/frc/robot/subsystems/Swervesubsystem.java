package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.sensors.Pigeon2;
import java.lang.Math;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveMod;
import edu.wpi.first.math.geometry.Rotation2d;

public class Swervesubsystem extends SubsystemBase {
  private WPI_TalonFX RightFront;
  private WPI_TalonFX LeftFront;
  private WPI_TalonFX RightBack;
  private WPI_TalonFX LeftBack;
  private WPI_TalonFX r_RightFront;
  private WPI_TalonFX r_LeftFront;
  private WPI_TalonFX r_RightBack;
  private WPI_TalonFX r_LeftBack;
  private SwerveMod TopRight;
  private SwerveMod TopLeft;
  private SwerveMod BottomRight;
  private SwerveMod BottomLeft;
//  private Pigeon2 gyrosensor;

  public Swervesubsystem() {
      RightFront = new WPI_TalonFX(1);
      LeftFront = new WPI_TalonFX(2);
      RightBack = new WPI_TalonFX(3);
      LeftBack = new WPI_TalonFX(4);
      r_RightFront = new WPI_TalonFX(5);
      r_LeftFront = new WPI_TalonFX(6);
      r_RightBack = new WPI_TalonFX(7);
      r_LeftBack = new WPI_TalonFX(8);

      RightFront.setNeutralMode(NeutralMode.Brake);
      RightBack.setNeutralMode(NeutralMode.Brake);
      LeftFront.setNeutralMode(NeutralMode.Brake);
      LeftBack.setNeutralMode(NeutralMode.Brake);
      r_RightFront.setNeutralMode(NeutralMode.Brake);
      r_RightBack.setNeutralMode(NeutralMode.Brake);
      r_LeftFront.setNeutralMode(NeutralMode.Brake);
      r_LeftBack.setNeutralMode(NeutralMode.Brake);
      
      TopRight = new SwerveMod(RightFront, r_RightFront, 1);
      TopLeft = new SwerveMod(LeftFront, r_LeftFront, 2);
      BottomRight = new SwerveMod(RightBack, r_RightBack, 3);
      BottomLeft = new SwerveMod(LeftBack, r_LeftBack, 4);      
  }

  public void swerve_mode(double x_speed, double y_speed, double orientation) {
    if(x_speed == 0 && y_speed == 0 && orientation == 0) {
      TopRight.control(-orientation, 45);
      TopLeft.control(orientation, 45);
      BottomRight.control(-orientation, 45);
      BottomLeft.control(orientation, 45);
    }
    else {
      double u_magnitude = Math.sqrt(Math.pow(x_speed, 2) + Math.pow(y_speed, 2));
      double u_lmagnitude = u_magnitude - 0.75 * u_magnitude * orientation;
      double joystick_rotation = Math.atan2(x_speed, y_speed);
      Rotation2d RU = new Rotation2d(u_magnitude, 0);
      Rotation2d RD = new Rotation2d(u_magnitude, 0);
      Rotation2d LU = new Rotation2d(u_magnitude, 0);
      Rotation2d LD = new Rotation2d(u_magnitude, 0);
      Rotation2d rotation = new Rotation2d(Math.PI * joystick_rotation);

      RU.rotateBy(new Rotation2d(-orientation*(Math.PI/4)));
      RD.rotateBy(new Rotation2d(-orientation*(Math.PI/4)));
      LU.rotateBy(new Rotation2d(orientation*(Math.PI/4)));
      LD.rotateBy(new Rotation2d(orientation*(Math.PI/4)));

      RU.rotateBy(rotation);
      RD.rotateBy(rotation);
      LU.rotateBy(rotation);
      LD.rotateBy(rotation);
    if(orientation > 0){
      TopRight.control(u_magnitude, RU.getDegrees());
      TopLeft.control(u_magnitude, LU.getDegrees());
      BottomRight.control(u_lmagnitude, RD.getDegrees());
      BottomLeft.control(u_lmagnitude, LD.getDegrees());
      }
    if(orientation < 0){
        TopRight.control(u_lmagnitude, RU.getDegrees());
        TopLeft.control(u_lmagnitude, LU.getDegrees());
        BottomRight.control(u_magnitude, RD.getDegrees());
        BottomLeft.control(u_magnitude, LD.getDegrees());
      }
    }
  }

  public void stop() {
    TopRight.control(0, 0);
    TopLeft.control(0, 0);
    BottomRight.control(0, 0);
    BottomLeft.control(0, 0);
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}