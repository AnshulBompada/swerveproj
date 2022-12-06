package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;

import java.lang.Math;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveMod;
import edu.wpi.first.math.geometry.Rotation2d;

public class Swervesubsystem extends SubsystemBase {
  private WPI_TalonSRX RightFront;
  private WPI_TalonSRX LeftFront;
  private WPI_TalonSRX RightBack;
  private WPI_TalonSRX LeftBack;
  private WPI_TalonSRX r_RightFront;
  private WPI_TalonSRX r_LeftFront;
  private WPI_TalonSRX r_RightBack;
  private WPI_TalonSRX r_LeftBack;
  private SwerveMod TopRight;
  private SwerveMod TopLeft;
  private SwerveMod BottomRight;
  private SwerveMod BottomLeft;
  private Pigeon2 gyrosensor;

  public Swervesubsystem() {
      RightFront = new WPI_TalonSRX(1);
      LeftFront = new WPI_TalonSRX(2);
      RightBack = new WPI_TalonSRX(3);
      LeftBack = new WPI_TalonSRX(4);
      r_RightFront = new WPI_TalonSRX(5);
      r_LeftFront = new WPI_TalonSRX(6);
      r_RightBack = new WPI_TalonSRX(7);
      r_LeftBack = new WPI_TalonSRX(8);

      RightFront.setNeutralMode(NeutralMode.Brake);
      RightBack.setNeutralMode(NeutralMode.Brake);
      LeftFront.setNeutralMode(NeutralMode.Brake);
      LeftBack.setNeutralMode(NeutralMode.Brake);
      r_RightFront.setNeutralMode(NeutralMode.Brake);
      r_RightBack.setNeutralMode(NeutralMode.Brake);
      r_LeftFront.setNeutralMode(NeutralMode.Brake);
      r_LeftBack.setNeutralMode(NeutralMode.Brake);
      
      TopRight = new SwerveMod(RightFront, r_RightFront);
      TopLeft = new SwerveMod(LeftFront, r_LeftFront);
      BottomRight = new SwerveMod(RightBack, r_RightBack);
      BottomLeft = new SwerveMod(LeftBack, r_LeftBack);      
  }

  public void swerve_mode(double x_speed, double y_speed, double orientation) {
    if(-0.2 <= x_speed && x_speed <= 0.2 && y_speed <= 0.2 && -0.2 <= y_speed) {
      TopRight.control(-orientation, 45);
      TopLeft.control(orientation, 45);
      BottomRight.control(-orientation, 45);
      BottomLeft.control(orientation, 45);
      //Create code for setting the motrs to an anlge
    }
    else if(orientation == 0) {
      Vector2d sped_vector = new Vector2d(x_speed, y_speed);
      Rotation2d rot_vector = new Rotation2d(x_speed, y_speed);
      double magnitude = sped_vector.magnitude();
      double sign = Math.signum(y_speed);
      double u_magnitude = sign * magnitude;
      double rotation = rot_vector.getDegrees();

      TopRight.control(u_magnitude, rotation);
      BottomRight.control(u_magnitude, rotation);
      TopLeft.control(u_magnitude, rotation);
      BottomLeft.control(u_magnitude, rotation);


      //Create code for setting the motors to an angle
    }
    //Work out code for rotating while translation
    else {
      Vector2d sped_vector = new Vector2d(x_speed, y_speed);
      Rotation2d rot_vector = new Rotation2d(x_speed, y_speed);
      double magnitude = sped_vector.magnitude();
      double sign = Math.signum(y_speed);
      double u_magnitude = sign * magnitude;
      double rotation = rot_vector.getDegrees();
      if(gyrosensor.getYaw() < 0 && orientation > 0) {
        TopLeft.control(orientation, 45);
        BottomRight.control(orientation, 45);
        BottomLeft.control(u_magnitude, rotation);
        TopRight.control(u_magnitude, rotation);
      }
      if(gyrosensor.getYaw() < 0 && orientation < 0) {
        TopRight.control(orientation, 45);
        BottomLeft.control(orientation, 45);
        BottomRight.control(u_magnitude, rotation);
        TopLeft.control(u_magnitude, rotation);
      }
    }
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}