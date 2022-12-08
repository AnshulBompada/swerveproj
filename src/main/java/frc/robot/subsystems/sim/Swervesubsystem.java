package frc.robot.subsystems.sim;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import java.lang.Math;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveMod;
import edu.wpi.first.math.geometry.Rotation2d;
//This module is only from simulation
public class Swervesubsystem extends SubsystemBase {
  private PWMSparkMax RightFront;
  private PWMSparkMax LeftFront;
  private PWMSparkMax RightBack;
  private PWMSparkMax LeftBack;
  private PWMSparkMax r_RightFront;
  private PWMSparkMax r_LeftFront;
  private PWMSparkMax r_RightBack;
  private PWMSparkMax r_LeftBack;
  private SwerveMod TopRight;
  private SwerveMod TopLeft;
  private SwerveMod BottomRight;
  private SwerveMod BottomLeft;
//  private Pigeon2 gyrosensor;

  public Swervesubsystem() {
      RightFront = new PWMSparkMax(1);
      LeftFront = new PWMSparkMax(2);
      RightBack = new PWMSparkMax(3);
      LeftBack = new PWMSparkMax(4);
      r_RightFront = new PWMSparkMax(5);
      r_LeftFront = new PWMSparkMax(6);
      r_RightBack = new PWMSparkMax(7);
      r_LeftBack = new PWMSparkMax(8);
      
      TopRight = new SwerveMod(RightFront, r_RightFront);
      TopLeft = new SwerveMod(LeftFront, r_LeftFront);
      BottomRight = new SwerveMod(RightBack, r_RightBack);
      BottomLeft = new SwerveMod(LeftBack, r_LeftBack);      
  }

  public void swerve_mode(double x_speed, double y_speed, double orientation) {
    if(x_speed == 0 & y_speed == 0) {
      TopRight.simcontrol(orientation, 45);
      BottomRight.simcontrol(-orientation, 45);
      TopLeft.simcontrol(-orientation, 45);
      BottomLeft.simcontrol(orientation, 45);
    }

    if(orientation == 0) {
      Vector2d sped_vector = new Vector2d(x_speed, y_speed);
      Rotation2d rot_vector = new Rotation2d(x_speed, y_speed);
      double magnitude = sped_vector.magnitude();
      double sign = Math.signum(y_speed);
      double u_magnitude = sign * magnitude;
      double rotation = rot_vector.getDegrees();

      TopRight.simcontrol(u_magnitude, rotation);
      BottomRight.simcontrol(u_magnitude, rotation);
      TopLeft.simcontrol(u_magnitude, rotation);
      BottomLeft.simcontrol(u_magnitude, rotation);
    }
    //Work out code for rotating while translation
    else {
      Vector2d sped_vector = new Vector2d(x_speed, y_speed);
      Rotation2d rot_vector = new Rotation2d(x_speed, y_speed);
      double magnitude = sped_vector.magnitude();
      double sign = Math.signum(y_speed);
      double u_magnitude = sign * magnitude;
      double rotation = rot_vector.getDegrees();
      if(orientation > 0) {
        TopLeft.simcontrol(orientation, 45);
        BottomRight.simcontrol(orientation, 45);
        BottomLeft.simcontrol(u_magnitude, rotation);
        TopRight.simcontrol(u_magnitude, rotation);
      }
      if(orientation < 0) {
        TopRight.simcontrol(orientation, 45);
        BottomLeft.simcontrol(orientation, 45);
        BottomRight.simcontrol(u_magnitude, rotation);
        TopLeft.simcontrol(u_magnitude, rotation);
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