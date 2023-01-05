package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import java.lang.Math;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveMod;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
  private Pigeon2 gyro;

  public Swervesubsystem() {
      RightFront = new WPI_TalonFX(12);
      LeftFront = new WPI_TalonFX(11);
      RightBack = new WPI_TalonFX(14);
      LeftBack = new WPI_TalonFX(13);
      r_RightFront = new WPI_TalonFX(22);
      r_LeftFront = new WPI_TalonFX(21);
      r_RightBack = new WPI_TalonFX(24);
      r_LeftBack = new WPI_TalonFX(23);

      RightFront.setNeutralMode(NeutralMode.Brake);
      RightBack.setNeutralMode(NeutralMode.Brake);
      LeftFront.setNeutralMode(NeutralMode.Brake);
      LeftBack.setNeutralMode(NeutralMode.Brake);
      r_RightFront.setNeutralMode(NeutralMode.Brake);
      r_RightBack.setNeutralMode(NeutralMode.Brake);
      r_LeftFront.setNeutralMode(NeutralMode.Brake);
      r_LeftBack.setNeutralMode(NeutralMode.Brake);

      gyro = new Pigeon2(0);

      gyro.setYaw(0);
      
      TopRight = new SwerveMod(RightFront, r_RightFront, 6);
      TopLeft = new SwerveMod(LeftFront, r_LeftFront, 5);
      BottomRight = new SwerveMod(RightBack, r_RightBack, 8);
      BottomLeft = new SwerveMod(LeftBack, r_LeftBack, 7);      
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


      Rotation2d RU = new Rotation2d(0);
      Rotation2d RD = new Rotation2d(0);
      Rotation2d LU = new Rotation2d(0);
      Rotation2d LD = new Rotation2d(0);
      Rotation2d rotation = new Rotation2d(Math.PI * joystick_rotation);
//      Rotation2d robotori = new Rotation2d(-gyro.getYaw());

      RU = RU.rotateBy(new Rotation2d(-orientation*(Math.PI/4)));
      RD = RD.rotateBy(new Rotation2d(-orientation*(Math.PI/4)));
      LU = LU.rotateBy(new Rotation2d(orientation*(Math.PI/4)));
      LD = LD.rotateBy(new Rotation2d(orientation*(Math.PI/4)));

      RU = RU.rotateBy(rotation);
      RD = RD.rotateBy(rotation);
      LU = LU.rotateBy(rotation);
      LD = LD.rotateBy(rotation);
/*
      RU.rotateBy(robotori);
      RD.rotateBy(robotori);
      LU.rotateBy(robotori);
      LD.rotateBy(robotori);
*/

    SmartDashboard.putNumber("Top left wheel degrees", TopLeft.getwheeldegs());
    SmartDashboard.putNumber("Bottom Left wheel degrees", BottomLeft.getwheeldegs());
    SmartDashboard.putNumber("Top Right wheel degrees", TopRight.getwheeldegs());
    SmartDashboard.putNumber("Bottom Right wheel degrees", BottomRight.getwheeldegs());
    SmartDashboard.putNumber("RU_Degrees", RU.getDegrees());
    SmartDashboard.putNumber("RD_Degrees", RD.getDegrees());
    SmartDashboard.putNumber("LU_Degrees", LU.getDegrees());
    SmartDashboard.putNumber("LD_Degrees", LD.getDegrees());
    SmartDashboard.putNumber("hi magn", u_magnitude);
    SmartDashboard.putNumber("lo magn", u_lmagnitude);

    double TR_deg = optimizeAzimuthPath(RU.getDegrees(), TopRight.getwheeldegs());
    double TL_deg = optimizeAzimuthPath(LU.getDegrees(), TopLeft.getwheeldegs());
    double DR_deg = optimizeAzimuthPath(RD.getDegrees(), BottomRight.getwheeldegs());
    double DL_deg = optimizeAzimuthPath(LD.getDegrees(), BottomLeft.getwheeldegs());

    if(orientation > 0){
      TopRight.control(u_magnitude, TR_deg);
      TopLeft.control(u_magnitude, TL_deg);
      BottomRight.control(u_lmagnitude, DR_deg);
      BottomLeft.control(u_lmagnitude, DL_deg);
      }
    if(orientation < 0){
        TopRight.control(u_lmagnitude, TR_deg);
        TopLeft.control(u_lmagnitude, TL_deg);
        BottomRight.control(u_magnitude, DR_deg);
        BottomLeft.control(u_magnitude, DL_deg);
      }
    }
  }

  public void stop() {
    TopRight.control(0, -45);
    TopLeft.control(0, 45);
    BottomRight.control(0, 45);
    BottomLeft.control(0, -45);
  }

  private double optimizeAzimuthPath (double target, double actual) {
    if (Math.min(Math.min(Math.abs(target - TopLeft.getwheeldegs()), Math.abs((target + 360) - actual)), Math.abs((target - 360) - actual)) == Math.abs((target + 360) - actual))
      target += 360;
    if (Math.min(Math.min(Math.abs(target - TopLeft.getwheeldegs()), Math.abs((target + 360) - actual)), Math.abs((target - 360) - actual)) == Math.abs((target - 360) - actual))
      target -= 360;
    return target;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}