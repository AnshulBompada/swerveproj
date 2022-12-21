package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

public class SwerveMod {
//    private double global_rotation;
    private WPI_TalonFX m_speed;
    private WPI_TalonFX m_rotation;
    private WPI_CANCoder rot_encoder;
//    private PWMSparkMax sim_speed;
//    private PWMSparkMax sim_rotation;
//    private AbsoluteSensorRange angle_range;
    private CANCoderConfiguration config;
    private StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT;

    public SwerveMod(WPI_TalonFX speed, WPI_TalonFX rotation, int encoder_port) {
        m_speed = speed;
        m_rotation = rotation;
        rot_encoder = new WPI_CANCoder(m_rotation.getDeviceID());
        config = new CANCoderConfiguration();
        config.sensorCoefficient = 360 / 4096.0;
        config.unitString = "deg";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        rot_encoder.configAllSettings(config);

        DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration();
        
        m_speed.configFactoryDefault();
        m_speed.setInverted(TalonFXInvertType.CounterClockwise);
        m_speed.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
        m_speed.config_kP(0, 0.044057);
        m_speed.config_kF(0, 0.028998);

        m_rotation.configFactoryDefault();
        m_rotation.setInverted(TalonFXInvertType.CounterClockwise);
        m_rotation.configRemoteFeedbackFilter(rot_encoder, 0);
        m_rotation.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        m_rotation.setSelectedSensorPosition(rot_encoder.getAbsolutePosition());
        m_rotation.config_kP(0, 0.2);
        m_rotation.config_kD(0, 0.01);

        rot_encoder.configFactoryDefault();
        rot_encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        rot_encoder.setPositionToAbsolute();
    }

/*    public SwerveMod(PWMSparkMax speed, PWMSparkMax rotation, int encoder_port) {
      sim_speed = speed;
      sim_rotation = rotation;
    }*/

    public void control(double speed, double rotation) {
      accturntoang(rotation * 180);
      m_speed.set(speed);
    }

    public void accturntoang(double rotationdegrees) {
      double act_position = ((m_rotation.getSelectedSensorPosition()/  4096) * 180) % 180;
      optimizeAzimuthPath(rotationdegrees, act_position);

      m_rotation.set(ControlMode.Position, ((rotationdegrees + (act_position - (act_position % 180))) / 180) * 4096);
    }

    private double optimizeAzimuthPath (double target, double actual) {
      if (Math.min(Math.min(Math.abs(target - actual), Math.abs((target + 180) - actual)), Math.abs((target - 180) - actual)) == Math.abs((target + 180) - actual))
        target += 180;
      if (Math.min(Math.min(Math.abs(target - actual), Math.abs((target + 180) - actual)), Math.abs((target - 180) - actual)) == Math.abs((target - 180) - actual))
        target -= 180;
      return target;
    }

    public void turntoang(double rotationdegrees) {
      double r = 10;
      double error;
      while(rotationdegrees - r < rot_encoder.getAbsolutePosition() || rotationdegrees + r > rot_encoder.getAbsolutePosition()) {
        error = rotationdegrees - rot_encoder.getAbsolutePosition();
        if(Math.abs(error) > 180) {
          m_rotation.set(-1);
        }
        if(Math.abs(error) < 180) {
          m_rotation.set(1);
        }
        m_rotation.set(0);
      }
    }

    public void dumbturntoang(double rotationdegrees) {
      double range = 0.2;
      double error;
      while(rotationdegrees - range < rot_encoder.getAbsolutePosition() || rotationdegrees + range > rot_encoder.getAbsolutePosition()) {
        error = rot_encoder.getAbsolutePosition() - rotationdegrees;
        if(Math.abs(error) > 180) {
          m_rotation.set(-dumbcalc(error));
        }
        if(Math.abs(error) < 180) {
          m_rotation.set(dumbcalc(error));
        }
      }
    }

    public double dumbcalc(double degrees) {
      if(Math.abs(degrees) > 180) {
        return (2/Math.PI) * Math.atan(Math.toRadians(Math.abs(degrees) - 180));
      }
      return (2/Math.PI) * Math.atan(Math.toRadians(degrees));
    }

    public double convert(double degrees) {
      double ticks = degrees * 4096/180;
      return ticks;
    }

/*  
    public void turntime(WPI_TalonFX motor, double speed, double time){
      Timer timer = new Timer();
      timer.start();
      while(timer.get() < time) {
        motor.set(speed);
  }
      motor.set(0);
  }

  public void turntime(PWMSparkMax motor, double speed, double time){
    Timer timer = new Timer();
    timer.start();
    while(timer.get() < time) {
      motor.set(speed);
  }
    motor.set(0);
  }

  public void setposition(double pos) {
    rot_encoder.setPosition(pos);
  }
  
  public void simcontrol(double speed, double rotation) {
    turntoang(sim_rotation, rotation * 180);
    sim_speed.set(speed);
  }

  public void turntoang(WPI_TalonFX motor, double rotationdegrees) {
//      double time = (75 / (13*180))/rotationdegrees - (75 / (13*180))* global_rotation;
//      double speed =1;
/*      if(time < 0) {
      time = -time;
      speed = -speed;
    }
    turntime(motor, speed, time);
    global_rotation =+ rotationdegrees;
  }

  public void turntoang(PWMSparkMax motor, double rotationdegrees) {
  double time = (75 / (13*180))/rotationdegrees - (75 / (13*180))* global_rotation;
  double speed =1;
  if(time < 0) {
    time = -time;
    speed = -speed;
  }
  turntime(motor, speed, time);
  global_rotation =+ rotationdegrees;
  }
*/
}