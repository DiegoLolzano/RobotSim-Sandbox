package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class Robot extends TimedRobot {
  // ==========================================
  // 1. HARDWARE
  // ==========================================
  private final PWMSparkMax motorElevador = new PWMSparkMax(0);
  private final PWMSparkMax motorBrazo = new PWMSparkMax(1);
  private final PWMSparkMax motorShooter = new PWMSparkMax(2);
  private final Joystick joystick = new Joystick(0);

  private final PWMSparkMax motorIzq = new PWMSparkMax(3);
  private final PWMSparkMax motorDer = new PWMSparkMax(4);
  private final Encoder encIzq = new Encoder(0, 1);
  private final Encoder encDer = new Encoder(2, 3);
  private final AnalogGyro gyro = new AnalogGyro(0);
  private final DifferentialDrive chasisTank = new DifferentialDrive(motorIzq::set, motorDer::set);

  // ==========================================
  // 2. MODELOS FÍSICOS (WPILib 2026)
  // ==========================================
  private final ElevatorSim simElevador = new ElevatorSim(
      DCMotor.getNEO(1), 10.0, 5.0, 0.05, 0.0, 1.4, true, 0.0);

  private final SingleJointedArmSim simBrazo = new SingleJointedArmSim(
      DCMotor.getNEO(1), 100.0, SingleJointedArmSim.estimateMOI(0.7, 5.0),
      0.7, Units.degreesToRadians(-75), Units.degreesToRadians(110), true, 0.0);

  private final FlywheelSim simShooter = new FlywheelSim(
      LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.005, 1.0),
      DCMotor.getNEO(1));

  private final EncoderSim simEncIzq = new EncoderSim(encIzq);
  private final EncoderSim simEncDer = new EncoderSim(encDer);
  private final AnalogGyroSim simGyro = new AnalogGyroSim(gyro);
  private final DifferentialDrivetrainSim simFisicaTank = new DifferentialDrivetrainSim(
      DCMotor.getNEO(2), 7.29, 7.5, 60.0, Units.inchesToMeters(3), 0.7112, null);

  private boolean isSwerveMode = false;
  private Pose2d poseSwerveVirtual = new Pose2d(8.0, 4.0, new Rotation2d());

  // ==========================================
  // 3. WIDGET 1: ELEVADOR DETALLADO
  // ==========================================
  private final Mechanism2d widgetElevador = new Mechanism2d(1.4, 2.0);
  private final MechanismLigament2d etapaMovilElevador;
  private final MechanismLigament2d carroElevador;

  // ==========================================
  // 4. WIDGET 2: BRAZO CON GARRA DETALLADO
  // ==========================================
  private final Mechanism2d widgetBrazo = new Mechanism2d(2.0, 2.0);
  private final MechanismLigament2d boomBrazo;

  // ==========================================
  // 5. WIDGET 3: SHOOTER DOBLE RUEDA CON ASPAS
  // ==========================================
  private final Mechanism2d widgetShooter = new Mechanism2d(1.5, 1.5);
  private final MechanismLigament2d[] aspasInferiores = new MechanismLigament2d[4];
  private final MechanismLigament2d[] aspasSuperiores = new MechanismLigament2d[4];
  private double anguloAcumuladoShooter = 0.0;

  // ==========================================
  // 6. WIDGET 4: CANCHA 2D
  // ==========================================
  private final Field2d canchaVirtual = new Field2d();

  public Robot() {
    // --- CONSTRUCCIÓN VISUAL DEL ELEVADOR ---
    // Base del robot (Gris)
    MechanismRoot2d baseElev = widgetElevador.getRoot("BaseChasis", 0.2, 0.1);
    baseElev.append(new MechanismLigament2d("Chasis", 1.0, 0, 12, new Color8Bit(Color.kGray)));
    
    // Torre fija y topes (Gris oscuro y Rojo)
    MechanismRoot2d torreRoot = widgetElevador.getRoot("TorreFija", 0.45, 0.1);
    torreRoot.append(new MechanismLigament2d("MastilFijo", 1.6, 90, 10, new Color8Bit(Color.kDarkSlateGray)));
    
    // Etapa móvil (Azul) + Carro horizontal (Cian) + Punta de garra (Naranja)
    MechanismRoot2d carroRoot = widgetElevador.getRoot("CarroRoot", 0.52, 0.1);
    etapaMovilElevador = carroRoot.append(
        new MechanismLigament2d("EtapaMovil", 0.15, 90, 8, new Color8Bit(Color.kDodgerBlue)));
    carroElevador = etapaMovilElevador.append(
        new MechanismLigament2d("ForkHorizontal", 0.35, -90, 8, new Color8Bit(Color.kCyan)));
    carroElevador.append(
        new MechanismLigament2d("PuntaIntake", 0.12, 90, 6, new Color8Bit(Color.kOrange)));

    // --- CONSTRUCCIÓN VISUAL DEL BRAZO ---
    // Base y pedestal central (Gris)
    MechanismRoot2d baseBrazo = widgetBrazo.getRoot("BaseBrazo", 0.4, 0.15);
    baseBrazo.append(new MechanismLigament2d("ChasisBrazo", 1.2, 0, 12, new Color8Bit(Color.kGray)));
    MechanismRoot2d posteBrazo = widgetBrazo.getRoot("Pedestal", 1.0, 0.15);
    posteBrazo.append(new MechanismLigament2d("Torreta", 0.85, 90, 12, new Color8Bit(Color.kDarkSlateGray)));

    // Brazo principal (Rojo) + Muñeca y Pinzas (Cian)
    MechanismRoot2d pivoteRoot = widgetBrazo.getRoot("Pivote", 1.0, 1.0);
    boomBrazo = pivoteRoot.append(
        new MechanismLigament2d("BrazoPrincipal", 0.65, 0, 10, new Color8Bit(Color.kCrimson)));
    boomBrazo.append(new MechanismLigament2d("PinzaSuperior", 0.18, 35, 5, new Color8Bit(Color.kCyan)));
    boomBrazo.append(new MechanismLigament2d("PinzaInferior", 0.18, -35, 5, new Color8Bit(Color.kCyan)));

    // --- CONSTRUCCIÓN VISUAL DEL SHOOTER ---
    // Rampa/Capucha fija del shooter (Gris)
    MechanismRoot2d rampaShooter = widgetShooter.getRoot("RampaBase", 0.2, 0.25);
    rampaShooter.append(new MechanismLigament2d("RampaInclinada", 1.1, 25, 8, new Color8Bit(Color.kGray)));

    // Rueda principal (4 aspas para notar el giro) y rueda superior (Kicker)
    MechanismRoot2d ejePrincipal = widgetShooter.getRoot("EjeInferior", 0.75, 0.55);
    MechanismRoot2d ejeKicker = widgetShooter.getRoot("EjeSuperior", 0.60, 0.95);
    for (int i = 0; i < 4; i++) {
      aspasInferiores[i] = ejePrincipal.append(
          new MechanismLigament2d("AspaInf" + i, 0.22, i * 90, 6, new Color8Bit(Color.kOrangeRed)));
      aspasSuperiores[i] = ejeKicker.append(
          new MechanismLigament2d("AspaSup" + i, 0.12, i * 90, 4, new Color8Bit(Color.kDeepSkyBlue)));
    }
  }

  @Override
  public void robotInit() {
    // Publicamos cada mecanismo como un widget independiente
    SmartDashboard.putData("1. Sim Elevador", widgetElevador);
    SmartDashboard.putData("2. Sim Brazo Pivote", widgetBrazo);
    SmartDashboard.putData("3. Sim Shooter", widgetShooter);
    SmartDashboard.putData("4. Cancha FRC (Vista Superior)", canchaVirtual);
    SmartDashboard.putBoolean("Modo Swerve Activo", isSwerveMode);
  }

  @Override
  public void teleopPeriodic() {
    // Botón 2 (B) cambia entre Tank y Swerve
    if (joystick.getRawButtonPressed(2)) {
      isSwerveMode = !isSwerveMode;
      SmartDashboard.putBoolean("Modo Swerve Activo", isSwerveMode);
    }

    double velAdelante = -joystick.getRawAxis(1); // W/S
    double velGiro = -joystick.getRawAxis(4);     // A/D o Flechas
    double velLateral = -joystick.getRawAxis(0);  // Strafe para Swerve

    if (!isSwerveMode) {
      chasisTank.arcadeDrive(velAdelante, velGiro);
    } else {
      chasisTank.stopMotor();
    }

    // Elevador: POV (Cruceta) o Axis 2 (E/R en tu teclado configurado)
    if (joystick.getPOV() == 0 || joystick.getRawButton(3)) {
      motorElevador.set(0.6);
    } else if (joystick.getPOV() == 180 || joystick.getRawButton(4)) {
      motorElevador.set(-0.4);
    } else {
      motorElevador.set(0.0);
    }

    // Brazo: Botones 5 y 6
    if (joystick.getRawButton(5)) motorBrazo.set(0.4);
    else if (joystick.getRawButton(6)) motorBrazo.set(-0.4);
    else motorBrazo.set(0.0);

    // Shooter: Botón 1 (Z en Keyboard 0 por defecto)
    motorShooter.set(joystick.getRawButton(1) ? 1.0 : 0.0);
  }

  @Override
  public void simulationPeriodic() {
    double voltaje = RobotController.getBatteryVoltage();

    // 1. Actualizar Física
    simElevador.setInput(motorElevador.get() * voltaje);
    simBrazo.setInput(motorBrazo.get() * voltaje);
    simShooter.setInput(motorShooter.get() * voltaje);

    simElevador.update(0.020);
    simBrazo.update(0.020);
    simShooter.update(0.020);

    // 2. Actualizar Widget Elevador
    etapaMovilElevador.setLength(0.15 + simElevador.getPositionMeters());

    // 3. Actualizar Widget Brazo
    boomBrazo.setAngle(Units.radiansToDegrees(simBrazo.getAngleRads()));

    // 4. Actualizar Widget Shooter (Giro de las 4 aspas y cambio de color según RPM)
    double rpm = simShooter.getAngularVelocityRPM();
    double deltaGrados = Units.radiansToDegrees(simShooter.getAngularVelocityRadPerSec()) * 0.020;
    anguloAcumuladoShooter += deltaGrados;

    // Cambia a verde brillante cuando supera las 3000 RPM
    Color8Bit colorRueda = (rpm > 3000) ? new Color8Bit(Color.kLimeGreen) : new Color8Bit(Color.kOrangeRed);

    for (int i = 0; i < 4; i++) {
      aspasInferiores[i].setAngle(anguloAcumuladoShooter + (i * 90));
      aspasInferiores[i].setColor(colorRueda);
      // La rueda superior gira en sentido contrario para alimentar la pieza
      aspasSuperiores[i].setAngle(-anguloAcumuladoShooter * 1.5 + (i * 90));
    }
    SmartDashboard.putNumber("Shooter RPM", rpm);

    // 5. Actualizar Chasis en Cancha
    if (!isSwerveMode) {
      simFisicaTank.setInputs(motorIzq.get() * voltaje, motorDer.get() * voltaje);
      simFisicaTank.update(0.020);
      simEncIzq.setDistance(simFisicaTank.getLeftPositionMeters());
      simEncDer.setDistance(simFisicaTank.getRightPositionMeters());
      simGyro.setAngle(-simFisicaTank.getHeading().getDegrees());
      canchaVirtual.setRobotPose(simFisicaTank.getPose());
    } else {
      double dx = -joystick.getRawAxis(1) * 3.5 * 0.020;
      double dy = -joystick.getRawAxis(0) * 3.5 * 0.020;
      double dTheta = -joystick.getRawAxis(4) * 2.5 * 0.020;
      poseSwerveVirtual = poseSwerveVirtual.exp(new Twist2d(dx, dy, dTheta));
      canchaVirtual.setRobotPose(poseSwerveVirtual);
    }
  }
}