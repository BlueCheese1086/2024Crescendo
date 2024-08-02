// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.Drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;

import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2Phoenix6 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(RobotMap.Drive.gyro);
  private final StatusSignal<Double> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2Phoenix6() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(DriveConstants.odometeryFrequency);
    yawVelocity.setUpdateFrequency(100.0);
    pigeon.optimizeBusUtilization();
    
    yawTimestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  boolean valid = yaw.refresh().getStatus().isOK();
                  if (valid) {
                    return OptionalDouble.of(yaw.getValueAsDouble());
                  } else {
                    return OptionalDouble.empty();
                  }
                });
  }
  

  @Override
  public void processInputs(GyroIOInputsAutoLogged inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }


  @Override
  public void setYaw(double yaw) {
    pigeon.setYaw(yaw, 0.1);
  }


  @Override
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(yaw.getValueAsDouble());
  }


  @Override
  public void reset() {
    setYaw(0.0);
  }

  public Rotation2d normalizeAngle(Rotation2d yaw) {
    while (yaw.getRadians() < Math.PI * 2) {
      yaw.plus(Rotation2d.fromRadians(Math.PI));
     }
    return Rotation2d.fromRadians(yaw.getRadians() % (Math.PI * 2));
  }
}
