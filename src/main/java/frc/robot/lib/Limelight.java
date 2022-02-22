// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/** Add your docs here. */
public class Limelight {
  private final NetworkTable table;

  private final NetworkTableEntry m_tv;
  private final NetworkTableEntry m_tx;
  private final NetworkTableEntry m_ty;
  private final NetworkTableEntry m_ta;
  private final NetworkTableEntry m_ts;
  private final NetworkTableEntry m_tl;

  private final NetworkTableEntry m_tcornx;
  private final NetworkTableEntry m_tcorny;

  private final NetworkTableEntry m_ledMode;
  private final NetworkTableEntry m_camMode;
  private final NetworkTableEntry m_pipeline;
  private final NetworkTableEntry m_stream;
  private final NetworkTableEntry m_snapshot;

  /**
   * Creates a standard limelight.
   */
  public Limelight() {
    this(NetworkTableInstance.getDefault().getTable("limelight"));
  }

  /**
   * Creates a limelight instance for a specific limelight.
   * @param suffix The unique identifier for the limelight "limelight-<suffix>"
   */
  public Limelight(final String suffix) {
    this(NetworkTableInstance.getDefault().getTable("limelight-" + suffix));
  }

  public Limelight(final NetworkTable networkTable) {
    table = networkTable;

    m_tv = table.getEntry("tv");
    m_tx = table.getEntry("tx");
    m_ty = table.getEntry("ty");
    m_ta = table.getEntry("ta");
    m_ts = table.getEntry("ts");
    m_tl = table.getEntry("tl");

    m_tcornx = table.getEntry("tcornx");
    m_tcorny = table.getEntry("tcorny");

    m_ledMode = table.getEntry("ledMode");
    m_camMode = table.getEntry("camMode");
    m_pipeline = table.getEntry("pipeline");
    m_stream = table.getEntry("stream");
    m_snapshot = table.getEntry("snapshot");
  }

  public boolean canSeeTarget() {
    return m_tv.getDouble(0) == 1;
  }

  public double getTargetArea() {
    return m_ta.getDouble(0);
  }

  public double getTargetX() {
    return m_tx.getDouble(0);
  }

  public double getTargetY() {
    return m_ty.getDouble(0);
  }

  public Translation2d getTargetPosition() {
    return new Translation2d(getTargetX(), getTargetY());
  }

  public double getTargetSkew() {
    return m_ts.getDouble(0);
  }

  public double getPipelineLatency() {
    return m_tl.getDouble(0);
  }

  public void setCamMode(final CamMode mode) {
    m_camMode.setNumber(mode.ordinal());
  }

  public void setLedMode(final LedMode mode) {
    m_ledMode.setNumber(mode.ordinal());
  }

  public void setSnapshotsEnabled(final boolean isEnabled) {
    m_snapshot.setNumber(isEnabled ? 1 : 0);
  }

  public void setPipeline(final int pipeline) {
    m_pipeline.setNumber(pipeline);
  }

  public void setStreamMode(final StreamMode mode) {
    m_stream.setNumber(mode.ordinal());
  }

  public enum CamMode {
    VISION, // Run the vision pipeline.
    DRIVER  // Disable the pipeline.
  }

  public enum LedMode {
    DEFAULT,  // Sets LEDs to what is specified in the pipeline
    OFF,
    BLINK,
    ON
  }

  public enum StreamMode {
    STANDARD,     // Side-by-side streams if a webcam is attached to limelight.
    PIP_MAIN,     // Picture in picture, secondary stream in lower-right corner of primary stream.
    PIP_SECONDARY // Picture in picture, primary stream in lower right corner of secondary stream.
  }
}
