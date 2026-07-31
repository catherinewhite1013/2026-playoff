// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.StatusConstants;

public class StatusSubsystem extends SubsystemBase {
  private final AddressableLED led = new AddressableLED(StatusConstants.kPwmPort);
  private final AddressableLEDBuffer buffer =
      new AddressableLEDBuffer(StatusConstants.kTotalLedCount);

  private boolean autoShootActive = false;
  private boolean shooterReady = false;
  private double chargeProgress = 0.0;
  private int animationTick = 0;
  private int rainbowFirstPixelHue = 0;

  public StatusSubsystem() {
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
  }

  public void beginAutoShoot() {
    autoShootActive = true;
    shooterReady = false;
    chargeProgress = 0.0;
    animationTick = 0;
  }

  public void updateAutoShoot(boolean ready, double progress) {
    if (ready && !shooterReady) {
      animationTick = 0;
    }
    shooterReady = ready;
    chargeProgress = ready ? 1.0 : Math.max(0.0, Math.min(1.0, progress));
  }

  public void endAutoShoot() {
    autoShootActive = false;
    shooterReady = false;
    chargeProgress = 0.0;
    animationTick = 0;
  }

  @Override
  public void periodic() {
    if (autoShootActive) {
      renderAutoShoot();
    } else {
      renderNormalStream();
    }

    led.setData(buffer);
    animationTick++;
  }

  private void renderNormalStream() {
    for (int i = 0; i < buffer.getLength(); i++) {
      int hue = (rainbowFirstPixelHue + i * 180 / buffer.getLength()) % 180;
      buffer.setHSV(i, hue, 255, StatusConstants.kNormalBrightness);
    }
    rainbowFirstPixelHue =
        (rainbowFirstPixelHue + StatusConstants.kRainbowStep) % 180;
  }

  private void renderAutoShoot() {
    if (shooterReady) {
      renderReadyChargeStrobe();
    } else {
      renderFrontChargeSegments();
    }

    int chargeStart = StatusConstants.kAimLedCount;
    int chargedCount =
        (int) Math.round(chargeProgress * StatusConstants.kChargeLedCount);
    for (int i = 0; i < StatusConstants.kChargeLedCount; i++) {
      int ledIndex = chargeStart + i;
      if (i < chargedCount) {
        buffer.setRGB(ledIndex, StatusConstants.kShootBrightness, 0, 0);
      } else {
        buffer.setRGB(ledIndex, 0, 0, 0);
      }
    }

    if (!shooterReady && chargedCount > 0) {
      int chargeHead = chargeStart + chargedCount - 1;
      buffer.setRGB(chargeHead, 255, 48, 0);
    }
  }

  private void renderReadyChargeStrobe() {
    int mainStart = 0;
    int forwardStart = mainStart + StatusConstants.kMainChargeLedCount;
    int reverseStart = forwardStart + StatusConstants.kForwardChargeLedCount;
    int chargeStep =
        ((animationTick / StatusConstants.kReadyChargeStepTicks) + 1)
            % (StatusConstants.kForwardChargeLedCount + 1);
    boolean mainStrobeOn =
        (animationTick / StatusConstants.kReadyStrobeHalfPeriodTicks) % 2 == 0;

    renderChargeSegment(
        mainStart,
        StatusConstants.kMainChargeLedCount,
        false,
        mainStrobeOn ? StatusConstants.kMainChargeLedCount : 0,
        StatusConstants.kShootBrightness,
        0,
        0);
    renderChargeSegment(
        forwardStart,
        StatusConstants.kForwardChargeLedCount,
        false,
        chargeStep,
        StatusConstants.kShootBrightness,
        0,
        0);
    renderChargeSegment(
        reverseStart,
        StatusConstants.kReverseChargeLedCount,
        true,
        chargeStep,
        StatusConstants.kShootBrightness,
        0,
        0);
  }

  private void renderFrontChargeSegments() {
    int mainStart = 0;
    int forwardStart = mainStart + StatusConstants.kMainChargeLedCount;
    int reverseStart = forwardStart + StatusConstants.kForwardChargeLedCount;

    renderChargeSegment(
        mainStart,
        StatusConstants.kMainChargeLedCount,
        false,
        (int) Math.round(chargeProgress * StatusConstants.kMainChargeLedCount),
        0,
        StatusConstants.kShootBrightness,
        0);
    renderChargeSegment(
        forwardStart,
        StatusConstants.kForwardChargeLedCount,
        false,
        (int) Math.round(chargeProgress * StatusConstants.kForwardChargeLedCount),
        0,
        StatusConstants.kShootBrightness,
        0);
    renderChargeSegment(
        reverseStart,
        StatusConstants.kReverseChargeLedCount,
        true,
        (int) Math.round(chargeProgress * StatusConstants.kReverseChargeLedCount),
        0,
        StatusConstants.kShootBrightness,
        0);
  }

  private void renderChargeSegment(
      int start,
      int length,
      boolean reversed,
      int chargedCount,
      int red,
      int green,
      int blue) {
    for (int i = 0; i < length; i++) {
      boolean isCharged = reversed ? i >= length - chargedCount : i < chargedCount;
      buffer.setRGB(
          start + i,
          isCharged ? red : 0,
          isCharged ? green : 0,
          isCharged ? blue : 0);
    }
  }
}
