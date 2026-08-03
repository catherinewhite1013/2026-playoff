// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.match;

import java.util.Locale;
import java.util.Objects;

/** Pure 2026 REBUILT HUB shift timing and status calculation. */
public final class HubShiftCalculator {
  public static final double TRANSITION_END_SECONDS = 130.0;
  public static final double SHIFT_1_END_SECONDS = 105.0;
  public static final double SHIFT_2_END_SECONDS = 80.0;
  public static final double SHIFT_3_END_SECONDS = 55.0;
  public static final double SHIFT_4_END_SECONDS = 30.0;

  public enum MatchMode {
    DISABLED,
    AUTONOMOUS,
    TELEOP,
    TEST,
    UNKNOWN
  }

  public enum AllianceColor {
    RED,
    BLUE,
    UNKNOWN
  }

  public enum TestOverride {
    DISABLED("Disabled"),
    RED("Red won AUTO"),
    BLUE("Blue won AUTO");

    private final String displayName;

    TestOverride(String displayName) {
      this.displayName = displayName;
    }

    public String displayName() {
      return displayName;
    }
  }

  public enum Phase {
    DISABLED("DISABLED"),
    AUTO("AUTO"),
    TRANSITION("TRANSITION"),
    SHIFT_1("SHIFT 1"),
    SHIFT_2("SHIFT 2"),
    SHIFT_3("SHIFT 3"),
    SHIFT_4("SHIFT 4"),
    END_GAME("END GAME"),
    TEST("TEST"),
    INVALID_TIME("INVALID TIME"),
    UNKNOWN("UNKNOWN MODE");

    private final String label;

    Phase(String label) {
      this.label = label;
    }

    public String label() {
      return label;
    }
  }

  public enum DataSource {
    FMS_GAME_DATA("FMS Game Data"),
    DS_GAME_DATA("DS Game Data"),
    PRACTICE_OVERRIDE("Practice Override"),
    FMS_DATA_MISSING("FMS Game Data Missing"),
    FMS_DATA_INVALID("FMS Game Data Invalid"),
    DS_DATA_MISSING("DS Game Data Missing"),
    DS_DATA_INVALID("DS Game Data Invalid");

    private final String label;

    DataSource(String label) {
      this.label = label;
    }

    public String label() {
      return label;
    }
  }

  public record Input(
      double matchTimeSeconds,
      MatchMode mode,
      AllianceColor alliance,
      String gameData,
      boolean fmsAttached,
      TestOverride testOverride) {
    public Input {
      Objects.requireNonNull(mode);
      Objects.requireNonNull(alliance);
      Objects.requireNonNull(testOverride);
    }
  }

  public record Result(
      double displayMatchTimeSeconds,
      Phase phase,
      double secondsToNextChange,
      boolean hubActive,
      boolean hubStatusKnown,
      DataSource dataSource,
      String effectiveGameData,
      String rawGameData,
      boolean testOverrideEnabled,
      boolean testOverrideApplied) {}

  private record GameDataResolution(AllianceColor autoWinner, DataSource source) {}

  private HubShiftCalculator() {}

  public static Result calculate(Input input) {
    Objects.requireNonNull(input);

    String rawGameData = input.gameData() == null ? "" : input.gameData().trim();
    boolean overrideEnabled =
        !input.fmsAttached() && input.testOverride() != TestOverride.DISABLED;
    GameDataResolution gameData = resolveGameData(input, rawGameData);

    Phase phase;
    double displayTime;
    double countdown;

    if (input.mode() == MatchMode.DISABLED) {
      phase = Phase.DISABLED;
      displayTime = -1.0;
      countdown = 0.0;
    } else if (input.mode() == MatchMode.TEST) {
      phase = Phase.TEST;
      displayTime = -1.0;
      countdown = 0.0;
    } else if (input.mode() == MatchMode.UNKNOWN) {
      phase = Phase.UNKNOWN;
      displayTime = -1.0;
      countdown = 0.0;
    } else if (!Double.isFinite(input.matchTimeSeconds()) || input.matchTimeSeconds() < 0.0) {
      phase = Phase.INVALID_TIME;
      displayTime = -1.0;
      countdown = 0.0;
    } else if (input.mode() == MatchMode.AUTONOMOUS) {
      phase = Phase.AUTO;
      displayTime = input.matchTimeSeconds();
      countdown = input.matchTimeSeconds();
    } else {
      displayTime = input.matchTimeSeconds();
      if (displayTime > TRANSITION_END_SECONDS) {
        phase = Phase.TRANSITION;
        countdown = displayTime - TRANSITION_END_SECONDS;
      } else if (displayTime > SHIFT_1_END_SECONDS) {
        phase = Phase.SHIFT_1;
        countdown = displayTime - SHIFT_1_END_SECONDS;
      } else if (displayTime > SHIFT_2_END_SECONDS) {
        phase = Phase.SHIFT_2;
        countdown = displayTime - SHIFT_2_END_SECONDS;
      } else if (displayTime > SHIFT_3_END_SECONDS) {
        phase = Phase.SHIFT_3;
        countdown = displayTime - SHIFT_3_END_SECONDS;
      } else if (displayTime > SHIFT_4_END_SECONDS) {
        phase = Phase.SHIFT_4;
        countdown = displayTime - SHIFT_4_END_SECONDS;
      } else {
        phase = Phase.END_GAME;
        countdown = displayTime;
      }
    }

    boolean hubStatusKnown = isAlwaysActivePhase(phase);
    boolean hubActive = hubStatusKnown;

    if (isAllianceShift(phase)
        && input.alliance() != AllianceColor.UNKNOWN
        && gameData.autoWinner() != AllianceColor.UNKNOWN) {
      hubStatusKnown = true;
      boolean winnerActive = phase == Phase.SHIFT_2 || phase == Phase.SHIFT_4;
      boolean robotIsAutoWinner = input.alliance() == gameData.autoWinner();
      hubActive = winnerActive == robotIsAutoWinner;
    }

    return new Result(
        displayTime,
        phase,
        Math.max(0.0, countdown),
        hubActive,
        hubStatusKnown,
        gameData.source(),
        allianceToGameData(gameData.autoWinner()),
        rawGameData,
        overrideEnabled,
        gameData.source() == DataSource.PRACTICE_OVERRIDE);
  }

  private static GameDataResolution resolveGameData(Input input, String rawGameData) {
    AllianceColor dsWinner = parseGameData(rawGameData);
    if (dsWinner != AllianceColor.UNKNOWN) {
      return new GameDataResolution(
          dsWinner,
          input.fmsAttached() ? DataSource.FMS_GAME_DATA : DataSource.DS_GAME_DATA);
    }

    if (input.fmsAttached()) {
      return new GameDataResolution(
          AllianceColor.UNKNOWN,
          rawGameData.isEmpty() ? DataSource.FMS_DATA_MISSING : DataSource.FMS_DATA_INVALID);
    }

    if (input.testOverride() == TestOverride.RED) {
      return new GameDataResolution(AllianceColor.RED, DataSource.PRACTICE_OVERRIDE);
    }
    if (input.testOverride() == TestOverride.BLUE) {
      return new GameDataResolution(AllianceColor.BLUE, DataSource.PRACTICE_OVERRIDE);
    }

    return new GameDataResolution(
        AllianceColor.UNKNOWN,
        rawGameData.isEmpty() ? DataSource.DS_DATA_MISSING : DataSource.DS_DATA_INVALID);
  }

  private static AllianceColor parseGameData(String gameData) {
    String normalized = gameData.toUpperCase(Locale.ROOT);
    if (normalized.equals("R")) {
      return AllianceColor.RED;
    }
    if (normalized.equals("B")) {
      return AllianceColor.BLUE;
    }
    return AllianceColor.UNKNOWN;
  }

  private static String allianceToGameData(AllianceColor alliance) {
    return switch (alliance) {
      case RED -> "R";
      case BLUE -> "B";
      case UNKNOWN -> "Unknown";
    };
  }

  private static boolean isAlwaysActivePhase(Phase phase) {
    return phase == Phase.AUTO || phase == Phase.TRANSITION || phase == Phase.END_GAME;
  }

  private static boolean isAllianceShift(Phase phase) {
    return phase == Phase.SHIFT_1
        || phase == Phase.SHIFT_2
        || phase == Phase.SHIFT_3
        || phase == Phase.SHIFT_4;
  }
}
