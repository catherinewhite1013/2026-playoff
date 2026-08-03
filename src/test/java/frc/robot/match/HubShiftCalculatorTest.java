package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.match.HubShiftCalculator.AllianceColor;
import frc.robot.match.HubShiftCalculator.DataSource;
import frc.robot.match.HubShiftCalculator.Input;
import frc.robot.match.HubShiftCalculator.MatchMode;
import frc.robot.match.HubShiftCalculator.Phase;
import frc.robot.match.HubShiftCalculator.Result;
import frc.robot.match.HubShiftCalculator.TestOverride;
import java.util.stream.Stream;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

class HubShiftCalculatorTest {
  private static final double EPSILON = 1e-9;

  @ParameterizedTest
  @MethodSource("phaseBoundaries")
  void calculatesEveryTeleopBoundary(double matchTime, Phase phase, double countdown) {
    Result result = calculate(
        matchTime, MatchMode.TELEOP, AllianceColor.RED, "R", false, TestOverride.DISABLED);

    assertEquals(phase, result.phase());
    assertEquals(countdown, result.secondsToNextChange(), EPSILON);
    assertTrue(result.secondsToNextChange() >= 0.0);
  }

  static Stream<Arguments> phaseBoundaries() {
    return Stream.of(
        Arguments.of(140.0, Phase.TRANSITION, 10.0),
        Arguments.of(130.001, Phase.TRANSITION, 0.001),
        Arguments.of(130.0, Phase.SHIFT_1, 25.0),
        Arguments.of(105.001, Phase.SHIFT_1, 0.001),
        Arguments.of(105.0, Phase.SHIFT_2, 25.0),
        Arguments.of(80.001, Phase.SHIFT_2, 0.001),
        Arguments.of(80.0, Phase.SHIFT_3, 25.0),
        Arguments.of(55.001, Phase.SHIFT_3, 0.001),
        Arguments.of(55.0, Phase.SHIFT_4, 25.0),
        Arguments.of(30.001, Phase.SHIFT_4, 0.001),
        Arguments.of(30.0, Phase.END_GAME, 30.0),
        Arguments.of(0.001, Phase.END_GAME, 0.001),
        Arguments.of(0.0, Phase.END_GAME, 0.0));
  }

  @ParameterizedTest
  @MethodSource("hubStatusCases")
  void alternatesHubStatusFromAutoWinner(
      AllianceColor robotAlliance,
      String gameData,
      double matchTime,
      boolean expectedActive) {
    Result result = calculate(
        matchTime,
        MatchMode.TELEOP,
        robotAlliance,
        gameData,
        false,
        TestOverride.DISABLED);

    assertTrue(result.hubStatusKnown());
    assertEquals(expectedActive, result.hubActive());
  }

  static Stream<Arguments> hubStatusCases() {
    return Stream.of(
        Arguments.of(AllianceColor.RED, "R", 120.0, false),
        Arguments.of(AllianceColor.RED, "R", 100.0, true),
        Arguments.of(AllianceColor.RED, "R", 70.0, false),
        Arguments.of(AllianceColor.RED, "R", 40.0, true),
        Arguments.of(AllianceColor.BLUE, "R", 120.0, true),
        Arguments.of(AllianceColor.BLUE, "R", 100.0, false),
        Arguments.of(AllianceColor.RED, "B", 120.0, true),
        Arguments.of(AllianceColor.BLUE, "B", 120.0, false),
        Arguments.of(AllianceColor.BLUE, "B", 100.0, true));
  }

  @Test
  void autoTransitionAndEndGameAreAlwaysActiveWithoutGameData() {
    Result auto = calculate(
        20.0, MatchMode.AUTONOMOUS, AllianceColor.UNKNOWN, "", false, TestOverride.DISABLED);
    Result transition = calculate(
        140.0, MatchMode.TELEOP, AllianceColor.UNKNOWN, "", false, TestOverride.DISABLED);
    Result endGame = calculate(
        0.0, MatchMode.TELEOP, AllianceColor.UNKNOWN, "", false, TestOverride.DISABLED);

    assertEquals(Phase.AUTO, auto.phase());
    assertEquals(20.0, auto.secondsToNextChange(), EPSILON);
    assertTrue(auto.hubActive());
    assertTrue(auto.hubStatusKnown());
    assertTrue(transition.hubActive());
    assertTrue(transition.hubStatusKnown());
    assertTrue(endGame.hubActive());
    assertTrue(endGame.hubStatusKnown());
  }

  @ParameterizedTest
  @MethodSource("practiceOverrideCases")
  void practiceOverrideSuppliesMissingOrInvalidGameData(
      String rawGameData,
      TestOverride testOverride,
      String expectedGameData,
      boolean expectedActive) {
    Result result = calculate(
        120.0, MatchMode.TELEOP, AllianceColor.RED, rawGameData, false, testOverride);

    assertEquals(DataSource.PRACTICE_OVERRIDE, result.dataSource());
    assertEquals(expectedGameData, result.effectiveGameData());
    assertTrue(result.testOverrideEnabled());
    assertTrue(result.testOverrideApplied());
    assertTrue(result.hubStatusKnown());
    assertEquals(expectedActive, result.hubActive());
  }

  static Stream<Arguments> practiceOverrideCases() {
    return Stream.of(
        Arguments.of("", TestOverride.RED, "R", false),
        Arguments.of("", TestOverride.BLUE, "B", true),
        Arguments.of("invalid", TestOverride.BLUE, "B", true));
  }

  @Test
  void legalDsGameDataHasPriorityOverPracticeOverride() {
    Result result = calculate(
        120.0, MatchMode.TELEOP, AllianceColor.RED, "B", false, TestOverride.RED);

    assertEquals(DataSource.DS_GAME_DATA, result.dataSource());
    assertEquals("B", result.effectiveGameData());
    assertTrue(result.testOverrideEnabled());
    assertFalse(result.testOverrideApplied());
    assertTrue(result.hubActive());
  }

  @Test
  void fmsGameDataHasPriorityAndBlocksPracticeOverride() {
    Result valid = calculate(
        120.0, MatchMode.TELEOP, AllianceColor.RED, "B", true, TestOverride.RED);
    Result missing = calculate(
        120.0, MatchMode.TELEOP, AllianceColor.RED, "", true, TestOverride.RED);
    Result invalid = calculate(
        120.0, MatchMode.TELEOP, AllianceColor.RED, "X", true, TestOverride.BLUE);

    assertEquals(DataSource.FMS_GAME_DATA, valid.dataSource());
    assertEquals("B", valid.effectiveGameData());
    assertFalse(valid.testOverrideEnabled());
    assertFalse(valid.testOverrideApplied());
    assertEquals(DataSource.FMS_DATA_MISSING, missing.dataSource());
    assertFalse(missing.hubStatusKnown());
    assertFalse(missing.hubActive());
    assertFalse(missing.testOverrideEnabled());
    assertFalse(missing.testOverrideApplied());
    assertEquals(DataSource.FMS_DATA_INVALID, invalid.dataSource());
    assertFalse(invalid.hubStatusKnown());
    assertFalse(invalid.testOverrideEnabled());
    assertFalse(invalid.testOverrideApplied());
  }

  @Test
  void missingOrInvalidDataWithoutOverrideIsExplicitlyUnknown() {
    Result missing = calculate(
        120.0, MatchMode.TELEOP, AllianceColor.RED, "", false, TestOverride.DISABLED);
    Result invalid = calculate(
        120.0, MatchMode.TELEOP, AllianceColor.RED, "RB", false, TestOverride.DISABLED);

    assertEquals(DataSource.DS_DATA_MISSING, missing.dataSource());
    assertEquals("Unknown", missing.effectiveGameData());
    assertFalse(missing.hubStatusKnown());
    assertFalse(missing.hubActive());
    assertEquals(DataSource.DS_DATA_INVALID, invalid.dataSource());
    assertFalse(invalid.hubStatusKnown());
  }

  @Test
  void unknownAllianceCannotClaimAnAllianceShiftStatus() {
    Result result = calculate(
        120.0, MatchMode.TELEOP, AllianceColor.UNKNOWN, "R", false, TestOverride.DISABLED);

    assertEquals(DataSource.DS_GAME_DATA, result.dataSource());
    assertFalse(result.hubStatusKnown());
    assertFalse(result.hubActive());
  }

  @ParameterizedTest
  @MethodSource("inactiveModes")
  void disabledTestAndUnknownModesClearDisplayedTiming(MatchMode mode, Phase phase) {
    Result result = calculate(
        99.0, mode, AllianceColor.RED, "R", false, TestOverride.DISABLED);

    assertEquals(phase, result.phase());
    assertEquals(-1.0, result.displayMatchTimeSeconds(), EPSILON);
    assertEquals(0.0, result.secondsToNextChange(), EPSILON);
    assertFalse(result.hubStatusKnown());
    assertFalse(result.hubActive());
  }

  static Stream<Arguments> inactiveModes() {
    return Stream.of(
        Arguments.of(MatchMode.DISABLED, Phase.DISABLED),
        Arguments.of(MatchMode.TEST, Phase.TEST),
        Arguments.of(MatchMode.UNKNOWN, Phase.UNKNOWN));
  }

  @ParameterizedTest
  @MethodSource("invalidTimes")
  void invalidEnabledMatchTimesDoNotLeaveStaleValues(double matchTime) {
    Result result = calculate(
        matchTime, MatchMode.TELEOP, AllianceColor.RED, "R", false, TestOverride.DISABLED);

    assertEquals(Phase.INVALID_TIME, result.phase());
    assertEquals(-1.0, result.displayMatchTimeSeconds(), EPSILON);
    assertEquals(0.0, result.secondsToNextChange(), EPSILON);
    assertFalse(result.hubStatusKnown());
    assertFalse(result.hubActive());
  }

  static Stream<Double> invalidTimes() {
    return Stream.of(-1.0, Double.NaN, Double.POSITIVE_INFINITY);
  }

  private static Result calculate(
      double matchTime,
      MatchMode mode,
      AllianceColor alliance,
      String gameData,
      boolean fmsAttached,
      TestOverride testOverride) {
    return HubShiftCalculator.calculate(
        new Input(matchTime, mode, alliance, gameData, fmsAttached, testOverride));
  }
}
