// Copyright 2024 LaSIS Lab
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>
#include <cmath>

// Re-declare the map function for testing (same as in lj_handler.cpp)
double map(double value, double in_min, double in_max, double out_min, double out_max)
{
  return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Helper function to convert steering ratio to internal ratio (same logic as set_steering_ratio)
double steering_ratio_to_internal(double steering_ratio)
{
  double ratio = (steering_ratio + 1.0) / 2.0;
  return std::max(0.0, std::min(1.0, ratio));
}

// Helper function to convert throttle value to internal ratio (same logic as set_throttle_brake)
double throttle_to_internal(double throttle_value)
{
  double ratio = (throttle_value + 1.0) / 2.0;
  return std::max(0.0, std::min(1.0, ratio));
}

// =============================================================================
// Map Function Tests
// =============================================================================

TEST(MapFunctionTest, MapsMinToMin)
{
  double result = map(0.0, 0.0, 1.0, 0.0, 5.0);
  EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(MapFunctionTest, MapsMaxToMax)
{
  double result = map(1.0, 0.0, 1.0, 0.0, 5.0);
  EXPECT_DOUBLE_EQ(result, 5.0);
}

TEST(MapFunctionTest, MapsMidpoint)
{
  double result = map(0.5, 0.0, 1.0, 0.0, 10.0);
  EXPECT_DOUBLE_EQ(result, 5.0);
}

TEST(MapFunctionTest, MapsNegativeRange)
{
  double result = map(0.0, -1.0, 1.0, 0.0, 100.0);
  EXPECT_DOUBLE_EQ(result, 50.0);
}

TEST(MapFunctionTest, MapsToNegativeOutput)
{
  double result = map(0.5, 0.0, 1.0, -10.0, 10.0);
  EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(MapFunctionTest, MapsVoltagePercentage)
{
  // Simulating voltage mapping: ratio 0.5 with min_perc=0.15, max_perc=0.85
  // nom_voltage = 5.0V
  double nom_voltage = 5.0;
  double min_perc = 0.15;
  double max_perc = 0.85;
  double min_v = nom_voltage * min_perc;  // 0.75V
  double max_v = nom_voltage * max_perc;  // 4.25V
  
  double result = map(0.5, 0.0, 1.0, min_v, max_v);
  EXPECT_DOUBLE_EQ(result, 2.5);  // Midpoint between 0.75 and 4.25
}

// =============================================================================
// Steering Ratio Conversion Tests
// =============================================================================

TEST(SteeringRatioTest, FullLeftConvertsToZero)
{
  double result = steering_ratio_to_internal(-1.0);
  EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(SteeringRatioTest, CenterConvertsToHalf)
{
  double result = steering_ratio_to_internal(0.0);
  EXPECT_DOUBLE_EQ(result, 0.5);
}

TEST(SteeringRatioTest, FullRightConvertsToOne)
{
  double result = steering_ratio_to_internal(1.0);
  EXPECT_DOUBLE_EQ(result, 1.0);
}

TEST(SteeringRatioTest, HalfLeftConvertsToQuarter)
{
  double result = steering_ratio_to_internal(-0.5);
  EXPECT_DOUBLE_EQ(result, 0.25);
}

TEST(SteeringRatioTest, HalfRightConvertsToThreeQuarters)
{
  double result = steering_ratio_to_internal(0.5);
  EXPECT_DOUBLE_EQ(result, 0.75);
}

TEST(SteeringRatioTest, ClampsValuesBelowMinusOne)
{
  double result = steering_ratio_to_internal(-2.0);
  EXPECT_DOUBLE_EQ(result, 0.0);  // Clamped to 0.0
}

TEST(SteeringRatioTest, ClampsValuesAboveOne)
{
  double result = steering_ratio_to_internal(2.0);
  EXPECT_DOUBLE_EQ(result, 1.0);  // Clamped to 1.0
}

// =============================================================================
// Throttle Conversion Tests
// =============================================================================

TEST(ThrottleConversionTest, FullBrakeConvertsToZero)
{
  double result = throttle_to_internal(-1.0);
  EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(ThrottleConversionTest, NeutralConvertsToHalf)
{
  double result = throttle_to_internal(0.0);
  EXPECT_DOUBLE_EQ(result, 0.5);
}

TEST(ThrottleConversionTest, FullThrottleConvertsToOne)
{
  double result = throttle_to_internal(1.0);
  EXPECT_DOUBLE_EQ(result, 1.0);
}

TEST(ThrottleConversionTest, HalfBrakeConvertsToQuarter)
{
  double result = throttle_to_internal(-0.5);
  EXPECT_DOUBLE_EQ(result, 0.25);
}

TEST(ThrottleConversionTest, HalfThrottleConvertsToThreeQuarters)
{
  double result = throttle_to_internal(0.5);
  EXPECT_DOUBLE_EQ(result, 0.75);
}

TEST(ThrottleConversionTest, ClampsValuesBelowMinusOne)
{
  double result = throttle_to_internal(-1.5);
  EXPECT_DOUBLE_EQ(result, 0.0);
}

TEST(ThrottleConversionTest, ClampsValuesAboveOne)
{
  double result = throttle_to_internal(1.5);
  EXPECT_DOUBLE_EQ(result, 1.0);
}

// =============================================================================
// Voltage Calculation Tests (simulating set_control_axis logic)
// =============================================================================

class VoltageCalculationTest : public ::testing::Test {
protected:
  double nom_vs_master_ = 5.0;
  double nom_vs_slave_ = 5.0;
  double min_perc_ = 0.15;
  double max_perc_ = 0.85;
  
  void calculate_voltages(double ratio, bool opposition,
                          double& m1, double& s1, double& m2, double& s2)
  {
    double master_min = nom_vs_master_ * min_perc_;
    double master_max = nom_vs_master_ * max_perc_;
    double slave_min = nom_vs_slave_ * min_perc_;
    double slave_max = nom_vs_slave_ * max_perc_;
    
    // First pair (M1, S1)
    m1 = map(ratio, 0.0, 1.0, master_min, master_max);
    m1 = std::max(master_min, std::min(master_max, m1));
    
    s1 = map(1.0 - ratio, 0.0, 1.0, slave_min, slave_max);
    s1 = std::max(slave_min, std::min(slave_max, s1));
    
    // Second pair (M2, S2)
    double m2_ratio = opposition ? (1.0 - ratio) : ratio;
    m2 = map(m2_ratio, 0.0, 1.0, master_min, master_max);
    m2 = std::max(master_min, std::min(master_max, m2));
    
    s2 = map(1.0 - m2_ratio, 0.0, 1.0, slave_min, slave_max);
    s2 = std::max(slave_min, std::min(slave_max, s2));
  }
};

TEST_F(VoltageCalculationTest, CenterPositionOppositionMode)
{
  double m1, s1, m2, s2;
  calculate_voltages(0.5, true, m1, s1, m2, s2);
  
  // At center (ratio=0.5), all voltages should be at midpoint
  double expected_mid = nom_vs_master_ * (min_perc_ + max_perc_) / 2.0;  // 2.5V
  EXPECT_DOUBLE_EQ(m1, expected_mid);
  EXPECT_DOUBLE_EQ(s1, expected_mid);
  EXPECT_DOUBLE_EQ(m2, expected_mid);
  EXPECT_DOUBLE_EQ(s2, expected_mid);
}

TEST_F(VoltageCalculationTest, FullLeftOppositionMode)
{
  double m1, s1, m2, s2;
  calculate_voltages(0.0, true, m1, s1, m2, s2);
  
  double master_min = nom_vs_master_ * min_perc_;  // 0.75V
  double master_max = nom_vs_master_ * max_perc_;  // 4.25V
  double slave_min = nom_vs_slave_ * min_perc_;
  double slave_max = nom_vs_slave_ * max_perc_;
  
  EXPECT_DOUBLE_EQ(m1, master_min);
  EXPECT_DOUBLE_EQ(s1, slave_max);
  EXPECT_DOUBLE_EQ(m2, master_max);  // Opposition: 1.0 - 0.0 = 1.0
  EXPECT_DOUBLE_EQ(s2, slave_min);
}

TEST_F(VoltageCalculationTest, FullRightOppositionMode)
{
  double m1, s1, m2, s2;
  calculate_voltages(1.0, true, m1, s1, m2, s2);
  
  double master_min = nom_vs_master_ * min_perc_;
  double master_max = nom_vs_master_ * max_perc_;
  double slave_min = nom_vs_slave_ * min_perc_;
  double slave_max = nom_vs_slave_ * max_perc_;
  
  EXPECT_DOUBLE_EQ(m1, master_max);
  EXPECT_DOUBLE_EQ(s1, slave_min);
  EXPECT_DOUBLE_EQ(m2, master_min);  // Opposition: 1.0 - 1.0 = 0.0
  EXPECT_DOUBLE_EQ(s2, slave_max);
}

TEST_F(VoltageCalculationTest, VoltagesStayWithinBounds)
{
  double m1, s1, m2, s2;
  double master_min = nom_vs_master_ * min_perc_;
  double master_max = nom_vs_master_ * max_perc_;
  double slave_min = nom_vs_slave_ * min_perc_;
  double slave_max = nom_vs_slave_ * max_perc_;
  
  // Test various ratios
  for (double ratio = 0.0; ratio <= 1.0; ratio += 0.1) {
    calculate_voltages(ratio, true, m1, s1, m2, s2);
    
    EXPECT_GE(m1, master_min) << "M1 below min at ratio " << ratio;
    EXPECT_LE(m1, master_max) << "M1 above max at ratio " << ratio;
    EXPECT_GE(s1, slave_min) << "S1 below min at ratio " << ratio;
    EXPECT_LE(s1, slave_max) << "S1 above max at ratio " << ratio;
    EXPECT_GE(m2, master_min) << "M2 below min at ratio " << ratio;
    EXPECT_LE(m2, master_max) << "M2 above max at ratio " << ratio;
    EXPECT_GE(s2, slave_min) << "S2 below min at ratio " << ratio;
    EXPECT_LE(s2, slave_max) << "S2 above max at ratio " << ratio;
  }
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
