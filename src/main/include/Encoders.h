// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>
#include <rev/CANSparkMax.h>
#include <frc/AnalogInput.h>

class DriveEncoder {
 public:
     DriveEncoder(const int &a, const int &b) {};

  double GetRate() const {return 0.0;};
  void SetDistancePerPulse(const double &) {};
};

#if 0
class DriveEncoder : public frc::Encoder {
 public:
     DriveEncoder(const int &a, const int &b) : Encoder(a, b) {};

//DriveEncoder(frc::MotorController & motorController)
//   : m_motorController(motorController)
//{};

  double GetRate() const {return 0.0;};

 private:
  static constexpr int kEncoderResolution = 4096;

//frc::MotorController motorController;
};
#endif

class MA3AnalogEncoder {
 public:
     MA3AnalogEncoder(const int &a, const int &b) : m_channel(a), m_potentiometer(a), m_relative_zero(0.0) 
     {
      //  printf("a=%d %p\n", a, (void*)&m_potentiometer);
       m_relative_zero = m_potentiometer.GetAverageVoltage();
     };

  double Get() const 
  {
      double v = m_potentiometer.GetAverageVoltage();
      double a = v - m_relative_zero;

      if (a < 0.0) a += 5.0;

      printf("%d: a=%5.2f, r=%5.2f\n", m_channel, a, 2 * 3.1415 * (0.0 - a));

      return 2 * 3.1415 * (5.0 - a);
  };

  void SetDistancePerPulse(const double &) {};

private:
    int m_channel;
      frc::AnalogInput m_potentiometer;
      double m_relative_zero;
};

