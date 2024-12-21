#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <array>
#include <vector>
#include <chrono>
#include <iomanip>

// target radius
float RR_hip_tar = -0.447;
float RR_thigh_tar = 2.200;
float RR_calf_tar = -1.571;
std::array<double, 3> targetAngles{RR_hip_tar, RR_thigh_tar, RR_calf_tar};

// offset radius
float RR_hip_off = 0.603828;
float RR_thigh_off = 3.16746;
float RR_calf_off = 3.21931;
std::array<double, 3> offsetAngles{RR_hip_off, RR_thigh_off, RR_calf_off};

// cmd radius
float q0 = 0.0;
float q1 = 2.4;
float q2 = -1.57;
std::array<double, 3> cmdAngles{q0, q1, q2};

// inverted index
std::array<double, 3> invertedIndex{-1, -1, 1};

int main()
{

  SerialPort serial("/dev/ttyUSB0");
  MotorCmd cmd;
  MotorData data;
  //   std::vector<MotorCmd> motorCmdVec;
  //   std::vector<MotorData> motorDataVec;

  auto t_start = std::chrono::high_resolution_clock::now();
  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  auto tStartStateCalibHip = std::chrono::high_resolution_clock::now();
  auto tStartStateCalibThigh = std::chrono::high_resolution_clock::now();
  auto tStartStateCalibFinal = std::chrono::high_resolution_clock::now();

  // loop three times
  while (false)
  {
    for (uint64_t id{0}; id < 1; ++id)
    {
      cmd.motorType = MotorType::A1;
      data.motorType = MotorType::A1;
      cmd.mode = queryMotorMode(MotorType::A1, MotorMode::FOC);
      cmd.id = id;
      cmd.kp = 0.0;
      cmd.kd = 0.0;

      // y = (-x + b) * gear_ratio
      cmd.q = (invertedIndex[id] * cmdAngles[id] + offsetAngles[id]) * queryGearRatio(MotorType::GO_M8010_6);
      cmd.dq = 0.0;
      cmd.tau = 0.0;

      serial.sendRecv(&cmd, &data);

      std::cout << "[" << id << "] cmd angle  : " << cmdAngles[id] << "\n";
      std::cout << "[" << id << "] go1 angle  : " << data.q / queryGearRatio(MotorType::GO_M8010_6) << "\n";
      std::cout << "[" << id << "] offset     : " << offsetAngles[id] << "\n";
      std::cout << "[" << id << "] cmd.q      : " << cmd.q << "\n";
    }
    std::cout << "\n";
    usleep(200);
  }

  // loop at once
  while (true)
  {
    std::vector<MotorCmd> motorCmdVec;
    std::vector<MotorData> motorDataVec;

    for (uint64_t id{0}; id < 1; ++id)
    {
      MotorData data{};
      cmd.motorType = MotorType::A1;
      data.motorType = MotorType::A1;
      cmd.mode = queryMotorMode(MotorType::A1, MotorMode::BRAKE);
      cmd.id = id;
      cmd.kp = 0.016;
      cmd.kd = 0.32;

      // y = (-x + b) * gear_ratio
      cmd.q = (invertedIndex[id] * cmdAngles[id] + offsetAngles[id]) * queryGearRatio(MotorType::GO_M8010_6);
      cmd.dq = 0.0;
      cmd.tau = 0.0;

      motorCmdVec.push_back(cmd);
      motorDataVec.push_back(data);
    }

    serial.sendRecv(motorCmdVec, motorDataVec);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << motorDataVec[0].q << " | " << motorDataVec[1].q << " | " << motorDataVec[2].q << "\n";
    std::cout << motorDataVec[0].tau << " | " << motorDataVec[1].tau << " | " << motorDataVec[2].tau << "\n";

    usleep(200);
  }
}
