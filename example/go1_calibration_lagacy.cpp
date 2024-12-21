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
  std::vector<MotorCmd> motorCmdVec;
  std::vector<MotorData> motorDataVec;

  auto t_start = std::chrono::high_resolution_clock::now();
  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

  auto tStartStateCalibHip = std::chrono::high_resolution_clock::now();
  auto tStartStateCalibThigh = std::chrono::high_resolution_clock::now();
  auto tStartStateCalibFinal = std::chrono::high_resolution_clock::now();

  int state{0}; // 0: init
                // 1: calibrate hip
                // 2: calibrate thigh
                // 3: finished

  // Loop Cmd Data
  while (false)
  {
    auto currTime{std::chrono::high_resolution_clock::now()};

    // STATE MACHINE 0
    if (state == 0)
    {
      for (uint64_t id{0}; id < 3; ++id)
      {
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id = id;
        cmd.kp = 0.0;
        cmd.kd = 0.0;

        cmd.q = 0;
        cmd.dq = 0.0;
        cmd.tau = 0.0;

        serial.sendRecv(&cmd, &data);
      }
      // init state...
      system("clear");
      std::cout << "Initializing... Process starts in 3 seconds \n";
      sleep(1);
      system("clear");
      std::cout << "Initializing... Process starts in 2 seconds \n";
      sleep(1);
      system("clear");
      std::cout << "Initializing... Process starts in 1 seconds \n";
      sleep(1);
      system("clear");
      state = 1;
      tStartStateCalibHip = std::chrono::high_resolution_clock::now();
    }

    // STATE MACHINE 1
    else if (state == 1)
    {
      // Go to state 2 after 5 second.
      if (std::chrono::duration<double, std::milli>(currTime - tStartStateCalibHip).count() >= 5000)
      {
        // Transition to next state.
        system("clear");
        std::cout << "Process starts in 3 seconds, **REMOVE YOUR HAND** \n";
        sleep(1);
        system("clear");
        std::cout << "Process starts in 2 seconds, **REMOVE YOUR HAND** \n";
        sleep(1);
        system("clear");
        std::cout << "Process starts in 1 seconds, **REMOVE YOUR HAND** \n";
        sleep(1);
        system("clear");
        state = 2;
        tStartStateCalibThigh = std::chrono::high_resolution_clock::now();
      }
      else
      {
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode = queryMotorMode(MotorType::A1, MotorMode::BRAKE);
        cmd.id = 0;
        cmd.kp = 0.0;
        cmd.kd = 0.0;
        cmd.q = 0.0;
        cmd.dq = 0.0;
        cmd.tau = 0.0;

        serial.sendRecv(&cmd, &data);

        // bias = y / gear_ratio - ax
        offsetAngles[0] = data.q / queryGearRatio(MotorType::GO_M8010_6) - invertedIndex[0] * targetAngles[0];
        std::cout << "hip offset: " << offsetAngles[0] << "\n";
      }
    }

    // STATE MACHINE 2
    else if (state == 2)
    {
      // Go to state 3 after 15 second.
      if (std::chrono::duration<double, std::milli>(currTime - tStartStateCalibThigh).count() >= 10000)
      {
        // Transition to next state.
        system("clear");
        std::cout << "Process starts in 3 seconds, **REMOVE YOUR HAND** \n";
        sleep(1);
        system("clear");
        std::cout << "Process starts in 2 seconds, **REMOVE YOUR HAND** \n";
        sleep(1);
        system("clear");
        std::cout << "Process starts in 1 seconds, **REMOVE YOUR HAND** \n";
        sleep(1);
        system("clear");
        tStartStateCalibFinal = std::chrono::high_resolution_clock::now();
        state = 3;
      }
      else
      {
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id = 0;
        cmd.kp = 0.016;
        cmd.kd = 0.32;

        // y = (ax + bias) * gear_ratio
        cmd.q = (invertedIndex[0] * cmdAngles[0] + offsetAngles[0]) * queryGearRatio(MotorType::GO_M8010_6);
        cmd.dq = 0.0;
        cmd.tau = 0.0;

        serial.sendRecv(&cmd, &data);

        for (uint64_t id{1}; id < 3; ++id)
        {
          cmd.motorType = MotorType::A1;
          data.motorType = MotorType::A1;
          cmd.mode = queryMotorMode(MotorType::A1, MotorMode::FOC);
          cmd.id = id;
          cmd.kp = 0.0;
          cmd.kd = 0.0;

          cmd.q = 0;
          cmd.dq = 0.0;
          cmd.tau = 0.0;

          serial.sendRecv(&cmd, &data);

          // bias = y - a * x
          offsetAngles[id] = data.q / queryGearRatio(MotorType::GO_M8010_6) - invertedIndex[id] * targetAngles[id];

          std::cout << "[" << id << "] offset     : " << offsetAngles[id] << "\n";
          std::cout << "[" << id << "] go1 angle  : " << data.q / queryGearRatio(MotorType::GO_M8010_6) << "\n";
          std::cout << "[" << id << "] tar angle  : " << targetAngles[id] << "\n";
        }
      }
    }

    // STATE MACHINE 3
    else if (state == 3)
    {
      system("clear");
      std::cout << "enter numbers\n";
      std::cin >> q0;
      std::cin >> q1;
      std::cin >> q2;
      std::cout << "you enter: " << q0 << " " << q1 << " " << q2 << "\n";
      cmdAngles = {q0, q1, q2};

      for (uint64_t id{0}; id < 3; ++id)
      {
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode = queryMotorMode(MotorType::A1, MotorMode::FOC);
        cmd.id = id;
        cmd.kp = 0.016;
        cmd.kd = 0.32;

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
      // break;
    }
    serial.sendRecv(&cmd, &data);
    usleep(200);
  }
}
