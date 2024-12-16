#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <array>
#include <vector>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <exception>


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

std::vector<MotorCmd> allMotorsZeroEffort(){
  std::cout << "All motors zero effort.\n";
  std::vector<MotorCmd> cmdVec;
  MotorCmd motorCmd{};
  motorCmd.kp = 0.0;
  motorCmd.kd = 0.0;
  motorCmd.q = 0.0;
  motorCmd.tau = 0.0;
  motorCmd.dq = 0.0;
  motorCmd.motorType = MotorType::A1;
  motorCmd.mode = queryMotorMode(MotorType::A1,MotorMode::FOC);

  for (int i{0}; i < 3; ++i) {
    motorCmd.id = i;
    cmdVec.push_back(motorCmd);
  }
  return cmdVec;
}

void systemInfoInit(){
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
}

void systemInfoWarn(){
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
}

void safetyMonitor(const std::vector<MotorData>& motorDataVec){
  double threshDq{30.0};
  double threshTau{1.0};

  for (const auto& motor : motorDataVec) {
    if (std::abs(motor.dq) > threshDq) {
      std::cerr << "velocity safety.\n";
      std::cout << "velocity was " << std::abs(motor.dq) << "\n";
      std::terminate();
    }

    if (std::abs(motor.tau) > threshTau) {
      std::cerr << "torque safety.\n";
      std::cout << "torque was " << std::abs(motor.tau) << "\n";
      std::terminate();
    }
  }
}

int main() {

  SerialPort serial("/dev/ttyUSB0");

  auto t_start = std::chrono::high_resolution_clock::now();
  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();

  auto tStartStateCalibHip = std::chrono::high_resolution_clock::now();
  auto tStartStateCalibThigh = std::chrono::high_resolution_clock::now();
  auto tStartStateCalibFinal = std::chrono::high_resolution_clock::now();

  int state{0}; // 0: init
                // 1: calibrate hip
                // 2: calibrate thigh
                // 3: finished

  MotorData templateMotorData{};
  templateMotorData.motorType = MotorType::A1;
  std::array<float, 3> storedQs{0.0, 0.0, 0.0};

  std::size_t ticks{0};

  std::vector<MotorData> motorDataVec;
  std::vector<MotorCmd> motorCmdVec;
  while(true){
    auto currTime{std::chrono::high_resolution_clock::now()};
    motorCmdVec.clear();

    // STATE MACHINE 0
    if (state == 0)
    {
      motorCmdVec = allMotorsZeroEffort();
      if (motorDataVec.size() == 3){
        systemInfoInit(); // This will stop the process for 3 seconds
        state = 1;
        tStartStateCalibHip = std::chrono::high_resolution_clock::now();
      }
    } 
    
    // STATE MACHINE 1
    else if (state == 1)
    {      
      // Go to state 2 after 5 second.
      if (std::chrono::duration<double, std::milli>(currTime-tStartStateCalibHip).count() >= 5000)
      {
        motorCmdVec = allMotorsZeroEffort();
        systemInfoWarn(); // This will stop the process for 3 seconds
        state = 2;
        tStartStateCalibThigh = std::chrono::high_resolution_clock::now();
      }
      else
      {
        motorCmdVec = allMotorsZeroEffort();
        // offset = motor_angle / gear_ratio - inverted_index * target_angle
        offsetAngles[0] = motorDataVec[0].q / queryGearRatio(MotorType::GO_M8010_6) - invertedIndex[0] *  targetAngles[0];
        std::cout << "hip offset: " << offsetAngles[0] << "\n";
      }
    }
    
    // STATE MACHINE 2
    else if (state == 2)
    {
      // Go to state 3 after 10 second.
      if (std::chrono::duration<double, std::milli>(currTime-tStartStateCalibThigh).count() >= 10000)
      {
        motorCmdVec = allMotorsZeroEffort();
        systemInfoWarn(); // This will stop the process for 3 seconds
        tStartStateCalibFinal = std::chrono::high_resolution_clock::now();
        state = 3;
      }
      else
      {
        MotorData data{};
        MotorCmd cmd{};

        for (uint64_t id{0}; id < 3; ++id){
          cmd.motorType = MotorType::A1;
          data.motorType = MotorType::A1;
          cmd.mode = queryMotorMode(MotorType::A1,MotorMode::FOC);
          cmd.id = id;
          cmd.dq = 0.0;
          cmd.tau = 0.0;

          if(id == 0){
            // motor_angle = (inverted_index * target_angle + offset) * gear_ratio
            cmd.q = (invertedIndex[0] * cmdAngles[0] + offsetAngles[0]) * queryGearRatio(MotorType::GO_M8010_6);
            cmd.kp = 0.016;
            cmd.kd = 0.32;
          }
          else{
            cmd.kp = 0.0;
            cmd.kd = 0.0;
            cmd.q = 0;
            // offset = motor_angle / gear_ratio - inverted_index * target_angle
            offsetAngles[id] = motorDataVec[id].q / queryGearRatio(MotorType::GO_M8010_6) - invertedIndex[id] * targetAngles[id];

            std::cout << "[" << id << "] offset     : " << offsetAngles[id] << "\n";
            std::cout << "[" << id << "] go1 angle  : " << motorDataVec[id].q / queryGearRatio(MotorType::GO_M8010_6) << "\n";
            std::cout << "[" << id << "] tar angle  : " << targetAngles[id] << "\n";
          }
          
          motorCmdVec.push_back(cmd);
          motorDataVec.push_back(data); 
        }
      }
    }
    
    // STATE MACHINE 3
    else if (state == 3)
    {
      system("clear");
      std::cout << "Enter motors' angles (hip, thig and cliff) \n";
      std::cin >> q0;
      std::cin >> q1;
      std::cin >> q2;
      std::cout << "you enter: " << q0 << " " << q1 << " " << q2 << "\n";
      
      // loop exit code
      if(q0 == 666){
        break;
      }

      cmdAngles = {q0, q1, q2};

      for (uint64_t id{0}; id < 3; ++id)
      {
        MotorData data{};
        MotorCmd cmd{};
        cmd.motorType = MotorType::A1;
        data.motorType = MotorType::A1;
        cmd.mode = queryMotorMode(MotorType::A1,MotorMode::FOC);
        cmd.id = id;
        cmd.kp = 0.016;
        cmd.kd = 0.32;

        // motor_angle = (inverted_index * target_angle + offset) * gear_ratio
        cmd.q = (invertedIndex[id] * cmdAngles[id] + offsetAngles[id]) * queryGearRatio(MotorType::GO_M8010_6);
        cmd.dq = 0.0;
        cmd.tau = 0.0;

        motorCmdVec.push_back(cmd);
        motorDataVec.push_back(data);
        
        std::cout << "[" << id << "] cmd angle  : " << cmdAngles[id] << "\n";
        std::cout << "[" << id << "] go1 angle  : " << motorDataVec[id].q / queryGearRatio(MotorType::GO_M8010_6) << "\n";
        std::cout << "[" << id << "] offset     : " << offsetAngles[id] << "\n";
        std::cout << "[" << id << "] cmd.q      : " << cmd.q << "\n";
      }
      std::cout << "\n";
    }

    // Safety monitoring.
    if(motorDataVec.size() == 3){
      safetyMonitor(motorDataVec);
    }
    
    motorDataVec.clear();
    for (uint32_t i{0}; i < 3; ++i){
      motorDataVec.push_back(templateMotorData);
    }

    if (motorCmdVec.size() != 3) {
      std::cerr << "Unexpected motorCmdVec size.\n";
      break;
    }
    serial.sendRecv(motorCmdVec, motorDataVec);
    usleep(200);
  }
  
  // Effort zero for all motors after shutdown
  motorCmdVec.clear();
  motorDataVec.clear();
  motorCmdVec = allMotorsZeroEffort();
  for (uint32_t i{0}; i < 3; ++i){motorDataVec.push_back(templateMotorData);}
  serial.sendRecv(motorCmdVec, motorDataVec);
}