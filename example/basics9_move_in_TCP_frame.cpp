/**
 * @example basics9_move_in_TCP_frame.cpp
 * @brief NRT Cartesian-space pure motion control in TCP frame
 * @date 2024-02-05
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 * @author Flexiv
 */
#include <flexiv/Robot.hpp>
#include <flexiv/Log.hpp>
#include <flexiv/Scheduler.hpp>
#include <flexiv/Utility.hpp>

#include <iostream>
#include <cmath>
#include <thread>
namespace {
/**
 * @brief Maximum Cartesian linear velocity when moving to the target pose.
Default maximum linear velocity is used when set to 0. Unit: \f$ [m/s] \f$.
 */

constexpr double k_maxLinearVel = 0.05;
/**
 * @brief maxLinearVel maxAngularVel Maximum Cartesian angular velocity when moving to the target
pose.Default maximum angular velocity is used when set to 0. Unit: \f$[rad / s] \f$.
 */
constexpr double k_maxAngularVel = 0.05;
}
/** @brief Print tutorial description */
void printDescription()
{
    std::cout << "This tutorial runs a NRT cartesian-space pure motion control in TCP frame"
              << std::endl
              << std::endl;
}

/** @brief Print program usage help */
void printHelp()
{
    // clang-format off
    std::cout << "Required arguments: [robot SN]" << std::endl;
    std::cout << "    robot SN: Serial number of the robot to connect to. "
                 "Remove any space, for example: Rizon4s-123456" << std::endl;
    std::cout << std::endl;
    // clang-format on
}

int main(int argc, char* argv[])
{
    // Program Setup
    // ================================================
    // Logger for printing message with timestamp and coloring
    flexiv::Log log;

    // Parse parameters
    if (argc < 2 || flexiv::utility::programArgsExistAny(argc, argv, {"-h", "--help"})) {
        printHelp();
        return 1;
    }
    // Serial number of the robot to connect to. Remove any space, for example: Rizon4s-123456
    std::string robotSN = argv[1];

    // Print description
    log.info("Tutorial description:");
    printDescription();

    try {
        // RDK Initialization
        // ============================================
        // Instantiate robot interface
        flexiv::Robot robot(robotSN);

        // Create data struct for storing robot states
        flexiv::RobotStates robotStates;

        // Clear fault on robot server if any
        if (robot.isFault()) {
            log.warn("Fault occurred on robot server, trying to clear ...");
            // Try to clear the fault
            robot.clearFault();
            std::this_thread::sleep_for(std::chrono::seconds(2));
            // Check again
            if (robot.isFault()) {
                log.error("Fault cannot be cleared, exiting ...");
                return 1;
            }
            log.info("Fault on robot server is cleared");
        }

        // Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...");
        robot.enable();

        // Wait for the robot to become operational
        while (!robot.isOperational()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Robot is now operational");

        // Move robot to home pose
        log.info("Moving to home pose");
        robot.setMode(flexiv::Mode::NRT_PRIMITIVE_EXECUTION);
        robot.executePrimitive("Home()");

        // Wait for the primitive to finish
        while (robot.isBusy()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        log.info("Now robot is already at home");

        // Switch to NRT mode for continuous motion control
        robot.setMode(flexiv::Mode::NRT_CARTESIAN_MOTION_FORCE);

        // Set initial pose to current TCP pose
        auto initPose = robot.getRobotStates().tcpPose;
        log.info("Initial TCP pose set to [position 3x1, rotation (quaternion) 4x1]: "
                 + flexiv::utility::arr2Str(initPose));
        // =========code implementation to move relatively in tcp frame===================

        // Get current homogeneous transformation matrix from tcp to world
        Eigen::Matrix4d h_Tcp2World;
        auto tcpPoseInWorld = flexiv::utility::array2EigenVector(robot.getRobotStates().tcpPose);
        h_Tcp2World.block<3, 1>(0, 3) = tcpPoseInWorld.head(3);
        h_Tcp2World.block<3, 3>(0, 0)
            = flexiv::utility::eigenVector4d2Quat(tcpPoseInWorld.tail(4)).toRotationMatrix();

        // Set target pose in TCP frame
        // Rot 45 degree around z-axis and move 5cm along x-axis in TCP frame
        std::array<double, 3> targetZYXEulerInTcp = {45.0 * M_PI / 180.0, 0, 0};
        std::array<double, 3> targetPosInTcp = {0.05, 0, 0};

        // Compose target pose in TCP frame
        Eigen::Matrix4d targetPoseInTcp = Eigen::Matrix4d::Identity();
        targetPoseInTcp.block<3, 1>(0, 3) = flexiv::utility::array2EigenVector(targetPosInTcp);
        targetPoseInTcp.block<3, 3>(0, 0)
            = (flexiv::utility::eulerZYX2Quat(targetZYXEulerInTcp)).toRotationMatrix();
        // Transform target pose in TCP frame to world frame
        Eigen::Matrix4d targetPose = h_Tcp2World * targetPoseInTcp;

        // Compose target pose in world frame
        Eigen::Quaterniond targetQ(targetPose.block<3, 3>(0, 0));
        std::array<double, 7> targetPoseInWorld = {targetPose(0, 3), targetPose(1, 3),
            targetPose(2, 3), targetQ.w(), targetQ.x(), targetQ.y(), targetQ.z()};

        // =========code implementation to move relatively in tcp frame===================

        // Send target pose to robot and limit the velocity. Keep target wrench 0 at this
        // stage since we are not doing force control yet
        // robot.sendCartesianMotionForce(targetPoseInWorld, {}, k_maxLinearVel, k_maxAngularVel);
        robot.sendCartesianMotionForce(targetPoseInWorld, {});

        std::this_thread::sleep_for(std::chrono::seconds(5));

    } catch (const std::exception& e) {
        log.error(e.what());
        return 1;
    }

    return 0;
}
