/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BLACKW_FSM_HPP
#define BLACKW_FSM_HPP

#include "fsm_core.hpp"
#include "rl_sdk.hpp"
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace blackw_fsm
{

constexpr double kPolicyDefaultDofPosTolerance = 1e-3;

struct BridgeDriveConfig
{
    int prepare_cycles = 400;
    int exit_to_rl_cycles = 400;
    double max_x = 0.25;
    double max_yaw = 0.25;
    double wheel_velocity_scale = 6.0;
    double yaw_to_wheel_velocity = 2.0;
    std::vector<double> dof_pos;
    std::vector<double> kp;
    std::vector<double> kd;
    std::vector<double> wheel_velocity_sign;
};

inline std::vector<double> ReadBridgeVector(const YAML::Node &node)
{
    std::vector<double> values;
    if (!node || !node.IsSequence())
    {
        return values;
    }
    for (const YAML::Node &value : node)
    {
        values.push_back(value.as<double>());
    }
    return values;
}

inline bool IsWheelIndex(const RL &rl, int index)
{
    return std::find(rl.params.wheel_indices.begin(), rl.params.wheel_indices.end(), index) != rl.params.wheel_indices.end();
}

inline BridgeDriveConfig MakeDefaultDriveConfig(const RL &rl)
{
    BridgeDriveConfig config;
    config.prepare_cycles = 400;
    config.exit_to_rl_cycles = 400;
    config.max_x = 0.25;
    config.max_yaw = 0.25;
    config.wheel_velocity_scale = 6.0;
    config.yaw_to_wheel_velocity = 2.0;
    config.dof_pos.resize(rl.params.num_of_dofs);
    config.kp.resize(rl.params.num_of_dofs);
    config.kd.resize(rl.params.num_of_dofs);
    config.wheel_velocity_sign = std::vector<double>(rl.params.wheel_indices.size(), 1.0);

    for (int i = 0; i < rl.params.num_of_dofs; ++i)
    {
        config.dof_pos[i] = rl.params.default_dof_pos[0][i].item<double>();
        config.kp[i] = rl.params.fixed_kp[0][i].item<double>();
        config.kd[i] = rl.params.fixed_kd[0][i].item<double>();
    }

    return config;
}

inline BridgeDriveConfig ReadDriveConfig(
    const RL &rl,
    const std::string &config_file,
    const std::string &dof_pos_key,
    const std::string &mode_label)
{
    BridgeDriveConfig config = MakeDefaultDriveConfig(rl);
    const std::string config_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/policy/" + rl.robot_name + "/" + config_file;
    try
    {
        YAML::Node root = YAML::LoadFile(config_path);
        YAML::Node node = root[rl.robot_name] ? root[rl.robot_name] : root;
        if (node["prepare_cycles"]) config.prepare_cycles = std::max(1, node["prepare_cycles"].as<int>());
        if (node["exit_to_rl_cycles"]) config.exit_to_rl_cycles = std::max(1, node["exit_to_rl_cycles"].as<int>());
        if (node["max_x"]) config.max_x = std::max(0.0, node["max_x"].as<double>());
        if (node["max_yaw"]) config.max_yaw = std::max(0.0, node["max_yaw"].as<double>());
        if (node["wheel_velocity_scale"]) config.wheel_velocity_scale = node["wheel_velocity_scale"].as<double>();
        if (node["yaw_to_wheel_velocity"]) config.yaw_to_wheel_velocity = node["yaw_to_wheel_velocity"].as<double>();

        std::vector<double> dof_pos = ReadBridgeVector(node[dof_pos_key]);
        std::vector<double> kp = ReadBridgeVector(node["kp"]);
        std::vector<double> kd = ReadBridgeVector(node["kd"]);
        std::vector<double> wheel_velocity_sign = ReadBridgeVector(node["wheel_velocity_sign"]);
        if (static_cast<int>(dof_pos.size()) == rl.params.num_of_dofs) config.dof_pos = dof_pos;
        if (static_cast<int>(kp.size()) == rl.params.num_of_dofs) config.kp = kp;
        if (static_cast<int>(kd.size()) == rl.params.num_of_dofs) config.kd = kd;
        if (wheel_velocity_sign.size() == rl.params.wheel_indices.size()) config.wheel_velocity_sign = wheel_velocity_sign;
    }
    catch (const std::exception &e)
    {
        std::cout << LOGGER::WARNING << "Failed to read " << config_file
                  << ", using default " << mode_label << " config: " << e.what() << std::endl;
    }

    return config;
}

inline BridgeDriveConfig ReadBridgeDriveConfig(const RL &rl)
{
    return ReadDriveConfig(rl, "bridge_drive.yaml", "bridge_default_dof_pos", "bridge drive");
}

inline BridgeDriveConfig ReadLowBarDriveConfig(const RL &rl)
{
    return ReadDriveConfig(rl, "low_bar_drive.yaml", "low_bar_default_dof_pos", "low-bar drive");
}

inline BridgeDriveConfig ReadCarDriveConfig(const RL &rl)
{
    return ReadDriveConfig(rl, "car_drive.yaml", "car_default_dof_pos", "car drive");
}

inline BridgeDriveConfig ReadRetryConfig(const RL &rl)
{
    return ReadDriveConfig(rl, "retry_mode.yaml", "retry_default_dof_pos", "retry mode");
}

inline bool RequestNextPolicySwitch(RL &rl)
{
    const std::string target_config = rl.GetNextPolicyConfig();
    if (target_config.empty())
    {
        std::cout << LOGGER::WARNING << "No available policy_config for switch cycle." << std::endl;
        return false;
    }
    return rl.RequestPolicySwitch(target_config);
}

inline YAML::Node LoadPolicyConfigNode(const RL &rl, const std::string &config_name)
{
    const std::string config_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/policy/" + rl.robot_name + "/" + config_name + "/config.yaml";
    YAML::Node root = YAML::LoadFile(config_path);
    YAML::Node config = root[rl.robot_name + "/" + config_name];
    if (!config)
    {
        throw std::runtime_error("missing policy config node: " + rl.robot_name + "/" + config_name);
    }
    return config;
}

inline std::string SelectPolicySwitchState(RL &rl)
{
    std::string target_config;
    if (!rl.PeekPolicySwitchRequest(target_config))
    {
        return "";
    }

    try
    {
        const YAML::Node config = LoadPolicyConfigNode(rl, target_config);
        if (config && config["force_policy_transition"] && config["force_policy_transition"].as<bool>())
        {
            return "RLFSMStatePolicyTransition";
        }
    }
    catch (const std::exception &e)
    {
        std::cout << LOGGER::WARNING << "Failed to read policy switch options for '" << target_config
                  << "': " << e.what() << std::endl;
    }

    if (rl.PolicyDefaultDofPosMatchesCurrent(target_config, kPolicyDefaultDofPosTolerance))
    {
        return "RLFSMStatePolicyReload";
    }
    return "RLFSMStatePolicyTransition";
}

inline bool IsRetryCommand(const RL &rl)
{
    return rl.control.current_keyboard == Input::Keyboard::Num5 ||
        rl.control.current_gamepad == Input::Gamepad::B;
}

inline bool IsGetDownCommand(const RL &rl)
{
    return rl.control.current_keyboard == Input::Keyboard::Num9 ||
        rl.control.current_gamepad == Input::Gamepad::RB_B;
}

inline bool IsRLModeCommand(const RL &rl)
{
    return rl.control.current_keyboard == Input::Keyboard::Num1 ||
        rl.control.current_gamepad == Input::Gamepad::RB_DPadUp;
}

inline bool IsBridgeModeCommand(const RL &rl)
{
    return rl.control.current_keyboard == Input::Keyboard::Num2 ||
        rl.control.current_gamepad == Input::Gamepad::RB_DPadRight;
}

inline bool IsLowBarModeCommand(const RL &rl)
{
    return rl.control.current_keyboard == Input::Keyboard::Num3 ||
        rl.control.current_gamepad == Input::Gamepad::RB_DPadDown;
}

inline bool IsCarModeCommand(const RL &rl)
{
    return rl.control.current_keyboard == Input::Keyboard::Num4 ||
        rl.control.current_gamepad == Input::Gamepad::RB_DPadLeft;
}

inline bool RememberDriveModeCommand(RL &rl)
{
    Input::Keyboard target = Input::Keyboard::None;
    if (IsRLModeCommand(rl))
    {
        target = Input::Keyboard::Num1;
    }
    else if (IsBridgeModeCommand(rl))
    {
        target = Input::Keyboard::Num2;
    }
    else if (IsLowBarModeCommand(rl))
    {
        target = Input::Keyboard::Num3;
    }
    else if (IsCarModeCommand(rl))
    {
        target = Input::Keyboard::Num4;
    }

    if (target == Input::Keyboard::None)
    {
        return false;
    }

    rl.control.current_keyboard = target;
    rl.control.last_keyboard = Input::Keyboard::None;
    rl.control.current_gamepad = Input::Gamepad::None;
    rl.control.last_gamepad = Input::Gamepad::None;
    return true;
}

inline void ClearMotionCommand(RL &rl)
{
    rl.control.x = 0.0;
    rl.control.y = 0.0;
    rl.control.yaw = 0.0;
}

inline void ClearDiscreteCommand(RL &rl)
{
    rl.control.current_keyboard = Input::Keyboard::None;
    rl.control.last_keyboard = Input::Keyboard::None;
    rl.control.current_gamepad = Input::Gamepad::None;
    rl.control.last_gamepad = Input::Gamepad::None;
}

inline std::string ConsumeDriveModeCommandState(RL &rl)
{
    if (!RememberDriveModeCommand(rl))
    {
        return "";
    }

    std::string state;
    if (IsRLModeCommand(rl))
    {
        state = "RLFSMStateRL_Locomotion";
    }
    else if (IsBridgeModeCommand(rl))
    {
        state = "RLFSMStateBridgeDrive";
    }
    else if (IsLowBarModeCommand(rl))
    {
        state = "RLFSMStateLowBarDrive";
    }
    else if (IsCarModeCommand(rl))
    {
        state = "RLFSMStateCarDrive";
    }

    ClearDiscreteCommand(rl);
    return state;
}

inline void ClearPendingPolicySwitch(RL &rl)
{
    bool cleared = false;
    {
        std::lock_guard<std::mutex> lock(rl.policy_switch_mutex);
        if (!rl.policy_switch_in_progress &&
            (rl.policy_switch_requested || !rl.pending_config_name.empty() || !rl.policy_switch_done))
        {
            rl.policy_switch_requested = false;
            rl.pending_config_name.clear();
            rl.policy_switch_done = true;
            rl.policy_switch_success = true;
            cleared = true;
        }
    }
    if (cleared)
    {
        rl.PublishPolicySwitchDone(true);
        rl.PublishPolicySwitchStatus("retry command locked");
    }
}

class RLFSMStatePassive : public RLFSMState
{
public:
    RLFSMStatePassive(RL *rl) : RLFSMState(*rl, "RLFSMStatePassive") {}

    void Enter() override
    {
        rl.running_percent = 0.0f;
        std::cout << LOGGER::NOTE << "Entered passive mode. Press '0' (Keyboard) or 'A' (Gamepad) to switch to RLFSMStateGetUp." << std::endl;
        std::cout << LOGGER::NOTE << "Success enter blackW." << std::endl;
    }

    void Run() override
    {
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            //printf("Entering passive state for joint!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
            // fsm_command->motor_command.q[i] = fsm_state->motor_state.q[i];
            //printf("enter passive state\n");
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = 0;
            fsm_command->motor_command.kd[i] = 3;
            fsm_command->motor_command.tau[i] = 0;
            //std::cout << "now_state.motor_state.q[" << i << "] = " << rl.now_state.motor_state.q[i] << std::endl;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (RememberDriveModeCommand(rl))
        {
            return "RLFSMStateGetUp";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    bool start_state_recorded = false;
    std::vector<double> start_pos;

    void Enter() override
    {
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;
        if (!start_state_recorded)
        {
            rl.start_state = rl.now_state;
            start_state_recorded = true;
        }
        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }
    }

    void Run() override
    {
        //printf("Running GetUp State\n");
        //std::cout << "\r\033[K" << " base_quat: " << rl.now_state.imu.quaternion<< std::flush;
        //std::cout << "angle vel: " << rl.now_state.imu.gyroscope << std::endl;
        if (rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / static_cast<float>(rl.getup_cycles);
            rl.running_percent = std::min(rl.running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] = (1 - rl.running_percent) * start_pos[i] + rl.running_percent * rl.params.default_dof_pos[0][i].item<double>();
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "Getting up " << std::fixed << std::setprecision(2) << rl.running_percent * 100.0f << "%" << std::flush;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        if (rl.running_percent == 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::T || rl.control.current_gamepad == Input::Gamepad::Y)
            {
                const bool accepted = RequestNextPolicySwitch(rl);
                ClearDiscreteCommand(rl);
                if (accepted)
                {
                    const std::string switch_state = SelectPolicySwitchState(rl);
                    if (!switch_state.empty())
                    {
                        return switch_state;
                    }
                }
            }
            else if (rl.HasPolicySwitchRequest())
            {
                const std::string switch_state = SelectPolicySwitchState(rl);
                if (!switch_state.empty())
                {
                    return switch_state;
                }
            }
            else if (IsRLModeCommand(rl))
            {
                ClearDiscreteCommand(rl);
                return "RLFSMStateRL_Locomotion";
            }
            else if (IsBridgeModeCommand(rl))
            {
                ClearDiscreteCommand(rl);
                return "RLFSMStateBridgeDrive";
            }
            else if (IsLowBarModeCommand(rl))
            {
                ClearDiscreteCommand(rl);
                return "RLFSMStateLowBarDrive";
            }
            else if (IsCarModeCommand(rl))
            {
                ClearDiscreteCommand(rl);
                return "RLFSMStateCarDrive";
            }
            else if (IsRetryCommand(rl))
            {
                ClearMotionCommand(rl);
                ClearDiscreteCommand(rl);
                return "RLFSMStateRetry";
            }
            else if (IsGetDownCommand(rl))
            {
                ClearDiscreteCommand(rl);
                return "RLFSMStateGetDown";
            }
        }
        return state_name_;
    }
};

class RLFSMStateGetDown : public RLFSMState
{
public:
    RLFSMStateGetDown(RL *rl) : RLFSMState(*rl, "RLFSMStateGetDown") {}

    void Enter() override
    {
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;
    }

    void Run() override
    {
        if (rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / 500.0f;
            rl.running_percent = std::min(rl.running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] = (1 - rl.running_percent) * rl.now_state.motor_state.q[i] + rl.running_percent * rl.start_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "Getting down "<< std::fixed << std::setprecision(2) << rl.running_percent * 100.0f << "%" << std::flush;
        }
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        if (RememberDriveModeCommand(rl))
        {
            return "RLFSMStateGetUp";
        }
        if (rl.running_percent == 1.0f)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRetry : public RLFSMState
{
public:
    RLFSMStateRetry(RL *rl) : RLFSMState(*rl, "RLFSMStateRetry") {}

    BridgeDriveConfig retry_config;
    float prepare_percent = 0.0f;
    bool pose_ready = false;
    std::vector<double> start_pos;

    void Enter() override
    {
        retry_config = ReadRetryConfig(rl);
        prepare_percent = 0.0f;
        pose_ready = false;
        start_pos.clear();
        rl.rl_init_done = false;
        rl.ClearOutputQueues();
        ClearPendingPolicySwitch(rl);
        ClearMotionCommand(rl);
        rl.control.navigation_mode = false;
        rl.now_state = *fsm_state;

        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }

        std::cout << LOGGER::WARNING
                  << "Entered retry mode. Commands are locked for manual carry. "
                  << "Press '0'/A to GetUp, or 'P'/LB+X to Passive."
                  << std::endl;
    }

    void Run() override
    {
        ClearMotionCommand(rl);
        ClearPendingPolicySwitch(rl);
        rl.control.navigation_mode = false;
        rl.now_state = *fsm_state;

        if (!pose_ready)
        {
            prepare_percent += 1.0f / static_cast<float>(retry_config.prepare_cycles);
            prepare_percent = std::min(prepare_percent, 1.0f);
            if (prepare_percent == 1.0f)
            {
                pose_ready = true;
                std::cout << std::endl << LOGGER::WARNING << "Retry pose ready." << std::endl;
            }
        }

        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            if (IsWheelIndex(rl, i))
            {
                fsm_command->motor_command.q[i] = rl.now_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = retry_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
            else
            {
                fsm_command->motor_command.q[i] = (1 - prepare_percent) * start_pos[i] + prepare_percent * retry_config.dof_pos[i];
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = retry_config.kp[i];
                fsm_command->motor_command.kd[i] = retry_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
        }

        std::cout << "\r\033[K" << std::flush << LOGGER::WARNING
                  << "Retry mode: command locked, manual carry allowed"
                  << (pose_ready ? " ready" : " prepare ")
                  << std::fixed << std::setprecision(2) << prepare_percent * 100.0f << "%"
                  << std::flush;
    }

    void Exit() override
    {
        ClearMotionCommand(rl);
        ClearPendingPolicySwitch(rl);
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            ClearMotionCommand(rl);
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            ClearMotionCommand(rl);
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        ClearDiscreteCommand(rl);
        return state_name_;
    }
};

class RLFSMStateRL_Locomotion : public RLFSMState
{
public:
    RLFSMStateRL_Locomotion(RL *rl) : RLFSMState(*rl, "RLFSMStateRL_Locomotion") {}

    void Enter() override
    {
        rl.episode_length_buf = 0;
        rl.rl_init_done = false;
        const bool is_policy_switch = rl.policy_switch_in_progress;

        // Use command-line or launch override when provided.
        if (rl.config_name.empty())
        {
            rl.config_name = "himloco";
        }
        std::string robot_path = rl.robot_name + "/" + rl.config_name;
        try
        {
            std::lock_guard<std::mutex> lock(rl.model_mutex);
            rl.InitRL(robot_path);
            rl.ClearOutputQueues();
            rl.rl_init_done = true;
            if (is_policy_switch)
            {
                rl.FinishPolicySwitch(true);
            }
        }
        catch (const std::exception& e)
        {
            std::cout << LOGGER::ERROR << "InitRL() failed: " << e.what() << std::endl;
            rl.rl_init_done = false;
            if (is_policy_switch)
            {
                rl.FinishPolicySwitch(false);
            }
            rl.control.current_keyboard = Input::Keyboard::Num0;
        }

        // pos init
    }

    void Run() override
    {
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "RL Controller x:" << rl.control.x << " y:" << rl.control.y << " yaw:" << rl.control.yaw << std::flush;
        torch::Tensor _output_dof_pos, _output_dof_vel;
        if (rl.output_dof_pos_queue.try_pop(_output_dof_pos) && rl.output_dof_vel_queue.try_pop(_output_dof_vel))
        {
            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                if (_output_dof_pos.defined() && _output_dof_pos.numel() > 0)
                {
                    fsm_command->motor_command.q[i] = rl.output_dof_pos[0][i].item<double>();
                }
                if (_output_dof_vel.defined() && _output_dof_vel.numel() > 0)
                {
                    fsm_command->motor_command.dq[i] = rl.output_dof_vel[0][i].item<double>();
                }
                fsm_command->motor_command.kp[i] = rl.params.rl_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.rl_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
        }
    }

    void Exit() override
    {
        rl.rl_init_done = false;
    }

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        else if (IsRetryCommand(rl))
        {
            ClearMotionCommand(rl);
            ClearDiscreteCommand(rl);
            return "RLFSMStateRetry";
        }
        else if (IsGetDownCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::T || rl.control.current_gamepad == Input::Gamepad::Y)
        {
            const bool accepted = RequestNextPolicySwitch(rl);
            ClearDiscreteCommand(rl);
            if (accepted)
            {
                const std::string switch_state = SelectPolicySwitchState(rl);
                if (!switch_state.empty())
                {
                    return switch_state;
                }
            }
        }
        else if (rl.HasPolicySwitchRequest())
        {
            const std::string switch_state = SelectPolicySwitchState(rl);
            if (!switch_state.empty())
            {
                return switch_state;
            }
        }
        else if (IsRLModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return state_name_;
        }
        else if (IsBridgeModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateBridgeDrive";
        }
        else if (IsLowBarModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateLowBarDrive";
        }
        else if (IsCarModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateCarDrive";
        }
        return state_name_;
    }
};

class RLFSMStatePolicyTransition : public RLFSMState
{
public:
    RLFSMStatePolicyTransition(RL *rl) : RLFSMState(*rl, "RLFSMStatePolicyTransition") {}

    float transition_percent = 0.0f;
    bool transition_failed = false;
    bool enter_getup_after_transition = false;
    int transition_cycles = 1;
    std::string target_config;
    std::vector<double> start_pos;
    std::vector<double> transition_kp;
    std::vector<double> transition_kd;
    torch::Tensor target_dof_pos;

    void Enter() override
    {
        transition_percent = 0.0f;
        transition_failed = false;
        enter_getup_after_transition = false;
        transition_cycles = std::max(1, rl.policy_transition_cycles);
        target_config.clear();
        start_pos.clear();
        transition_kp.clear();
        transition_kd.clear();
        target_dof_pos = torch::Tensor();
        rl.rl_init_done = false;
        rl.ClearOutputQueues();
        rl.now_state = *fsm_state;

        if (!rl.BeginPolicySwitch(target_config))
        {
            transition_failed = true;
            return;
        }

        try
        {
            const YAML::Node target_config_node = LoadPolicyConfigNode(rl, target_config);
            if (target_config_node["policy_transition_cycles"])
            {
                transition_cycles = std::max(1, target_config_node["policy_transition_cycles"].as<int>());
            }
            if (target_config_node["enter_getup_after_transition"])
            {
                enter_getup_after_transition = target_config_node["enter_getup_after_transition"].as<bool>();
            }
            transition_kp = ReadBridgeVector(target_config_node["fixed_kp"]);
            transition_kd = ReadBridgeVector(target_config_node["fixed_kd"]);
            if (static_cast<int>(transition_kp.size()) != rl.params.num_of_dofs)
            {
                transition_kp.clear();
            }
            if (static_cast<int>(transition_kd.size()) != rl.params.num_of_dofs)
            {
                transition_kd.clear();
            }
            target_dof_pos = rl.ReadPolicyDefaultDofPos(rl.robot_name + "/" + target_config);
            if (target_dof_pos.size(1) != rl.params.num_of_dofs)
            {
                throw std::runtime_error("target default_dof_pos size does not match num_of_dofs");
            }
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR << "Policy transition failed to read target config '" << target_config << "': " << e.what() << std::endl;
            transition_failed = true;
            rl.FinishPolicySwitch(false);
            return;
        }

        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }

        std::cout << LOGGER::INFO << "Switching policy posture to " << target_config << std::endl;
    }

    void Run() override
    {
        if (transition_failed)
        {
            return;
        }

        transition_percent += 1.0f / static_cast<float>(transition_cycles);
        transition_percent = std::min(transition_percent, 1.0f);

        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            const double target_pos = target_dof_pos[0][i].item<double>();
            fsm_command->motor_command.q[i] = (1 - transition_percent) * start_pos[i] + transition_percent * target_pos;
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = transition_kp.empty() ? rl.params.fixed_kp[0][i].item<double>() : transition_kp[i];
            fsm_command->motor_command.kd[i] = transition_kd.empty() ? rl.params.fixed_kd[0][i].item<double>() : transition_kd[i];
            fsm_command->motor_command.tau[i] = 0;
        }
        std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "Policy posture switch " << std::fixed << std::setprecision(2) << transition_percent * 100.0f << "%" << std::flush;
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (transition_failed)
        {
            return "RLFSMStateGetUp";
        }
        if (transition_percent == 1.0f)
        {
            rl.config_name = target_config;
            if (enter_getup_after_transition)
            {
                try
                {
                    std::lock_guard<std::mutex> lock(rl.model_mutex);
                    rl.InitRL(rl.robot_name + "/" + rl.config_name);
                    rl.ClearOutputQueues();
                    rl.rl_init_done = false;
                }
                catch (const std::exception &e)
                {
                    std::cout << LOGGER::ERROR << "InitRL() failed after policy posture switch: " << e.what() << std::endl;
                    transition_failed = true;
                    rl.FinishPolicySwitch(false);
                    return "RLFSMStateGetUp";
                }
                return "RLFSMStateGetUp";
            }
            return "RLFSMStateRL_Locomotion";
        }
        return state_name_;
    }
};

class RLFSMStatePolicyReload : public RLFSMState
{
public:
    RLFSMStatePolicyReload(RL *rl) : RLFSMState(*rl, "RLFSMStatePolicyReload") {}

    bool reload_failed = false;
    std::string target_config;

    void Enter() override
    {
        reload_failed = false;
        target_config.clear();
        rl.rl_init_done = false;
        rl.ClearOutputQueues();

        if (!rl.BeginPolicySwitch(target_config))
        {
            reload_failed = true;
            return;
        }

        rl.config_name = target_config;
        std::cout << LOGGER::INFO << "Reloading policy without posture switch: " << target_config << std::endl;
    }

    void Run() override {}

    void Exit() override {}

    std::string CheckChange() override
    {
        if (reload_failed)
        {
            return "RLFSMStateGetUp";
        }
        return "RLFSMStateRL_Locomotion";
    }
};

class RLFSMStateBridgeDrive : public RLFSMState
{
public:
    RLFSMStateBridgeDrive(RL *rl) : RLFSMState(*rl, "RLFSMStateBridgeDrive") {}

    BridgeDriveConfig bridge_config;
    float prepare_percent = 0.0f;
    bool drive_ready = false;
    std::vector<double> start_pos;

    void Enter() override
    {
        bridge_config = ReadBridgeDriveConfig(rl);
        prepare_percent = 0.0f;
        drive_ready = false;
        rl.rl_init_done = false;
        rl.ClearOutputQueues();
        rl.now_state = *fsm_state;

        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }

        std::cout << LOGGER::INFO << "Entered bridge drive mode. Press '1' or RB+DPadUp to return to RL locomotion." << std::endl;
    }

    void Run() override
    {
        rl.now_state = *fsm_state;
        if (!drive_ready)
        {
            prepare_percent += 1.0f / static_cast<float>(bridge_config.prepare_cycles);
            prepare_percent = std::min(prepare_percent, 1.0f);
            if (prepare_percent == 1.0f)
            {
                drive_ready = true;
                std::cout << std::endl << LOGGER::INFO << "Bridge drive pose ready." << std::endl;
            }
        }

        const double x = clamp(rl.control.x, -bridge_config.max_x, bridge_config.max_x);
        const double yaw = clamp(rl.control.yaw, -bridge_config.max_yaw, bridge_config.max_yaw);
        const double left_velocity = bridge_config.wheel_velocity_scale * x - bridge_config.yaw_to_wheel_velocity * yaw;
        const double right_velocity = bridge_config.wheel_velocity_scale * x + bridge_config.yaw_to_wheel_velocity * yaw;

        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            const bool is_wheel = IsWheelIndex(rl, i);
            if (is_wheel)
            {
                const auto wheel_iter = std::find(rl.params.wheel_indices.begin(), rl.params.wheel_indices.end(), i);
                const int wheel_id = static_cast<int>(std::distance(rl.params.wheel_indices.begin(), wheel_iter));
                const bool is_left_wheel = i == 3 || i == 11;
                const double wheel_velocity = is_left_wheel ? left_velocity : right_velocity;
                fsm_command->motor_command.q[i] = rl.now_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = bridge_config.wheel_velocity_sign[wheel_id] * wheel_velocity;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = bridge_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
            else
            {
                const double target_pos = bridge_config.dof_pos[i];
                fsm_command->motor_command.q[i] = (1 - prepare_percent) * start_pos[i] + prepare_percent * target_pos;
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = bridge_config.kp[i];
                fsm_command->motor_command.kd[i] = bridge_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
        }

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "Bridge drive " << (drive_ready ? "ready" : "prepare")
                  << " x:" << x << " yaw:" << yaw << std::flush;
    }

    void Exit() override
    {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        else if (IsRetryCommand(rl))
        {
            ClearMotionCommand(rl);
            ClearDiscreteCommand(rl);
            return "RLFSMStateRetry";
        }
        else if (IsGetDownCommand(rl))
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        else if (IsRLModeCommand(rl))
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStateBridgeToRLTransition";
        }
        else if (IsBridgeModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return state_name_;
        }
        else if (IsLowBarModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateLowBarDrive";
        }
        else if (IsCarModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateCarDrive";
        }
        return state_name_;
    }
};

class RLFSMStateBridgeToRLTransition : public RLFSMState
{
public:
    RLFSMStateBridgeToRLTransition(RL *rl) : RLFSMState(*rl, "RLFSMStateBridgeToRLTransition") {}

    BridgeDriveConfig bridge_config;
    float transition_percent = 0.0f;
    bool transition_failed = false;
    std::vector<double> start_pos;
    torch::Tensor target_dof_pos;

    void Enter() override
    {
        bridge_config = ReadBridgeDriveConfig(rl);
        transition_percent = 0.0f;
        transition_failed = false;
        start_pos.clear();
        target_dof_pos = torch::Tensor();
        rl.rl_init_done = false;
        rl.ClearOutputQueues();
        rl.now_state = *fsm_state;

        if (rl.config_name.empty())
        {
            rl.config_name = "himloco";
        }

        try
        {
            target_dof_pos = rl.ReadPolicyDefaultDofPos(rl.robot_name + "/" + rl.config_name);
            if (target_dof_pos.size(1) != rl.params.num_of_dofs)
            {
                throw std::runtime_error("target default_dof_pos size does not match num_of_dofs");
            }
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR << "Bridge-to-RL transition failed to read config '" << rl.config_name << "': " << e.what() << std::endl;
            transition_failed = true;
            return;
        }

        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }

        std::cout << LOGGER::INFO << "Bridge drive returning to RL pose: " << rl.config_name << std::endl;
    }

    void Run() override
    {
        if (transition_failed)
        {
            return;
        }

        rl.now_state = *fsm_state;
        transition_percent += 1.0f / static_cast<float>(std::max(1, bridge_config.exit_to_rl_cycles));
        transition_percent = std::min(transition_percent, 1.0f);

        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            if (IsWheelIndex(rl, i))
            {
                fsm_command->motor_command.q[i] = rl.now_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = bridge_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
            else
            {
                const double target_pos = target_dof_pos[0][i].item<double>();
                fsm_command->motor_command.q[i] = (1 - transition_percent) * start_pos[i] + transition_percent * target_pos;
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = bridge_config.kp[i];
                fsm_command->motor_command.kd[i] = bridge_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
        }

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "Bridge to RL pose " << std::fixed << std::setprecision(2)
                  << transition_percent * 100.0f << "%" << std::flush;
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        else if (IsRetryCommand(rl))
        {
            ClearMotionCommand(rl);
            ClearDiscreteCommand(rl);
            return "RLFSMStateRetry";
        }
        else if (IsGetDownCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A || transition_failed)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        const std::string requested_state = ConsumeDriveModeCommandState(rl);
        if (!requested_state.empty())
        {
            return requested_state == "RLFSMStateRL_Locomotion" ? state_name_ : requested_state;
        }
        if (transition_percent == 1.0f)
        {
            return "RLFSMStateRL_Locomotion";
        }
        return state_name_;
    }
};

class RLFSMStateLowBarDrive : public RLFSMState
{
public:
    RLFSMStateLowBarDrive(RL *rl) : RLFSMState(*rl, "RLFSMStateLowBarDrive") {}

    BridgeDriveConfig low_bar_config;
    float prepare_percent = 0.0f;
    bool drive_ready = false;
    std::vector<double> start_pos;

    void Enter() override
    {
        low_bar_config = ReadLowBarDriveConfig(rl);
        prepare_percent = 0.0f;
        drive_ready = false;
        rl.rl_init_done = false;
        rl.ClearOutputQueues();
        rl.now_state = *fsm_state;

        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }

        std::cout << LOGGER::INFO << "Entered low-bar drive mode. Press '1' or RB+DPadUp to return to RL locomotion." << std::endl;
    }

    void Run() override
    {
        rl.now_state = *fsm_state;
        if (!drive_ready)
        {
            prepare_percent += 1.0f / static_cast<float>(low_bar_config.prepare_cycles);
            prepare_percent = std::min(prepare_percent, 1.0f);
            if (prepare_percent == 1.0f)
            {
                drive_ready = true;
                std::cout << std::endl << LOGGER::INFO << "Low-bar drive pose ready." << std::endl;
            }
        }

        const double x = clamp(rl.control.x, -low_bar_config.max_x, low_bar_config.max_x);
        const double yaw = clamp(rl.control.yaw, -low_bar_config.max_yaw, low_bar_config.max_yaw);
        const double left_velocity = low_bar_config.wheel_velocity_scale * x - low_bar_config.yaw_to_wheel_velocity * yaw;
        const double right_velocity = low_bar_config.wheel_velocity_scale * x + low_bar_config.yaw_to_wheel_velocity * yaw;

        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            const bool is_wheel = IsWheelIndex(rl, i);
            if (is_wheel)
            {
                const auto wheel_iter = std::find(rl.params.wheel_indices.begin(), rl.params.wheel_indices.end(), i);
                const int wheel_id = static_cast<int>(std::distance(rl.params.wheel_indices.begin(), wheel_iter));
                const bool is_left_wheel = i == 3 || i == 11;
                const double wheel_velocity = is_left_wheel ? left_velocity : right_velocity;
                fsm_command->motor_command.q[i] = rl.now_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = low_bar_config.wheel_velocity_sign[wheel_id] * wheel_velocity;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = low_bar_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
            else
            {
                const double target_pos = low_bar_config.dof_pos[i];
                fsm_command->motor_command.q[i] = (1 - prepare_percent) * start_pos[i] + prepare_percent * target_pos;
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = low_bar_config.kp[i];
                fsm_command->motor_command.kd[i] = low_bar_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
        }

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "Low-bar drive " << (drive_ready ? "ready" : "prepare")
                  << " x:" << x << " yaw:" << yaw << std::flush;
    }

    void Exit() override
    {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        else if (IsRetryCommand(rl))
        {
            ClearMotionCommand(rl);
            ClearDiscreteCommand(rl);
            return "RLFSMStateRetry";
        }
        else if (IsGetDownCommand(rl))
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        else if (IsRLModeCommand(rl))
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStateLowBarToRLTransition";
        }
        else if (IsBridgeModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateBridgeDrive";
        }
        else if (IsLowBarModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return state_name_;
        }
        else if (IsCarModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateCarDrive";
        }
        return state_name_;
    }
};

class RLFSMStateLowBarToRLTransition : public RLFSMState
{
public:
    RLFSMStateLowBarToRLTransition(RL *rl) : RLFSMState(*rl, "RLFSMStateLowBarToRLTransition") {}

    BridgeDriveConfig low_bar_config;
    float transition_percent = 0.0f;
    bool transition_failed = false;
    std::vector<double> start_pos;
    torch::Tensor target_dof_pos;

    void Enter() override
    {
        low_bar_config = ReadLowBarDriveConfig(rl);
        transition_percent = 0.0f;
        transition_failed = false;
        start_pos.clear();
        target_dof_pos = torch::Tensor();
        rl.rl_init_done = false;
        rl.ClearOutputQueues();
        rl.now_state = *fsm_state;

        if (rl.config_name.empty())
        {
            rl.config_name = "himloco";
        }

        try
        {
            target_dof_pos = rl.ReadPolicyDefaultDofPos(rl.robot_name + "/" + rl.config_name);
            if (target_dof_pos.size(1) != rl.params.num_of_dofs)
            {
                throw std::runtime_error("target default_dof_pos size does not match num_of_dofs");
            }
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR << "Low-bar-to-RL transition failed to read config '" << rl.config_name << "': " << e.what() << std::endl;
            transition_failed = true;
            return;
        }

        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }

        std::cout << LOGGER::INFO << "Low-bar drive returning to RL pose: " << rl.config_name << std::endl;
    }

    void Run() override
    {
        if (transition_failed)
        {
            return;
        }

        rl.now_state = *fsm_state;
        transition_percent += 1.0f / static_cast<float>(std::max(1, low_bar_config.exit_to_rl_cycles));
        transition_percent = std::min(transition_percent, 1.0f);

        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            if (IsWheelIndex(rl, i))
            {
                fsm_command->motor_command.q[i] = rl.now_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = low_bar_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
            else
            {
                const double target_pos = target_dof_pos[0][i].item<double>();
                fsm_command->motor_command.q[i] = (1 - transition_percent) * start_pos[i] + transition_percent * target_pos;
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = low_bar_config.kp[i];
                fsm_command->motor_command.kd[i] = low_bar_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
        }

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "Low-bar to RL pose " << std::fixed << std::setprecision(2)
                  << transition_percent * 100.0f << "%" << std::flush;
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        else if (IsRetryCommand(rl))
        {
            ClearMotionCommand(rl);
            ClearDiscreteCommand(rl);
            return "RLFSMStateRetry";
        }
        else if (IsGetDownCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A || transition_failed)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        const std::string requested_state = ConsumeDriveModeCommandState(rl);
        if (!requested_state.empty())
        {
            return requested_state == "RLFSMStateRL_Locomotion" ? state_name_ : requested_state;
        }
        if (transition_percent == 1.0f)
        {
            return "RLFSMStateRL_Locomotion";
        }
        return state_name_;
    }
};

class RLFSMStateCarDrive : public RLFSMState
{
public:
    RLFSMStateCarDrive(RL *rl) : RLFSMState(*rl, "RLFSMStateCarDrive") {}

    BridgeDriveConfig car_config;
    float prepare_percent = 0.0f;
    bool drive_ready = false;
    std::vector<double> start_pos;

    void Enter() override
    {
        car_config = ReadCarDriveConfig(rl);
        prepare_percent = 0.0f;
        drive_ready = false;
        rl.rl_init_done = false;
        rl.ClearOutputQueues();
        rl.now_state = *fsm_state;

        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }

        std::cout << LOGGER::INFO << "Entered car drive mode. Press '1' or RB+DPadUp to return to RL locomotion." << std::endl;
    }

    void Run() override
    {
        rl.now_state = *fsm_state;
        if (!drive_ready)
        {
            prepare_percent += 1.0f / static_cast<float>(car_config.prepare_cycles);
            prepare_percent = std::min(prepare_percent, 1.0f);
            if (prepare_percent == 1.0f)
            {
                drive_ready = true;
                std::cout << std::endl << LOGGER::INFO << "Car drive pose ready." << std::endl;
            }
        }

        const double x = clamp(rl.control.x, -car_config.max_x, car_config.max_x);
        const double yaw = clamp(rl.control.yaw, -car_config.max_yaw, car_config.max_yaw);
        const double left_velocity = car_config.wheel_velocity_scale * x - car_config.yaw_to_wheel_velocity * yaw;
        const double right_velocity = car_config.wheel_velocity_scale * x + car_config.yaw_to_wheel_velocity * yaw;

        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            const bool is_wheel = IsWheelIndex(rl, i);
            if (is_wheel)
            {
                const auto wheel_iter = std::find(rl.params.wheel_indices.begin(), rl.params.wheel_indices.end(), i);
                const int wheel_id = static_cast<int>(std::distance(rl.params.wheel_indices.begin(), wheel_iter));
                const bool is_left_wheel = i == 3 || i == 11;
                const double wheel_velocity = is_left_wheel ? left_velocity : right_velocity;
                fsm_command->motor_command.q[i] = rl.now_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = car_config.wheel_velocity_sign[wheel_id] * wheel_velocity;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = car_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
            else
            {
                const double target_pos = car_config.dof_pos[i];
                fsm_command->motor_command.q[i] = (1 - prepare_percent) * start_pos[i] + prepare_percent * target_pos;
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = car_config.kp[i];
                fsm_command->motor_command.kd[i] = car_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
        }

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "Car drive " << (drive_ready ? "ready" : "prepare")
                  << " x:" << x << " yaw:" << yaw << std::flush;
    }

    void Exit() override
    {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        else if (IsRetryCommand(rl))
        {
            ClearMotionCommand(rl);
            ClearDiscreteCommand(rl);
            return "RLFSMStateRetry";
        }
        else if (IsGetDownCommand(rl))
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        else if (IsRLModeCommand(rl))
        {
            rl.control.x = 0.0;
            rl.control.y = 0.0;
            rl.control.yaw = 0.0;
            ClearDiscreteCommand(rl);
            return "RLFSMStateCarToRLTransition";
        }
        else if (IsBridgeModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateBridgeDrive";
        }
        else if (IsLowBarModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateLowBarDrive";
        }
        else if (IsCarModeCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return state_name_;
        }
        return state_name_;
    }
};

class RLFSMStateCarToRLTransition : public RLFSMState
{
public:
    RLFSMStateCarToRLTransition(RL *rl) : RLFSMState(*rl, "RLFSMStateCarToRLTransition") {}

    BridgeDriveConfig car_config;
    float transition_percent = 0.0f;
    bool transition_failed = false;
    std::vector<double> start_pos;
    torch::Tensor target_dof_pos;

    void Enter() override
    {
        car_config = ReadCarDriveConfig(rl);
        transition_percent = 0.0f;
        transition_failed = false;
        start_pos.clear();
        target_dof_pos = torch::Tensor();
        rl.rl_init_done = false;
        rl.ClearOutputQueues();
        rl.now_state = *fsm_state;

        if (rl.config_name.empty())
        {
            rl.config_name = "himloco";
        }

        try
        {
            target_dof_pos = rl.ReadPolicyDefaultDofPos(rl.robot_name + "/" + rl.config_name);
            if (target_dof_pos.size(1) != rl.params.num_of_dofs)
            {
                throw std::runtime_error("target default_dof_pos size does not match num_of_dofs");
            }
        }
        catch (const std::exception &e)
        {
            std::cout << LOGGER::ERROR << "Car-to-RL transition failed to read config '" << rl.config_name << "': " << e.what() << std::endl;
            transition_failed = true;
            return;
        }

        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }

        std::cout << LOGGER::INFO << "Car drive returning to RL pose: " << rl.config_name << std::endl;
    }

    void Run() override
    {
        if (transition_failed)
        {
            return;
        }

        rl.now_state = *fsm_state;
        transition_percent += 1.0f / static_cast<float>(std::max(1, car_config.exit_to_rl_cycles));
        transition_percent = std::min(transition_percent, 1.0f);

        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            if (IsWheelIndex(rl, i))
            {
                fsm_command->motor_command.q[i] = rl.now_state.motor_state.q[i];
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = 0.0;
                fsm_command->motor_command.kd[i] = car_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
            else
            {
                const double target_pos = target_dof_pos[0][i].item<double>();
                fsm_command->motor_command.q[i] = (1 - transition_percent) * start_pos[i] + transition_percent * target_pos;
                fsm_command->motor_command.dq[i] = 0.0;
                fsm_command->motor_command.kp[i] = car_config.kp[i];
                fsm_command->motor_command.kd[i] = car_config.kd[i];
                fsm_command->motor_command.tau[i] = 0.0;
            }
        }

        std::cout << "\r\033[K" << std::flush << LOGGER::INFO
                  << "Car to RL pose " << std::fixed << std::setprecision(2)
                  << transition_percent * 100.0f << "%" << std::flush;
    }

    void Exit() override {}

    std::string CheckChange() override
    {
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStatePassive";
        }
        else if (IsRetryCommand(rl))
        {
            ClearMotionCommand(rl);
            ClearDiscreteCommand(rl);
            return "RLFSMStateRetry";
        }
        else if (IsGetDownCommand(rl))
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A || transition_failed)
        {
            ClearDiscreteCommand(rl);
            return "RLFSMStateGetUp";
        }
        const std::string requested_state = ConsumeDriveModeCommandState(rl);
        if (!requested_state.empty())
        {
            return requested_state == "RLFSMStateRL_Locomotion" ? state_name_ : requested_state;
        }
        if (transition_percent == 1.0f)
        {
            return "RLFSMStateRL_Locomotion";
        }
        return state_name_;
    }
};

} // namespace blackw_fsm

class BLACKWFSMFactory : public FSMFactory
{
public:
    BLACKWFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<blackw_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<blackw_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<blackw_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRetry")
            return std::make_shared<blackw_fsm::RLFSMStateRetry>(rl);
        else if (state_name == "RLFSMStateRL_Locomotion")
            return std::make_shared<blackw_fsm::RLFSMStateRL_Locomotion>(rl);
        else if (state_name == "RLFSMStatePolicyTransition")
            return std::make_shared<blackw_fsm::RLFSMStatePolicyTransition>(rl);
        else if (state_name == "RLFSMStatePolicyReload")
            return std::make_shared<blackw_fsm::RLFSMStatePolicyReload>(rl);
        else if (state_name == "RLFSMStateBridgeDrive")
            return std::make_shared<blackw_fsm::RLFSMStateBridgeDrive>(rl);
        else if (state_name == "RLFSMStateBridgeToRLTransition")
            return std::make_shared<blackw_fsm::RLFSMStateBridgeToRLTransition>(rl);
        else if (state_name == "RLFSMStateLowBarDrive")
            return std::make_shared<blackw_fsm::RLFSMStateLowBarDrive>(rl);
        else if (state_name == "RLFSMStateLowBarToRLTransition")
            return std::make_shared<blackw_fsm::RLFSMStateLowBarToRLTransition>(rl);
        else if (state_name == "RLFSMStateCarDrive")
            return std::make_shared<blackw_fsm::RLFSMStateCarDrive>(rl);
        else if (state_name == "RLFSMStateCarToRLTransition")
            return std::make_shared<blackw_fsm::RLFSMStateCarToRLTransition>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "blackW"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRetry",
            "RLFSMStateRL_Locomotion",
            "RLFSMStatePolicyTransition",
            "RLFSMStatePolicyReload",
            "RLFSMStateBridgeDrive",
            "RLFSMStateBridgeToRLTransition",
            "RLFSMStateLowBarDrive",
            "RLFSMStateLowBarToRLTransition",
            "RLFSMStateCarDrive",
            "RLFSMStateCarToRLTransition"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(BLACKWFSMFactory, "RLFSMStatePassive")

#endif // BLACKW_FSM_HPP
