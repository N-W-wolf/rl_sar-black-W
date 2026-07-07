/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BLACK_FSM_HPP
#define BLACK_FSM_HPP

#include "fsm_core.hpp"
#include "rl_sdk.hpp"
#include <cmath>
#include <stdexcept>

namespace black_fsm
{

constexpr double kPolicyDefaultDofPosTolerance = 1e-3;
constexpr double kGetUpDefaultPoseTolerance = 0.08;

struct RetryConfig
{
    int prepare_cycles = 100;
    std::vector<double> dof_pos;
    std::vector<double> kp;
    std::vector<double> kd;
};

inline std::vector<double> ReadRetryVector(const YAML::Node &node)
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

inline RetryConfig MakeDefaultRetryConfig(const RL &rl)
{
    RetryConfig config;
    config.prepare_cycles = 100;
    config.dof_pos.resize(rl.params.num_of_dofs);
    config.kp.resize(rl.params.num_of_dofs);
    config.kd.resize(rl.params.num_of_dofs);

    for (int i = 0; i < rl.params.num_of_dofs; ++i)
    {
        config.dof_pos[i] = rl.params.default_dof_pos[0][i].item<double>();
        config.kp[i] = rl.params.fixed_kp[0][i].item<double>();
        config.kd[i] = rl.params.fixed_kd[0][i].item<double>();
    }

    return config;
}

inline RetryConfig ReadRetryConfig(const RL &rl)
{
    RetryConfig config = MakeDefaultRetryConfig(rl);
    const std::string config_path = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/policy/" + rl.robot_name + "/retry_mode.yaml";
    try
    {
        YAML::Node root = YAML::LoadFile(config_path);
        YAML::Node node = root[rl.robot_name] ? root[rl.robot_name] : root;
        if (node["prepare_cycles"]) config.prepare_cycles = std::max(1, node["prepare_cycles"].as<int>());

        std::vector<double> dof_pos = ReadRetryVector(node["retry_default_dof_pos"]);
        std::vector<double> kp = ReadRetryVector(node["kp"]);
        std::vector<double> kd = ReadRetryVector(node["kd"]);
        if (static_cast<int>(dof_pos.size()) == rl.params.num_of_dofs) config.dof_pos = dof_pos;
        if (static_cast<int>(kp.size()) == rl.params.num_of_dofs) config.kp = kp;
        if (static_cast<int>(kd.size()) == rl.params.num_of_dofs) config.kd = kd;
    }
    catch (const std::exception &e)
    {
        std::cout << LOGGER::WARNING << "Failed to read retry_mode.yaml, using default retry config: " << e.what() << std::endl;
    }

    return config;
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

inline std::string SelectPolicySwitchState(RL &rl)
{
    std::string target_config;
    if (!rl.PeekPolicySwitchRequest(target_config))
    {
        return "";
    }

    if (rl.PolicyDefaultDofPosMatchesCurrent(target_config, kPolicyDefaultDofPosTolerance))
    {
        return "RLFSMStatePolicyReload";
    }
    return "RLFSMStatePolicyTransition";
}

inline bool CurrentPoseMatchesDefault(const RL &rl, const RobotState<double> &state, double tolerance)
{
    for (int i = 0; i < rl.params.num_of_dofs; ++i)
    {
        const double target = rl.params.default_dof_pos[0][i].item<double>();
        if (std::fabs(state.motor_state.q[i] - target) > tolerance)
        {
            return false;
        }
    }
    return true;
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

inline void ClearMotionCommand(RL &rl)
{
    rl.control.x = 0.0;
    rl.control.y = 0.0;
    rl.control.yaw = 0.0;
}

inline void ClearDiscreteCommand(RL &rl)
{
    rl.control.current_keyboard = rl.control.last_keyboard;
    rl.control.current_gamepad = Input::Gamepad::None;
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
        std::cout << LOGGER::NOTE << "Success enter black!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
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
        if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateGetUp : public RLFSMState
{
public:
    RLFSMStateGetUp(RL *rl) : RLFSMState(*rl, "RLFSMStateGetUp") {}

    float pre_running_percent = 0.0f;
    std::vector<float> pre_running_pos = {
        0.00,  1.4, -2.2,
        0.00, -1.4,  2.2,
        0.00,  1.4, -2.2,
        0.00, -1.4,  2.2
    };
    bool skip_pre_stage = false;
    std::vector<double> start_pos;

    void Enter() override
    {
        rl.running_percent = 0.0f;
        rl.now_state = *fsm_state;
        rl.start_state = rl.now_state;
        skip_pre_stage = CurrentPoseMatchesDefault(rl, rl.now_state, kGetUpDefaultPoseTolerance);
        pre_running_percent = skip_pre_stage ? 1.0f : 0.0f;
        start_pos.resize(rl.params.num_of_dofs);
        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            start_pos[i] = rl.now_state.motor_state.q[i];
        }
        if (skip_pre_stage)
        {
            std::cout << LOGGER::NOTE << "GetUp pre stage skipped: current pose is already near default pose." << std::endl;
        }
    }

    void Run() override
    {
        //printf("Running GetUp State\n");
        //std::cout << "\r\033[K" << " base_quat: " << rl.now_state.imu.quaternion<< std::flush;
        //std::cout << "angle vel: " << rl.now_state.imu.gyroscope << std::endl;
        if (pre_running_percent < 1.0f)
        {
            pre_running_percent += 1.0f / static_cast<float>(rl.getup_pre_cycles);
            pre_running_percent = std::min(pre_running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                fsm_command->motor_command.q[i] = (1 - pre_running_percent) * start_pos[i] + pre_running_percent * pre_running_pos[i];
                fsm_command->motor_command.dq[i] = 0;
                fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
                fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
                fsm_command->motor_command.tau[i] = 0;
            }
            std::cout << "\r\033[K" << std::flush << LOGGER::INFO << "Pre Getting up " << std::fixed << std::setprecision(2) << pre_running_percent * 100.0f << "%" << std::flush;
        }

        if (pre_running_percent == 1 && rl.running_percent < 1.0f)
        {
            rl.running_percent += 1.0f / static_cast<float>(rl.getup_cycles);
            rl.running_percent = std::min(rl.running_percent, 1.0f);

            for (int i = 0; i < rl.params.num_of_dofs; ++i)
            {
                const double phase_start = skip_pre_stage ? start_pos[i] : pre_running_pos[i];
                fsm_command->motor_command.q[i] = (1 - rl.running_percent) * phase_start + rl.running_percent * rl.params.default_dof_pos[0][i].item<double>();
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
            return "RLFSMStatePassive";
        }
        if (rl.running_percent == 1.0f)
        {
            if (rl.control.current_keyboard == Input::Keyboard::T || rl.control.current_gamepad == Input::Gamepad::Y)
            {
                const bool accepted = RequestNextPolicySwitch(rl);
                rl.control.current_keyboard = rl.control.last_keyboard;
                rl.control.current_gamepad = Input::Gamepad::None;
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
            else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
            {
                return "RLFSMStateRL_Locomotion";
            }
            else if (IsRetryCommand(rl))
            {
                ClearMotionCommand(rl);
                ClearDiscreteCommand(rl);
                return "RLFSMStateRetry";
            }
            else if (IsGetDownCommand(rl))
            {
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
        if (rl.control.current_keyboard == Input::Keyboard::P || rl.control.current_gamepad == Input::Gamepad::LB_X || rl.running_percent == 1.0f)
        {
            return "RLFSMStatePassive";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        return state_name_;
    }
};

class RLFSMStateRetry : public RLFSMState
{
public:
    RLFSMStateRetry(RL *rl) : RLFSMState(*rl, "RLFSMStateRetry") {}

    RetryConfig retry_config;
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
            fsm_command->motor_command.q[i] = (1 - prepare_percent) * start_pos[i] + prepare_percent * retry_config.dof_pos[i];
            fsm_command->motor_command.dq[i] = 0.0;
            fsm_command->motor_command.kp[i] = retry_config.kp[i];
            fsm_command->motor_command.kd[i] = retry_config.kd[i];
            fsm_command->motor_command.tau[i] = 0.0;
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
            rl.config_name = rl.GetNextPolicyConfig();
            if (rl.config_name.empty())
            {
                rl.config_name = "himloco";
            }
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
            return "RLFSMStateGetDown";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::Num0 || rl.control.current_gamepad == Input::Gamepad::A)
        {
            return "RLFSMStateGetUp";
        }
        else if (rl.control.current_keyboard == Input::Keyboard::T || rl.control.current_gamepad == Input::Gamepad::Y)
        {
            const bool accepted = RequestNextPolicySwitch(rl);
            rl.control.current_keyboard = rl.control.last_keyboard;
            rl.control.current_gamepad = Input::Gamepad::None;
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
        else if (rl.control.current_keyboard == Input::Keyboard::Num1 || rl.control.current_gamepad == Input::Gamepad::RB_DPadUp)
        {
            return "RLFSMStateRL_Locomotion";
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
    std::string target_config;
    std::vector<double> start_pos;
    torch::Tensor target_dof_pos;

    void Enter() override
    {
        transition_percent = 0.0f;
        transition_failed = false;
        target_config.clear();
        start_pos.clear();
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

        transition_percent += 1.0f / static_cast<float>(std::max(1, rl.policy_transition_cycles));
        transition_percent = std::min(transition_percent, 1.0f);

        for (int i = 0; i < rl.params.num_of_dofs; ++i)
        {
            const double target_pos = target_dof_pos[0][i].item<double>();
            fsm_command->motor_command.q[i] = (1 - transition_percent) * start_pos[i] + transition_percent * target_pos;
            fsm_command->motor_command.dq[i] = 0;
            fsm_command->motor_command.kp[i] = rl.params.fixed_kp[0][i].item<double>();
            fsm_command->motor_command.kd[i] = rl.params.fixed_kd[0][i].item<double>();
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

} // namespace black_fsm

class BLACKFSMFactory : public FSMFactory
{
public:
    BLACKFSMFactory(const std::string& initial) : initial_state_(initial) {}
    std::shared_ptr<FSMState> CreateState(void *context, const std::string &state_name) override
    {
        RL *rl = static_cast<RL *>(context);
        if (state_name == "RLFSMStatePassive")
            return std::make_shared<black_fsm::RLFSMStatePassive>(rl);
        else if (state_name == "RLFSMStateGetUp")
            return std::make_shared<black_fsm::RLFSMStateGetUp>(rl);
        else if (state_name == "RLFSMStateGetDown")
            return std::make_shared<black_fsm::RLFSMStateGetDown>(rl);
        else if (state_name == "RLFSMStateRetry")
            return std::make_shared<black_fsm::RLFSMStateRetry>(rl);
        else if (state_name == "RLFSMStateRL_Locomotion")
            return std::make_shared<black_fsm::RLFSMStateRL_Locomotion>(rl);
        else if (state_name == "RLFSMStatePolicyTransition")
            return std::make_shared<black_fsm::RLFSMStatePolicyTransition>(rl);
        else if (state_name == "RLFSMStatePolicyReload")
            return std::make_shared<black_fsm::RLFSMStatePolicyReload>(rl);
        return nullptr;
    }
    std::string GetType() const override { return "black"; }
    std::vector<std::string> GetSupportedStates() const override
    {
        return {
            "RLFSMStatePassive",
            "RLFSMStateGetUp",
            "RLFSMStateGetDown",
            "RLFSMStateRetry",
            "RLFSMStateRL_Locomotion",
            "RLFSMStatePolicyTransition",
            "RLFSMStatePolicyReload"
        };
    }
    std::string GetInitialState() const override { return initial_state_; }
private:
    std::string initial_state_;
};

REGISTER_FSM_FACTORY(BLACKFSMFactory, "RLFSMStatePassive")

#endif // BLACK_FSM_HPP
