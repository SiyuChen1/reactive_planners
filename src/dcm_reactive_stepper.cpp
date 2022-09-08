/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Define the DcmReactiveStepper class.
 */

#include "reactive_planners/dcm_reactive_stepper.hpp"

namespace reactive_planners
{
DcmReactiveStepper::DcmReactiveStepper(bool is_using_mpc)
{
    // Parameters
    control_period_ = 0.0;
    is_left_leg_in_contact_ = false;
    step_duration_ = 0.0;
    nb_force_ = 0;
    time_from_last_step_touchdown_ = 0.0;
    previous_support_foot_position_.setZero();
    current_support_foot_position_.setZero();
    next_support_foot_position_.setZero();
    desired_com_velocity_.setZero();
    right_foot_position_.setZero();
    right_foot_velocity_.setZero();
    right_foot_acceleration_.setZero();
    left_foot_position_.setZero();
    left_foot_velocity_.setZero();
    left_foot_acceleration_.setZero();
    feasible_com_velocity_.setZero();
    running_ = false;
    local_frame_.setIdentity();
    nb_usage_of_force_ = 0.;
    // new_ = true;
    // for debugging use new_ = false;
    new_ = is_using_mpc;
}

DcmReactiveStepper::~DcmReactiveStepper() = default;

void DcmReactiveStepper::set_steptime_nominal(double t_nom)
{
    dcm_vrp_planner_.set_steptime_nominal(t_nom);
}

void DcmReactiveStepper::initialize(
    const bool& is_left_leg_in_contact,
    const double& l_min,
    const double& l_max,
    const double& w_min,
    const double& w_max,
    const double& t_min,
    const double& t_max,
    const double& l_p,
    const double& com_height,
    const Eigen::Vector9d& weight,
    const double& mid_air_foot_height,
    const double& control_period,
    const double& planner_loop,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position)
{
    // Initialize the dcm vrp planner and initialize it.
    dcm_vrp_planner_.initialize(
        l_min, l_max, w_min, w_max, t_min, t_max, l_p, com_height, weight);

    // Parameters
    control_period_ = control_period;
    planner_loop_ = planner_loop;
    is_left_leg_in_contact_ = is_left_leg_in_contact;
    step_duration_ = 0.0;
    time_from_last_step_touchdown_ = 0.0;
    previous_support_foot_position_.setZero();
    current_support_foot_position_.setZero();
    next_support_foot_position_.setZero();
    desired_com_velocity_.setZero();
    right_foot_position_.setZero();
    right_foot_position_ = right_foot_position;
    right_foot_velocity_.setZero();
    right_foot_acceleration_.setZero();
    left_foot_position_.setZero();
    left_foot_position_ = left_foot_position;
    left_foot_velocity_.setZero();
    left_foot_acceleration_.setZero();
    local_right_foot_position_.setZero();
    local_right_foot_velocity_.setZero();
    local_left_foot_position_.setZero();
    local_left_foot_velocity_.setZero();
    feasible_com_velocity_.setZero();

    // Initialize the end-effector trajectory generator.
    dynamically_consistent_end_eff_trajectory_.set_mid_air_height(
        mid_air_foot_height);
    dynamically_consistent_end_eff_trajectory_.set_planner_loop(
        planner_loop_);

    polynomial_end_eff_trajectory_.set_mid_air_height(mid_air_foot_height);

    if (is_left_leg_in_contact_)
    {
        stepper_head_.set_support_feet_pos(right_foot_position,
                                           left_foot_position);
        current_support_foot_position_ = left_foot_position_;
        previous_support_foot_position_ = right_foot_position;
    }
    else
    {
        stepper_head_.set_support_feet_pos(left_foot_position,
                                           right_foot_position);
        current_support_foot_position_ = right_foot_position;
        previous_support_foot_position_ = left_foot_position_;
    }
    forces_.resize(ceil(t_max * 1000) * 3);
    running_ = false;
}

bool DcmReactiveStepper::run(
    double time,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& com_position,
    const Eigen::Ref<const Eigen::Vector3d>& com_velocity,
    const double& base_yaw,
    const bool& is_closed_loop)
{
    Eigen::Vector3d support_foot;
    if (is_left_leg_in_contact_)
        support_foot << left_foot_position(0), left_foot_position(1),
            dcm_vrp_planner_.get_com_height();
    else
        support_foot << right_foot_position(0), right_foot_position(1),
            dcm_vrp_planner_.get_com_height();

    std::cout<<"(c++, DcmReactiveStepper) current support foot is left foot ="<<is_left_leg_in_contact_<<std::endl;
    // base frame: CoP position [x, y](contact foot position on the ground)and with CoM height z_0
    // world_M_base:rotation: is rotation matrix between world frame and base frame
    // this rotation matrix is dependent on the initial configuration of q

    // world_M_base:translation: position of support_foot in world frame
    pinocchio::SE3 world_M_base(
        Eigen::AngleAxisd(base_yaw, Eigen::Vector3d::UnitZ())
            .toRotationMatrix(),
        support_foot);

    return run(time,
               left_foot_position,
               right_foot_position,
               left_foot_vel,
               right_foot_vel,
               com_position,
               com_velocity,
               world_M_base,
               is_closed_loop);
}

bool DcmReactiveStepper::run(
    double time,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& com_position,
    const Eigen::Ref<const Eigen::Vector3d>& com_velocity,
    const pinocchio::SE3& world_M_base,
    const bool& is_closed_loop)
{
    local_frame_ = world_M_base;
    bool succeed = true;
    std::cout<<"running_ = "<<running_<<std::endl;
    std::cout<<"time_from_last_step_touchdown_ = "<<time_from_last_step_touchdown_<<std::endl;
    std::cout<<"control_period_ = "<<control_period_<<std::endl;
    std::cout<<"step_duration_ = "<<step_duration_<<std::endl;
    std::cout<<"std::numeric_limits<double>::epsilon() = "<<std::numeric_limits<double>::epsilon()<<std::endl;
    if (running_ ||
        (!running_ && time_from_last_step_touchdown_ + control_period_ +
                              std::numeric_limits<double>::epsilon() <
                          step_duration_))
    {
        std::cout<<"I am walking"<<std::endl;
        walk(time,
             left_foot_position,
             right_foot_position,
             left_foot_vel,
             right_foot_vel,
             com_position,
             com_velocity,
             local_frame_,
             is_closed_loop);
    }
    else
    {   
        std::cout<<"I am standing still"<<std::endl;
        stand_still(time, left_foot_position, right_foot_position);
    }
    return succeed;
}

bool DcmReactiveStepper::walk(
    double time,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_vel,
    const Eigen::Ref<const Eigen::Vector3d>& com_position,
    const Eigen::Ref<const Eigen::Vector3d>& com_velocity,
    pinocchio::SE3& local_frame,
    const bool& is_closed_loop)
{
    Eigen::Vector3d left_on_ground = left_foot_position;
    left_on_ground[2] = 0;
    std::cout<<"left_on_ground:"<<left_on_ground<<std::endl;
    Eigen::Vector3d right_on_ground = right_foot_position;
    right_on_ground[2] = 0;
    std::cout<<"right_on_ground:"<<right_on_ground<<std::endl;
    
    bool succeed = true;
    // Run the scheduler of the planner.
    if (is_left_leg_in_contact_)
    {
        // parameter time is current time
        // parameter step_duration_  = 0.0
        std::cout<<"step_duration_:"<<step_duration_<<std::endl;
        std::cout<<"time in stepper_head_:"<<time<<std::endl;
        // right_on_ground is next support location
        // at first the support foot will not be switched
        stepper_head_.run(step_duration_, right_on_ground, time);
    }
    else
    {
        stepper_head_.run(step_duration_, left_on_ground, time);
    }
    // Extract the useful information.
    is_left_leg_in_contact_ = stepper_head_.get_is_left_leg_in_contact();
    
    std::cout<<"Is left leg in contact: "<<is_left_leg_in_contact_<<std::endl;
    
    if (is_left_leg_in_contact_)
    {
        stepper_head_.set_support_foot_pos(left_on_ground);
        std::cout<<"current_support_foot_position_ used for setting = "<<left_on_ground<<std::endl;
    }
    else
    {
        stepper_head_.set_support_foot_pos(right_on_ground);
        std::cout<<"current_support_foot_position_ used for setting = "<<right_on_ground<<std::endl;
    }
    if (is_closed_loop)
    {
        if (is_left_leg_in_contact_)
        {
            right_foot_position_ = right_foot_position;
            right_foot_velocity_ = right_foot_vel;
        }
        else
        {
            left_foot_position_ = left_foot_position;
            left_foot_velocity_ = left_foot_vel;
        }
    }
    time_from_last_step_touchdown_ =
        stepper_head_.get_time_from_last_step_touchdown();
    
    std::cout<<"time_from_last_step_touchdown_ = "<<time_from_last_step_touchdown_<<std::endl;

    // current support foot location = left_on_ground or right_on_ground
    // current_support_foot_position_[2] = 0
    current_support_foot_position_ =
        stepper_head_.get_current_support_location();

    
    std::cout<<"current_support_foot_position_ = "<<current_support_foot_position_<<std::endl;

    previous_support_foot_position_ =
        stepper_head_.get_previous_support_location();

    std::cout<<"previous_support_foot_position_ = "<<previous_support_foot_position_<<std::endl;

    /// change solver loop time_step
    if (new_ && time_from_last_step_touchdown_ == 0.0) nb_usage_of_force_ = 0;
    int planner_frequency = round(planner_loop_ * 1000);
    if (new_ && nb_usage_of_force_ % planner_frequency != 0)
    {   
        std::cout<<"Compute the flying foot trajectory."<<std::endl;
        // Compute the flying foot trajectory.
        if (is_left_leg_in_contact_)  // check which foot is in contact
        {   
            std::cout<<"Flying foot is the right foot"<<std::endl;
            // flying foot is the right foot
            dynamically_consistent_end_eff_trajectory_.update_robot_status(
                right_foot_position_,
                right_foot_velocity_,
                right_foot_acceleration_);
            // The current support foot does not move
            left_foot_position_ = current_support_foot_position_;
            left_foot_velocity_.setZero();
            left_foot_acceleration_.setZero();
        }
        else
        {
            // flying foot is the left foot
            dynamically_consistent_end_eff_trajectory_.update_robot_status(
                left_foot_position_,
                left_foot_velocity_,
                left_foot_acceleration_);
            // The current support foot does not move
            right_foot_position_ = current_support_foot_position_;
            right_foot_velocity_.setZero();
            right_foot_acceleration_.setZero();
        }
        nb_usage_of_force_ += 1;
        return true;
    }

    // the only difference between current_support_foot_position_ and support_foot is 
    // current_support_foot_position_[2] = 0
    // support_foot[2] = dcm_vrp_planner_.get_com_height()
    Eigen::Vector3d support_foot;
    if (is_left_leg_in_contact_)
        support_foot << left_on_ground(0), left_on_ground(1),
            dcm_vrp_planner_.get_com_height();
    else
        support_foot << right_on_ground(0), right_on_ground(1),
            dcm_vrp_planner_.get_com_height();
    
    // local_frame is transformation matrix between world frame and current base frame
    // update translation using current CoP
    local_frame.translation() = support_foot;
    
    nb_usage_of_force_ = 1;
    /// change solver loop time_step

    // Run the DcmVrpPlanner to get the next foot step location.
    double current_time = stepper_head_.get_time_from_last_step_touchdown();

    std::cout<<"current_time = "<<current_time<<std::endl;

    double new_t_min = 0.0;
    if (new_)
    {
        if (is_left_leg_in_contact_)
            new_t_min =
                dynamically_consistent_end_eff_trajectory_.calculate_t_min(
                    right_foot_position_,
                    right_foot_velocity_,
                    is_left_leg_in_contact_);
        else
            new_t_min =
                dynamically_consistent_end_eff_trajectory_.calculate_t_min(
                    left_foot_position_,
                    left_foot_velocity_,
                    is_left_leg_in_contact_);
    }

    std::cout<<"new_t_min = "<<new_t_min<<std::endl;

    dcm_vrp_planner_.update(current_support_foot_position_,
                            time_from_last_step_touchdown_,
                            is_left_leg_in_contact_,
                            desired_com_velocity_, // desired com velocity
                            com_position,
                            com_velocity,
                            local_frame,
                            new_t_min);

    succeed = succeed && dcm_vrp_planner_.solve();
    // Extract the useful information.
    // step duration is T in the paper
    step_duration_ = dcm_vrp_planner_.get_duration_before_step_landing();
    std::cout<<"step_duration_ = "<<step_duration_<<std::endl;

    next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();
    std::cout<<"next_support_foot_position_ = "<<next_support_foot_position_<<std::endl;

    double start_time = 0.0;

    // end_time is the calculated time adaptation for swing foot
    // see equation 18 and equation 20 (QP) 
    double end_time = dcm_vrp_planner_.get_duration_before_step_landing();
    std::cout<<"end_time = "<<end_time<<std::endl;

    // Compute the flying foot trajectory.
    if (is_left_leg_in_contact_)  // check which foot is in contact
    {
        // flying foot is the right foot
        if (new_)
        {
            succeed =
                succeed && dynamically_consistent_end_eff_trajectory_.compute(
                               previous_support_foot_position_,
                               right_foot_position_,
                               right_foot_velocity_,
                               next_support_foot_position_,
                               start_time,
                               current_time,
                               end_time,
                               is_left_leg_in_contact_);
            nb_force_ = dynamically_consistent_end_eff_trajectory_.get_forces(
                forces_,
                right_foot_position_,
                right_foot_velocity_,
                right_foot_acceleration_);
        }
        else
        {
            // Avoid updating the polynominal at the end of the trajectory.
            if (current_time <= end_time - 0.05)
            {
                // bool PolynomialEndEffectorTrajectory::compute(
                //     const Eigen::Ref<const Eigen::Vector3d>& start_pose,
                //     const Eigen::Ref<const Eigen::Vector3d>& current_pose,
                //     const Eigen::Ref<const Eigen::Vector3d>& current_velocity,
                //     const Eigen::Ref<const Eigen::Vector3d>& current_acceleration,
                //     const Eigen::Ref<const Eigen::Vector3d>& target_pose,
                //     const double& start_time,
                //     const double& current_time,
                //     const double& end_time)

                std::cout<<"current_time, end_time - 0.05"<<current_time<<", "<<end_time - 0.05<<std::endl;
                std::cout<<"do swing foot trajectory planning = "<<std::endl;
                succeed = succeed && polynomial_end_eff_trajectory_.compute(
                                        previous_support_foot_position_,
                                        right_foot_position_,
                                        right_foot_velocity_,
                                        right_foot_acceleration_,
                                        next_support_foot_position_,
                                        start_time,
                                        current_time,
                                        end_time);
            }
            polynomial_end_eff_trajectory_.get_next_state(
                current_time + control_period_,
                right_foot_position_,
                right_foot_velocity_,
                right_foot_acceleration_);
        }
        // The current support foot does not move
        left_foot_position_ = current_support_foot_position_;
        left_foot_velocity_.setZero();
        left_foot_acceleration_.setZero();
    }
    else
    {
        // flying foot is the left foot
        if (new_)
        {
            succeed =
                succeed && dynamically_consistent_end_eff_trajectory_.compute(
                               previous_support_foot_position_,
                               left_foot_position_,
                               left_foot_velocity_,
                               next_support_foot_position_,
                               start_time,
                               current_time,
                               end_time,
                               is_left_leg_in_contact_);
            nb_force_ = dynamically_consistent_end_eff_trajectory_.get_forces(
                forces_,
                left_foot_position_,
                left_foot_velocity_,
                left_foot_acceleration_);
        }
        else
        {
            // Avoid updating the polynominal at the end of the trajectory.
            if (current_time <= end_time - 0.05)
            {
                succeed = succeed && polynomial_end_eff_trajectory_.compute(
                                        previous_support_foot_position_,
                                        left_foot_position_,
                                        left_foot_velocity_,
                                        left_foot_acceleration_,
                                        next_support_foot_position_,
                                        start_time,
                                        current_time,
                                        end_time);
            }
            polynomial_end_eff_trajectory_.get_next_state(
                current_time + control_period_,
                left_foot_position_,
                left_foot_velocity_,
                left_foot_acceleration_);
        }
        // The current support foot does not move
        right_foot_position_ = current_support_foot_position_;
        right_foot_velocity_.setZero();
        right_foot_acceleration_.setZero();
    }
    // Compute the feasible velocity.
    feasible_com_velocity_ =
        (next_support_foot_position_ - previous_support_foot_position_) * 0.5;
    feasible_com_velocity_[2] = 0.0;
    dcm_ = dcm_vrp_planner_.get_dcm_local();
    return succeed;
}

bool DcmReactiveStepper::stand_still(
    double time,
    const Eigen::Ref<const Eigen::Vector3d>& left_foot_position,
    const Eigen::Ref<const Eigen::Vector3d>& right_foot_position)
{
    bool succeed = true;

    // Run the scheduler of the planner.
    if (is_left_leg_in_contact_)
    {
        stepper_head_.run(0.0, right_foot_position, time);
    }
    else
    {
        stepper_head_.run(0.0, left_foot_position, time);
    }
    // Extract the useful information.
    time_from_last_step_touchdown_ =
        stepper_head_.get_time_from_last_step_touchdown();
    current_support_foot_position_ =
        stepper_head_.get_current_support_location();
    previous_support_foot_position_ =
        stepper_head_.get_previous_support_location();
    is_left_leg_in_contact_ = stepper_head_.get_is_left_leg_in_contact();

    // Extract the useful information.
    step_duration_ = 0.0;
    next_support_foot_position_ = dcm_vrp_planner_.get_next_step_location();

    // Feet do not move.
    left_foot_position_(2) = 0.0;
    left_foot_velocity_.setZero();
    left_foot_acceleration_.setZero();
    right_foot_position_(2) = 0.0;
    right_foot_velocity_.setZero();
    right_foot_acceleration_.setZero();

    // Compute the feasible velocity.
    feasible_com_velocity_.setZero();

    return succeed;
}

}  // namespace reactive_planners