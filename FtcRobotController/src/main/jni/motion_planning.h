#ifndef MOTION_PLANNING
#define MOTION_PLANNING

#include "maths.h"
#define point_amount 1000
/*Start of Waypoints*/
struct waypoint
{
    float x, y, theta;
    waypoint()
    {
    }
    waypoint(float x_in, float y_in, float theta_in)
    {
        x = x_in;
        y = y_in;
        theta = theta_in;
    }
};

struct waypointSequence
{
    int current_amount;//Actual number of waypoints
    int amount;//Total storage
    waypoint sequence[point_amount];

    waypointSequence(int n)
    {
        current_amount = 0;
        if(n > point_amount) n = point_amount;
        amount = n;
        waypoint b(-1,-1,-1);
        for(int i = 0; i < point_amount; i++)
            sequence[i] = b;
    }

    bool addWaypoint(waypoint w)
    {
        if(current_amount < amount)
        {
            sequence[current_amount++] = w;
            return true;
        }
        return false;
    }

    waypoint getWaypoint(int n)
    {
        if(n >= 0 && n < current_amount) return sequence[n];
                
        else assert(0); //throw 1
    }

    waypointSequence invertY() //Used for red/blue switching
    {
        waypointSequence inverted(amount);
        inverted.current_amount = current_amount;
        for(int i = 0; i < current_amount; i++)
        {
            inverted.sequence[i] = sequence[i];
            inverted.sequence[i].y *= -1;
            inverted.sequence[i].theta = boundAngle0to2PiRadians(2*pi - inverted.sequence[i].theta);
        }
        return inverted;
    }
};
/*End of Waypoints*/
struct quinticSpline //https://www.desmos.com/calculator/ctbggy8gbc
{
    union
    {
        struct
        {
            float a;
            float b;
            float c;
            float d;
            float e;
        };
        float data[5];
    };
    inline float & operator[](int a)
    {
        return data[a];
    }
    float x_offset;
    float y_offset;
    float knot_distance;
    float theta_offset;
    float arc_length;
    float yp0_hat;
    float yp1_hat;
    float theta0_hat;
    float theta1_hat;
    float x1_hat;
    quinticSpline(){
        x_offset = 0;
        y_offset = 0;
        knot_distance = 0;
        theta_offset = 0;
        arc_length = -1;
        yp0_hat = 0;
        yp1_hat = 0;
        theta0_hat = 0;
        theta1_hat = 0;
        x1_hat = 0;
        a = 0;
        b = 0;
        c = 0;
        d = 0;
        e = 0;
    }
    void reticulateSplines(float x0, float y0, float theta_0, float x1, float y1, float theta_1)//TODO: Add safetys so we don't crash
    {

        x_offset = x0;
        y_offset = y0;

        x1_hat = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
        if (x1_hat == 0) {
            printf("X1 Hat is 0, cannot handle this\n");
        }

        knot_distance = x1_hat;
        theta_offset = atan2(y1 - y0, x1 - x0);
        theta0_hat = getDifferenceInAngleRadians(theta_offset, theta_0);
        theta1_hat = getDifferenceInAngleRadians(theta_offset, theta_1);
        yp0_hat = tan(theta0_hat);
        yp1_hat = tan(theta1_hat);

        a = -(3 * (yp0_hat + yp1_hat)) / (x1_hat * x1_hat * x1_hat * x1_hat);
        b = (8 * yp0_hat + 7 * yp1_hat) / (x1_hat * x1_hat * x1_hat);
        c = -(6 * yp0_hat + 4 * yp1_hat) / (x1_hat * x1_hat);
        d = 0;
        e = yp0_hat;
    }
    void reticulateSplines(waypoint initial_waypoint, waypoint final_waypoint)
    {

        x_offset = initial_waypoint.x;
        y_offset = initial_waypoint.y;

        x1_hat = sqrt(sq(final_waypoint.x - initial_waypoint.x) + sq(final_waypoint.y - initial_waypoint.y));
        if (x1_hat == 0) {
            printf("X1 Hat is 0, cannot handle this\n");
        }

        knot_distance = x1_hat;
        theta_offset = atan2(final_waypoint.y - initial_waypoint.y, final_waypoint.x - initial_waypoint.x);
        theta0_hat = getDifferenceInAngleRadians(theta_offset, initial_waypoint.theta);
        theta1_hat = getDifferenceInAngleRadians(theta_offset, final_waypoint.theta);
        yp0_hat = tan(theta0_hat);
        yp1_hat = tan(theta1_hat);

        a = -(3 * (yp0_hat + yp1_hat)) / (x1_hat * x1_hat * x1_hat * x1_hat);
        b = (8 * yp0_hat + 7 * yp1_hat) / (x1_hat * x1_hat * x1_hat);
        c = -(6 * yp0_hat + 4 * yp1_hat) / (x1_hat * x1_hat);
        d = 0;
        e = yp0_hat;
    }

    float calculateLength()
    {
        if (arc_length >= 0) {
            return arc_length;
        }

        const int kNumSamples = 100000;
        float arc_length_specific = 0;
        float t, dydt;
        float integrand = sqrt(1 + derivativeAt(0) * derivativeAt(0)) / kNumSamples;
        float last_integrand = sqrt(1 + derivativeAt(0) * derivativeAt(0)) / kNumSamples;
        for (int i = 1; i <= kNumSamples; ++i)
        {
            t = ((float) i) / kNumSamples;
            dydt = derivativeAt(t);
            integrand = sqrt(1 + dydt * dydt) / kNumSamples;
            arc_length_specific += (integrand + last_integrand) / 2;
            last_integrand = integrand;
        }
        arc_length  = knot_distance * arc_length_specific;
        return arc_length;
    }

    float getPercentageForDistance(float distance)
    {
        const int kNumSamples = 100000;
        float arc_length_specific = 0;
        float t = 0;
        float last_arc_length = 0;
        float dydt;
        float integrand, last_integrand = sqrt(1 + derivativeAt(0) * derivativeAt(0)) / kNumSamples;
        distance /= knot_distance;
        for (int i = 1; i <= kNumSamples; ++i)
        {
            t = ((float) i) / kNumSamples;
            dydt = derivativeAt(t);
            integrand = sqrt(1 + dydt * dydt) / kNumSamples;
            arc_length_specific += (integrand + last_integrand) / 2;
            if (arc_length_specific > distance) {
                break;
            }
            last_integrand = integrand;
            last_arc_length = arc_length_specific;
        }

        // Interpolate between samples.
        float interpolated = t;
        if (arc_length_specific != last_arc_length)
        {
            interpolated += ((distance - last_arc_length)
                             / (arc_length_specific - last_arc_length) - 1) / (float) kNumSamples;
        }
        return interpolated;
    }

    v2f getXandY(float percentage) {
        v2f result;
        percentage = max(min(percentage, 1), 0);
        float x_hat = percentage * knot_distance;
        float y_hat = (a * x_hat + b) * x_hat * x_hat * x_hat * x_hat
            + c * x_hat * x_hat * x_hat + d * x_hat * x_hat + e * x_hat;

        float cos_theta = cos(theta_offset);
        float sin_theta = sin(theta_offset);

        result[0] = x_hat * cos_theta - y_hat * sin_theta + x_offset;
        result[1] = x_hat * sin_theta + y_hat * cos_theta + y_offset;
        return result;
    }

    float valueAt(float percentage)
    {
        percentage = max(min(percentage, 1), 0);
        float x_hat = percentage * knot_distance;
        float y_hat = (a * x_hat + b) * x_hat * x_hat * x_hat * x_hat
            + c * x_hat * x_hat * x_hat + d * x_hat * x_hat + e * x_hat;

        float cos_theta = cos(theta_offset);
        float sin_theta = sin(theta_offset);

        float value = x_hat * sin_theta + y_hat * cos_theta + y_offset;
        return value;
    }

    float derivativeAt(float percentage)
    {
        percentage = max(min(percentage, 1), 0);

        float x_hat = percentage * knot_distance;
        float yp_hat = (5 * a * x_hat + 4 * b) * x_hat * x_hat * x_hat + 3 * c * x_hat * x_hat
            + 2 * d * x_hat + e;

        return yp_hat;
    }

    float secondDerivativeAt(float percentage)
    {
        percentage = max(min(percentage, 1), 0);

        float x_hat = percentage * knot_distance;
        float ypp_hat = (20 * a * x_hat + 12 * b) * x_hat * x_hat + 6 * c * x_hat + 2 * d;

        return ypp_hat;
    }

    float angleAt(float percentage)
    {
        float angle = boundAngle0to2PiRadians(atan(derivativeAt(percentage)) + theta_offset);
        return angle;
    }

    float angleChangeAt(float percentage)
    {
        return boundAngleNegPiToPiRadians(atan(secondDerivativeAt(percentage)));
    }

};

//TODO: Path (with or without waypoints)

/*Start of Trajectory*/
struct segment
{
    float pos, vel, acc, jerk, heading, dt, x, y;
    segment()
    {
        pos = 0;
        vel = 5206;
        acc = 0;
        jerk = 0;
        heading = 0;
        dt = 0;
        x = 0;
        y = 0;
    }
    segment(float pos_in, float vel_in, float acc_in, float jerk_in, float heading_in, float dt_in, float x_in, float y_in)
    {
        pos = pos_in;
        vel = vel_in;
        acc = acc_in;
        jerk = jerk_in;
        heading = heading_in;
        dt = dt_in;
        x = x_in;
        y = y_in;
    }
    void yInvertScale(bool y_status)
    {
        if (!y_status)
        {
            return;
        }
        else
        {
            y *= -1.0;
            heading *= -1.0;
            return;
        }
    }
};

struct trajectory
{
    int length;
    int num_segments;
    segment segments[point_amount];
    bool inverted_y;
    trajectory()
    {
        num_segments = 0;
        inverted_y = false;
    }
    trajectory(int length_in)
    {
        num_segments = 0;
        inverted_y = false;
        length = length_in;
        for (int i = 0; i < length; ++i)
        {
            num_segments++;
            segments[i] = segment();
        }
    }

    trajectory(segment segments_in[])
    {
        num_segments = 0;
        inverted_y = false;
        for(int i = 0; i < point_amount; i++)
        {
            if(segments_in[i].vel == 5206) break;
            num_segments++;
            segments[i] = segments_in[i];
        }
    }

    segment getSegment(int index)
    {
        if (index < num_segments)
        {
            if (!inverted_y)
            {
                return segments[index];
            }
            else
            {
                segment result = segment(segments[index]);
                result.y *= -1.0;
                result.heading *= -1.0;
                return result;
            }
        }
        else
        {
            printf("Index exceeded bounds in a get segment.\n");
            assert(0);//throw 1;
        }
    }

    void setSegment(int index, segment segment_in)
    {
        if (index < num_segments) {
            segments[index] = segment_in;
        }
    }

    void scale(float scaling_factor)
    {
        for (int i = 0; i < num_segments; ++i)
        {
            segments[i].pos *= scaling_factor;
            segments[i].vel *= scaling_factor;
            segments[i].acc *= scaling_factor;
            segments[i].jerk *= scaling_factor;
        }
    }
    /*
      TODO: Actually write in c++, using way too much java stuff here.
      void append(trajectory to_append)
      {
      segment temp[num_segments + to_append.num_segments];

      for (int i = 0; i < num_segments; ++i)
      {
      temp[i] = segments[i];
      }
      for (int i = 0; i < to_append.num_segments; ++i)
      {
      temp[i + num_segments] = to_append.getSegment(i);
      }
      segments = temp;
      }
    */
};

struct pair
{
    trajectory left;
    trajectory right;
    pair(trajectory left_in, trajectory right_in)
    {
        left = left_in;
        right = right_in;
    }
    pair(){}
};

/*End of Trajectory*/
struct Config
{
    float max_vel;
    float max_acc;
    float max_jerk;
    float dt;
};

trajectory secondOrderFilter(int f1_length, int f2_length, float dt, float start_vel, float max_vel, float total_impulse, int length)
{

    if (length <= 0) assert(0);//throw 1;
    trajectory traj(length);

    segment last;
    // First segment is easy
    last.pos = 0;
    last.vel = start_vel;
    last.acc = 0;
    last.jerk = 0;
    last.dt = dt;

    // f2 is the average of the last f2_length samples from f1, so while we
    // can recursively compute f2's sum, we need to keep a buffer for f1.
    float f1[length];
    f1[0] = (start_vel / max_vel) * f1_length;
    float f2;
    for (int i = 0; i < length; ++i)
    {
        autonomousUpdate();
        
        // Apply input
        float input = min(total_impulse, 1);
        if (input < 1)
        {
            // The impulse is over, so decelerate
            input -= 1;
            total_impulse = 0;
        } else {
            total_impulse -= input;
        }

        // Filter through F1
        float f1_last;
        if (i > 0)
        {
            f1_last = f1[i - 1];
        }
        else
        {
            f1_last = f1[0];
        }
        f1[i] = max(0.0, min(f1_length, f1_last + input));

        f2 = 0;
        // Filter through F2
        for (int j = 0; j < f2_length; ++j)
        {
            if (i - j < 0) {
                break;
            }

            f2 += f1[i - j];
        }
        f2 = f2 / f1_length;

        // Velocity is the normalized sum of f2 * the max velocity
        traj.segments[i].vel = f2 / f2_length * max_vel;

        //Trapezoidal Integration
        traj.segments[i].pos = (last.vel + traj.segments[i].vel) / 2.0 * dt + last.pos;

        traj.segments[i].x = traj.segments[i].pos;
        traj.segments[i].y = 0;

        // Acceleration and jerk are the differences in velocity and
        // acceleration, respectively.
        traj.segments[i].acc = (traj.segments[i].vel - last.vel) / dt;
        traj.segments[i].jerk = (traj.segments[i].acc - last.acc) / dt;
        traj.segments[i].dt = dt;

        last = traj.segments[i];
    }
    return traj;
}

trajectory generate(Config &config,float start_vel,float start_heading, float goal_pos, float goal_vel, float goal_heading)
{
    trajectory traj;
    #if 0
    // How fast can we go given maximum acceleration and deceleration?
    float start_discount = .5 * start_vel * start_vel / config.max_acc;
    float end_discount = .5 * goal_vel * goal_vel / config.max_acc;

    float adjusted_max_vel = min(config.max_vel, sqrt(config.max_acc * goal_pos - start_discount - end_discount));
    float t_rampup = (adjusted_max_vel - start_vel) / config.max_acc;
    float x_rampup = start_vel * t_rampup + .5 * config.max_acc * t_rampup * t_rampup;
    float t_rampdown = (adjusted_max_vel - goal_vel) / config.max_acc;
    float x_rampdown = adjusted_max_vel * t_rampdown - .5 * config.max_acc * t_rampdown * t_rampdown;
    float x_cruise = goal_pos - x_rampdown - x_rampup;

    // The +.5 is to round to nearest
    int time_trajectory = (int) ((t_rampup + t_rampdown + x_cruise / adjusted_max_vel) / config.dt + .5);

    // Compute the length of the linear filters and impulse.
    int f1_length = (int) ceil((adjusted_max_vel / config.max_acc) / config.dt);
    float impulse = (goal_pos / adjusted_max_vel) / config.dt - start_vel / config.max_acc / config.dt + start_discount + end_discount;
    traj = secondOrderFilter(f1_length, 1, config.dt, start_vel, adjusted_max_vel, impulse, time_trajectory);
    #endif
    // How fast can we go given maximum acceleration and deceleration?
    float adjusted_max_vel = min(config.max_vel,(-config.max_acc * config.max_acc
                                                 + sqrt(config.max_acc * config.max_acc * config.max_acc * config.max_acc + 4 * config.max_jerk * config.max_jerk * config.max_acc * goal_pos)) / (2 * config.max_jerk));

    // Compute the length of the linear filters and impulse.
    int f1_length = (int) ceil((adjusted_max_vel / config.max_acc) / config.dt);
    int f2_length = (int) ceil((config.max_acc / config.max_jerk) / config.dt);
    float impulse = (goal_pos / adjusted_max_vel) / config.dt;
    int time_trajectory = (int) (ceil(f1_length + f2_length + impulse));
    traj = secondOrderFilter(f1_length, f2_length, config.dt, 0, adjusted_max_vel, impulse, time_trajectory);


    // Now assign headings by interpolating along the path.
    // Don't do any wrapping because we don't know units.
    float total_heading_change = goal_heading - start_heading;
    for (int i = 0; i < traj.num_segments; ++i)
    {
        segment seg = traj.getSegment(i);
        traj.segments[i].heading = start_heading + total_heading_change * (traj.segments[i].pos) / traj.segments[traj.num_segments - 1].pos;
    }

    return traj;
}



struct path
{
    pair go_left_pair;
    bool go_left;

    path(pair go_left_pair_in)
    {
        go_left_pair = go_left_pair_in;
        go_left = true;
    }
    void goLeft()
    {
        go_left = true;
        go_left_pair.left.inverted_y = false;
        go_left_pair.right.inverted_y = false;
    }

    void goRight()
    {
        go_left = false;
        go_left_pair.left.inverted_y = true;
        go_left_pair.right.inverted_y = true;
    }

    trajectory getLeftWheelTrajectory() {
        return (go_left ? go_left_pair.left : go_left_pair.right);
    }

    trajectory getRightWheelTrajectory() {
        return (go_left ? go_left_pair.right : go_left_pair.left);
    }

    pair getPair()
    {
        return go_left_pair;
    }

    float getEndHeading() {
        int numSegments = getLeftWheelTrajectory().num_segments;
        segment lastSegment = getLeftWheelTrajectory().getSegment(numSegments - 1);
        return lastSegment.heading;
    }
};

trajectory generateFromPath(waypointSequence &path, Config &config)
{

    if (path.current_amount < 2) assert(0);//throw 1;
    // Compute the total length of the path by creating splines for each pair
    // of waypoints.
    quinticSpline splines[point_amount];//Actual length is path.current_amount-1
    float spline_lengths[point_amount];
    float total_distance = 0;
    for (int i = 0; i < path.current_amount-1; ++i) {
        if(path.getWaypoint(i).x  == -1) break;
        splines[i].reticulateSplines(path.getWaypoint(i), path.getWaypoint(i + 1));
        spline_lengths[i] = splines[i].calculateLength();
        total_distance += spline_lengths[i];
    }

    // Generate a smooth trajectory over the total distance.
    trajectory traj = generate(config, 0.0, path.getWaypoint(0).theta, total_distance, 0.0, path.getWaypoint(0).theta);
    trajectory debug = traj;
    // Assign headings based on the splines.
    int cur_spline = 0;
    float cur_spline_start_pos = 0;
    float length_of_splines_finished = 0;
    for (int i = 0; i < traj.num_segments; ++i)
    {
        //if(traj.segments[i].vel == 5206) break;
        //traj.segments[i].yInvertScale(traj.inverted_y);
        float cur_pos = traj.segments[i].pos;

        bool found_spline = false;
        while (!found_spline) {
            float cur_pos_relative = cur_pos - cur_spline_start_pos;
            if (cur_pos_relative <= spline_lengths[cur_spline])
            {
                float percentage = splines[cur_spline].getPercentageForDistance(cur_pos_relative);
                float head_temp = splines[cur_spline].angleAt(percentage);
                traj.segments[i].heading = head_temp;
                v2f coords = splines[cur_spline].getXandY(percentage);
                traj.segments[i].x = coords[0];
                traj.segments[i].y = coords[1];
                found_spline = true;
            }
            else if (cur_spline < path.current_amount)
            {
                length_of_splines_finished += spline_lengths[cur_spline];
                cur_spline_start_pos = length_of_splines_finished;
                ++cur_spline;
            }
            else
            {
                traj.segments[i].heading = splines[path.current_amount].angleAt(1.0);
                v2f coords = splines[path.current_amount].getXandY(1.0);
                traj.segments[i].x = coords[0];
                traj.segments[i].y = coords[1];
                found_spline = true;
            }
        }
    }

    return traj;
}

/**
 * Generate left and right wheel trajectories from a reference.
 *
 * @param input The reference trajectory.
 * @param wheelbase_width The center-to-center distance between the left and
 * right sides.
 * @return [0] is left, [1] is right
 */
pair makeLeftAndRightTrajectories(trajectory &input, float wheelbase_width)
{
    trajectory left = input;
    trajectory right = input;

    for (int i = 0; i < input.num_segments; ++i)
    {
        //input.segments[i].yInvertScale(input.inverted_y);
        segment current = input.getSegment(i);
        //current.yInvertScale(input.inverted_y);
        float cos_angle = cos(current.heading);
        float sin_angle = sin(current.heading);

        segment s_left = left.getSegment(i);
        s_left.x = current.x - wheelbase_width / 2 * sin_angle;
        s_left.y = current.y + wheelbase_width / 2 * cos_angle;
        if (i > 0)
        {
            // Get distance between current and last segment
            float dist = sqrt((s_left.x - left.getSegment(i - 1).x)
                              * (s_left.x - left.getSegment(i - 1).x)
                              + (s_left.y - left.getSegment(i - 1).y)
                              * (s_left.y - left.getSegment(i - 1).y));
            s_left.pos = left.getSegment(i - 1).pos + dist;
            s_left.vel = dist / s_left.dt;
            s_left.acc = (s_left.vel - left.getSegment(i - 1).vel) / s_left.dt;
            s_left.jerk = (s_left.acc - left.getSegment(i - 1).acc) / s_left.dt;
        }
        left.segments[i] = s_left;
        segment s_right = right.getSegment(i);
        s_right.x = current.x + wheelbase_width / 2 * sin_angle;
        s_right.y = current.y - wheelbase_width / 2 * cos_angle;
        if (i > 0) {
            // Get distance between current and last segment
            float dist = sqrt((s_right.x - right.getSegment(i - 1).x)
                              * (s_right.x - right.getSegment(i - 1).x)
                              + (s_right.y - right.getSegment(i - 1).y)
                              * (s_right.y - right.getSegment(i - 1).y));
            s_right.pos = right.getSegment(i - 1).pos + dist;
            s_right.vel = dist / s_right.dt;
            s_right.acc = (s_right.vel - right.getSegment(i - 1).vel) / s_right.dt;
            s_right.jerk = (s_right.acc - right.getSegment(i - 1).acc) / s_right.dt;
        }
        right.segments[i] = s_right;
    }

    return pair(left, right);
}

pair generateLeftAndRightFromSeq(waypointSequence &path, Config &config, float wheelbase_width)
{
    printf("Generate Left Right from Seq: %f\n", config.max_vel);
    trajectory tempTraj = generateFromPath(path, config);
    return makeLeftAndRightTrajectories(tempTraj, wheelbase_width);
}

path makePath(waypointSequence &waypoints, Config &config, float wheelbase_width)
{
    printf("Make Path: %f\n", config.max_vel);
    path p_debug = generateLeftAndRightFromSeq(waypoints, config, wheelbase_width);
    path p = p_debug;
    return p_debug;
}


struct TrajectoryFollower
{
    float kp;
    float ki;  // Not currently used, but might be in the future.
    float kd;
    float kv;
    float ka;
    float last_error;
    float current_heading;
    int current_segment;
    trajectory profile;

    TrajectoryFollower()
    {
        current_segment = 1;
    }

    void configure(float kp_in, float ki_in, float kd_in, float kv_in, float ka_in)
    {
        kp = kp_in;
        ki = ki_in;
        kd = kd_in;
        kv = kv_in;
        ka = ka_in;
    }

    void reset()
    {
        last_error = 0.0;
        current_segment = 0;
    }

    void setTrajectory(trajectory &profile_in)
    {
        profile = profile_in;
    }

    float calculate(float distance_so_far)
    {
        if (current_segment < profile.num_segments)
        {
            segment seg = profile.getSegment(current_segment);
            float error = seg.pos - distance_so_far;
            float output = kp * error + kd * ((error - last_error)
                                              / seg.dt - seg.vel) + (kv * seg.vel
                                                                     + ka * seg.acc);

            last_error = error;
            current_heading = seg.heading;
            current_segment++;
            //System.out.println("so far: " + distance_so_far + "; output: " + output);
            return output;
        }
        else
        {
            return -5206;
        }
    }

    v4f positionCalc(float distance_so_far)
    {
        if (current_segment < profile.num_segments)
        {
            segment seg = profile.getSegment(current_segment);
            float error = seg.pos - distance_so_far;
            float output = kp * error + kd * ((error - last_error)
                                              / seg.dt - seg.vel) + (kv * seg.vel + ka * seg.acc);

            last_error = error;
            current_heading = seg.heading;
            current_segment++;
            //System.out.println("so far: " + distance_so_far + "; output: " + output);
            return (v4f) {seg.x, seg.y, current_heading, seg.pos};
        }
        else
        {
            return (v4f) {0,0,0,0};
        }
    }

    float getHeading()
    {
        return current_heading;
    }

    bool isFinishedTrajectory()
    {
        return current_segment >= profile.num_segments;
    }
};
#endif
