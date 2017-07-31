requestAnimationFrame = function(callback) {
    window.setTimeout(callback, 1000 / 24);
};

function run()
{
    var _canvas = document.getElementById('canvas');
    var _ctx = document.getElementById('canvas').getContext('2d');

    // Globals
    var _line_scale_x = (canvas.width/canvas.height)*12.0;
    var _line_scale_y = 12.0;
    var _t_begin = Date.now() / 1000.0
    var _last_t = 0;
    var simulation_time = 0;
    var random_index = 0;

    // Constants
    var Pi = 3.14159265359;
    var Two_Pi = 6.28318530718;
    var Speed_Multiplier = 1;
    var Num_Targets = 10;
    var Num_Obstacles=  4;
    var Num_Robots = Num_Targets + Num_Obstacles;

    // Constants in sim_robot
    var Random_Min = 0;
    var Random_Max = 64;
    var Meters = 1.0;
    var Millimeters = 0.001;
    var Seconds = 1.0;
    var Robot_Speed = (330.0 * Millimeters / Seconds);

    // Time between trajectory noise injections
    var Noise_Interval = (5.0 * Seconds);

    // Time between auto-reverse
    var Reverse_Interval = (20.0 * Seconds);

    // Time needed to affect trajectory
    var Noise_Length = (0.850 * Seconds);

    // Time needed to reverse trajectory
    // (.33/2 m/s * pi * wheelbase / 2)
    var Reverse_Length = (2.456 * Seconds);

    // Time needed to spin 45 degrees
    var Top_Touch_Time = (Reverse_Length / 4.0);

    // enum robot_State
    var Robot_Start = 0;
    var Robot_ObstacleWait = 1;
    var Robot_ObstacleRun = 2;
    var Robot_ObstacleCollision = 3;
    var Robot_TargetWait = 4;
    var Robot_TargetRun = 5;
    var Robot_TrajectoryNoise = 6;
    var Robot_Reverse = 7;
    var Robot_TargetCollision = 8;
    var Robot_TopTouch = 9;

    // enum sim_CommandType
    var sim_CommandType_LandOnTopOf = 0; // trigger one 45 deg turn of robot (i)
    var sim_CommandType_LandInFrontOf = 1; // trigger one 180 deg turn of robot (i)
    var sim_CommandType_Track = 2; // follow robot (i) at a constant height
    var sim_CommandType_Search = 3; // ascend to 3 meters and go to (x, y)

    // key state
    var key_land_down = false;
    var key_land_down = false;
    var key_tap_down = false;
    var key_left_down = false;
    var key_down_down = false;
    var key_right_down = false;
    var key_up_down = false;

    // key mappings
    var key_land = 90;
    var key_tap = 88;
    var key_left = 37;
    var key_down = 40;
    var key_right = 39;
    var key_up = 38;

    var drone = Drone();
    var targets = [];
    var obstacles = [];

    _ctx.lineWidth = 2;

    document.addEventListener("keyup",on_key_up, false);
    document.addEventListener("keydown",on_key_down, false);

    sim_init();
    animation_loop();

    function on_key_up(event)
    {
        switch (event.keyCode)
        {
            case key_land:
                key_land_down = false;
                break;
            case key_tap:
                key_tap_down = false;
                break;
            case key_left:
                key_left_down = false;
                break;
            case key_down:
                key_down_down = false;
                break;
            case key_right:
                key_right_down = false;
                break;
            case key_up:
                key_up_down = false;
                break;
        }
    }

    function on_key_down(event)
    {
        switch (event.keyCode)
        {
            case key_land:
                key_land_down = true;
                break;
            case key_tap:
                key_tap_down = true;
                break;
            case key_left:
                key_left_down = true;
                break;
            case key_down:
                key_down_down = true;
                break;
            case key_right:
                key_right_down = true;
                break;
            case key_up:
                key_up_down = true;
                break;
        }
    }

    function Robot()
    {
        return {
        state: Robot_Start,
        internal: robot_Internal(),
        action: robot_Action(),
        x: 0,
        y: 0,
        L: 0,
        vl: 0,
        vr: 0,
        q: 0,
        height: 0,
        tangent_x: 0,
        tangent_y: 0,
        forward_x: 0,
        forward_y: 0,
        removed: false };
    }

    function sim_Command()
    {
        return { type: sim_CommandType_Search,
        x: 0,
        y: 0,
        i: 0 };
    }

    function Drone()
    {
        return {
        zeta: 0,
        w: 0,
        a: 0,
        x1: 0,
        x2: 0,
        y1: 0,
        y2: 0,
        z1: 0,
        z2: 0,
        x: 0,
        y: 0,
        z: 0,
        xr: 0,
        yr: 0,
        zr: 0,
        v_max: 0,
        cmd: sim_Command(),
        cmd_complete: false };
    }

    function robot_Internal()
    {
        return { begin_noise: 0,
        begin_reverse: 0,
        begin_top_touch: 0,
        last_noise: 0,
        last_reverse: 0,
        initialized: false };
    }

    function robot_Event()
    {
        return {is_run_sig: false,
        is_wait_sig: false,
        is_top_touch: false,
        is_bumper: false,
        target_switch_pin: false,
        elapsed_sim_time: 0};
    }

    function robot_Action()
    {
        return { left_wheel: 0,
        right_wheel: 0,
        red_led: 0,
        green_led: 0,
        was_top_touched: 0,
        was_bumped: 0 };
    }

    function animation_loop()
    {
        t = (Date.now()/1000.0) - _t_begin;
        dt = t - _last_t;
        _last_t = t;
        _ctx.clearRect(0, 0, 500, 500);
        _ctx.fillStyle = "#0a0a0a";
        _ctx.fillRect(0,0,_canvas.width,_canvas.height);
        sim_tick(t, dt);
        requestAnimationFrame(animation_loop);
    }

    function set_color(r, g, b, a)
    {
        _ctx.strokeStyle = "rgba("+Math.floor(r*255)+","+Math.floor(g*255)+","+Math.floor(b*255)+","+Math.floor(a*255)+")";
        _ctx.globalAlpha = a;
    }

    function draw_line(x1, y1, x2, y2)
    {
         _ctx.beginPath();
         var xs1 = (0.5+0.5*(x1 / _line_scale_x))*_canvas.width;
         var ys1 = (0.5+0.5*(y1 / _line_scale_y))*_canvas.height;
         var xs2 = (0.5+0.5*(x2 / _line_scale_x))*_canvas.width;
         var ys2 = (0.5+0.5*(y2 / _line_scale_y))*_canvas.height;
         _ctx.moveTo(xs1, ys1);
         _ctx.lineTo(xs2, ys2);
         _ctx.stroke();
    }

    function draw_circle(x, y, r)
    {
        var n = 32;
        for (var i = 0; i < n; i++)
        {
            var t1 = Two_Pi * i / n;
            var t2 = Two_Pi * (i + 1) / n;
            var x1 = x + r*cos(t1);
            var y1 = y + r*sin(t1);
            var x2 = x + r*cos(t2);
            var y2 = y + r*sin(t2);

            draw_line(x1, y1, x2, y2);
        }
    }

    function clamp(x, x0, x1)
    {
        if (x < x0) return x0;
        else if (x > x1) return x1;
        else return x;
    }

    function drone_integrate(drone, dt)
    {
        var x1y1_max = drone.v_max / (drone.w*drone.w);
        var a11 = -2.0 * drone.zeta * drone.w;
        var a12 = -drone.w*drone.w;
        var a21 = 1.0;
        var a22 = 0.0;
        var c1 = drone.a * drone.w * drone.w;
        var c2 = drone.w * drone.w;
        var Dx1 = a11 * drone.x1 + a12 * drone.x2 + drone.xr;
        var Dx2 = a21 * drone.x1 + a22 * drone.x2;
        drone.x1 += Dx1 * dt;
        drone.x2 += Dx2 * dt;
        drone.x1 = clamp(drone.x1, -x1y1_max, x1y1_max);
        drone.x = c1 * drone.x1 + c2 * drone.x2;

        // The dynamics from reference y to y is the same as above.
        var Dy1 = a11 * drone.y1 + a12 * drone.y2 + drone.yr;
        var Dy2 = a21 * drone.y1 + a22 * drone.y2;
        drone.y1 += Dy1 * dt;
        drone.y2 += Dy2 * dt;
        drone.y1 = clamp(drone.y1, -x1y1_max, x1y1_max);
        drone.y = c1 * drone.y1 + c2 * drone.y2;

        // And from reference height to height
        var Dz1 = a11 * drone.z1 + a12 * drone.z2 + drone.zr;
        var Dz2 = a21 * drone.z1 + a22 * drone.z2;
        drone.z1 += Dz1 * dt;
        drone.z2 += Dz2 * dt;
        drone.z = c1 * drone.z1 + c2 * drone.z2;
    }

    function cos(x)
    {
        return Math.cos(x);
    }

    function sin(x)
    {
        return Math.sin(x);
    }

    function robot_integrate(robot, dt)
    {
        var v = 0.5 * (robot.vl + robot.vr);
        var w = (robot.vr - robot.vl) / (robot.L*0.5);
        if (Math.abs(w) < 0.001)
        {
            robot.x += -v*sin(robot.q)*dt;
            robot.y +=  v*cos(robot.q)*dt;
        }
        else
        {
            robot.x += (v / w) * (cos(robot.q + w*dt) - cos(robot.q));
            robot.y += (v / w) * (sin(robot.q + w*dt) - sin(robot.q));
        }
        robot.q += w * dt;

        while (robot.q >= Two_Pi)
            robot.q -= Two_Pi;

        robot.tangent_x = cos(robot.q);
        robot.tangent_y = sin(robot.q);
        robot.forward_x = -robot.tangent_y;
        robot.forward_y =  robot.tangent_x;
    }

    function vector_length(dx, dy)
    {
        return Math.sqrt(dx*dx + dy*dy);
    }

    function CollisionInfo()
    {
        return {hits: 0, bumper_hits: 0, resolve_delta_x: 0, resolve_delta_y: 0};
    }

    function advance_state(dt)
    {
        simulation_time += dt;

        var events = [];
        var collision = [];

        drone_integrate(drone, dt);

        for (var i = 0; i < Num_Robots; i++)
        {
            var robot;
            if (i < Num_Targets)
                robot = targets[i];
            else
                robot = obstacles[i - Num_Targets];
            if (robot.removed) continue;

            events[i] = robot_Event();
            events[i].is_run_sig = 0;
            events[i].is_wait_sig = 0;
            events[i].is_top_touch = 0;
            events[i].is_bumper = 0;
            events[i].target_switch_pin = 0;
            events[i].elapsed_sim_time = simulation_time;

            collision[i] = CollisionInfo();
            collision[i].hits = 0;
            collision[i].bumper_hits = 0;
            collision[i].resolve_delta_x = 0.0;
            collision[i].resolve_delta_y = 0.0;

            switch (robot.state)
            {
                case Robot_Start:
                {
                    if (i < Num_Targets)
                        events[i].target_switch_pin = true;
                    else
                        events[i].target_switch_pin = false;
                } break;

                case Robot_TargetWait:
                {
                    events[i].is_run_sig = true;
                };

                case Robot_ObstacleWait:
                {
                    events[i].is_run_sig = true;
                } break;
            }

            // Check for collisions and compute the average resolve
            // delta vector. The resolve delta will be used to move
            // the robot away so it no longer collides.
            for (var n = 0; n < Num_Robots; n++)
            {
                var other;
                if (n < Num_Targets)
                    other = targets[n];
                else
                    other = obstacles[n - Num_Targets];
                if (i == n || other.removed)
                {
                    continue;
                }
                else
                {
                    var x1 = robot.x;
                    var y1 = robot.y;
                    var r1 = robot.L * 0.5;
                    var x2 = other.x;
                    var y2 = other.y;
                    var r2 = other.L * 0.5;
                    var dx = x1 - x2;
                    var dy = y1 - y2;
                    var L = vector_length(dx, dy);
                    var intersection = r2 + r1 - L;
                    if (intersection > 0.0)
                    {
                        collision[i].hits++;
                        collision[i].resolve_delta_x += (dx / L) * intersection;
                        collision[i].resolve_delta_y += (dy / L) * intersection;

                        // The robot only reacts (in a fsm sense) if the collision
                        // triggers the bumper sensor in front of the robot. (We
                        // still resolve physical collisions anyway, though).
                        // TODO: Determine the angular region that the bumper
                        // sensor covers (I have assumed 180 degrees).
                        var on_bumper = (dx * robot.forward_x +
                                          dy * robot.forward_y) <= 0.0;
                        if (on_bumper)
                            collision[i].bumper_hits++;
                    }
                }
            }
            if (collision[i].hits > 0)
            {
                collision[i].resolve_delta_x /= collision[i].hits;
                collision[i].resolve_delta_y /= collision[i].hits;
            }
            if (collision[i].bumper_hits > 0)
                events[i].is_bumper = 1;
        }

        switch (drone.cmd.type)
        {
            case sim_CommandType_LandOnTopOf:
            {
                if (!drone.cmd_complete)
                {
                    drone.xr = targets[drone.cmd.i].x;
                    drone.yr = targets[drone.cmd.i].y;
                    if (vector_length(drone.xr - drone.x, drone.yr - drone.y)
                        <= targets[drone.cmd.i].L)
                    {
                        drone.zr = 0.125;
                    }
                    if (drone.z < 0.2 && drone.zr < 0.2)
                    {
                        events[drone.cmd.i].is_top_touch = 1;
                    }
                    if (targets[drone.cmd.i].action.was_top_touched)
                    {
                        drone.zr = 1.5;
                        drone.cmd_complete = 1;
                    }
                }
            } break;
            case sim_CommandType_LandInFrontOf:
            {
                if (!drone.cmd_complete)
                {
                    drone.xr = targets[drone.cmd.i].x;
                    drone.yr = targets[drone.cmd.i].y;
                    if (vector_length(drone.xr - drone.x, drone.yr - drone.y)
                        <= targets[drone.cmd.i].L)
                    {
                        drone.zr = 0.05;
                    }
                    if (drone.z < 0.1 && drone.zr < 0.1)
                    {
                        events[drone.cmd.i].is_bumper = 1;
                    }
                    if (targets[drone.cmd.i].action.was_bumped)
                    {
                        drone.zr = 1.5;
                        drone.cmd_complete = 1;
                    }
                }
            } break;
            case sim_CommandType_Track:
            {
                // Follow a given robot
                drone.xr = targets[drone.cmd.i].x;
                drone.yr = targets[drone.cmd.i].y;
                drone.zr = 1.5;
            } break;
            case sim_CommandType_Search:
            {
                // Go to a setpoint and hover high
                drone.xr = drone.cmd.x;
                drone.yr = drone.cmd.y;
                drone.zr = 3.0;
            } break;
        }

        for (var i = 0; i < Num_Robots; i++)
        {
            var robot;
            if (i < Num_Targets)
            {
                robot = targets[i];
            }
            else
            {
                robot = obstacles[i-Num_Targets];
            }
            if (robot.removed)
                continue;
            // robot.vl = 1;
            // robot.vr = 0.8;
            robot_integrate(robot, dt);
            if (robot.x < -10.5 ||
                robot.x > +10.5 ||
                robot.y < -10.5 ||
                robot.y > +10.5)
            {
                robot.removed = true;
            }
            if (collision[i].hits > 0)
            {
                robot.x += collision[i].resolve_delta_x * 1.02;
                robot.y += collision[i].resolve_delta_y * 1.02;
            }
            robot.state = robot_fsm(robot.state,
                                    robot.internal,
                                    events[i],
                                    robot.action);
            robot.vl = robot.action.left_wheel;
            robot.vr = robot.action.right_wheel;
        }
    }

    function draw_robot(r)
    {
        if (r.removed) return;
        draw_circle(r.x, r.y, r.L * 0.5);
        draw_line(r.x - r.tangent_x * r.L * 0.5,
                  r.y - r.tangent_y * r.L * 0.5,
                  r.x + r.tangent_x * r.L * 0.5,
                  r.y + r.tangent_y * r.L * 0.5);
        draw_line(r.x, r.y,
                  r.x + r.forward_x * r.L * 1.3,
                  r.y + r.forward_y * r.L * 1.3);
    }

    function sim_init()
    {
        for (var i = 0; i < Num_Targets; i++)
        {
            var robot = Robot();

            robot.L = 0.5;
            robot.height = 0.1;

            // Spawn each ground robot in a circle
            t = Two_Pi * i / (Num_Targets);
            robot.x = -1.0 * sin(t);
            robot.y = 1.0 * cos(t);
            robot.q = t;
            robot.internal.initialized = false;
            robot.state = Robot_Start;
            robot.removed = false;

            targets[i] = robot;
        }

        for (var i = 0; i < Num_Obstacles; i++)
        {
            var t = Two_Pi * i / (Num_Obstacles);

            var robot = Robot();

            robot.L = 0.5;
            robot.height = 2.0;

            robot.x = -5.0 * sin(t);
            robot.y = 5.0 * cos(t);
            robot.q = t + Pi / 2.0;
            robot.internal.initialized = false;
            robot.state = Robot_Start;
            robot.removed = false;

            obstacles[i] = robot;
        }

        drone.zeta = 0.9;
        drone.w = 3.5;
        drone.a = -0.03;
        drone.x = 0.0;
        drone.x1 = 0.0;
        drone.x2 = 0.0;
        drone.y = 0.0;
        drone.y1 = 0.0;
        drone.y2 = 0.0;
        drone.xr = 0.0;
        drone.yr = 0.0;
        drone.z = 1.0;
        drone.z1 = 0.0;
        drone.z2 = 0.0;
        drone.zr = 1.5;
        drone.v_max = 2.0;
        drone.cmd.type = sim_CommandType_Search;
        drone.cmd.x = 0.0;
        drone.cmd.y = 0.0;
        drone.cmd.i = 0;
        drone.cmd_complete = 0;
    }

    function sim_tick(t, dt)
    {
        var closest_target = 0;
        var closest_target_d = 100.0;
        for (var i = 0; i < Num_Targets; i++)
        {
            var target_x = targets[i].x;
            var target_y = targets[i].y;
            var d = vector_length(target_x - drone.xr, target_y - drone.yr);
            if (d <= closest_target_d)
            {
                closest_target_d = d;
                closest_target = i;
            }
        }
        if (key_left_down)
        {
            drone.cmd.type = sim_CommandType_Search;
            drone.cmd.x -= 5.0 * dt;
        }
        if (key_right_down)
        {
            drone.cmd.type = sim_CommandType_Search;
            drone.cmd.x += 5.0 * dt;
        }
        if (key_up_down)
        {
            drone.cmd.type = sim_CommandType_Search;
            drone.cmd.y -= 5.0 * dt;
        }
        if (key_down_down)
        {
            drone.cmd.type = sim_CommandType_Search;
            drone.cmd.y += 5.0 * dt;
        }
        if (key_land_down)
        {
            drone.cmd.type = sim_CommandType_LandInFrontOf;
            drone.cmd.i = closest_target;
            drone.cmd_complete = 0;
        }
        if (key_tap_down)
        {
            drone.cmd.type = sim_CommandType_LandOnTopOf;
            drone.cmd.i = closest_target;
            drone.cmd_complete = 0;
        }
        if (drone.cmd.type == sim_CommandType_LandInFrontOf ||
            drone.cmd.type == sim_CommandType_LandOnTopOf)
        {
            // Need to synchronize these when keyboard input is used,
            // otherwise we get a jarring jump in the drawn circle
            // when you use keyboard after performing a land command.
            drone.cmd.x = drone.xr;
            drone.cmd.y = drone.yr;
        }

        for (var i = 0; i < Speed_Multiplier; i++)
        {
            advance_state(dt);
        }

        // draw grid
        set_color(0.34, 0.4, 0.49, 0.45);
        for (var i = 0; i <= 20; i++)
        {
            x = (-1.0 + 2.0 * i / 20.0) * 10.0;
            draw_line(x, -10.0, x, +10.0);
            draw_line(-10.0, x, +10.0, x);
        }

        // draw green line
        set_color(0.1, 1.0, 0.3, 0.45);
        draw_line(10.0, -10.0, 10.0, 10.0);

        // draw targets
        set_color(0.75, 0.2, 0.26, 1.0);
        for (var i = 0; i < Num_Targets; i++)
            draw_robot(targets[i]);

        // draw obstacles
        set_color(0.71, 0.7, 0.07, 1.0);
        for (var i = 0; i < Num_Obstacles; i++)
            draw_robot(obstacles[i]);

        // draw drone
        set_color(0.33, 0.55, 0.53, 1.0);
        draw_line(drone.x - 0.5, drone.y,
                  drone.x + 0.5, drone.y);
        draw_line(drone.x, drone.y - 0.5,
                  drone.x, drone.y + 0.5);

        // draw drone goto
        set_color(0.2, 0.5, 1.0, 0.5);
        draw_circle(drone.xr, drone.yr, 0.45);

        // draw indicators of magnet or bumper activations
        for (var i = 0; i < Num_Targets; i++)
        {
            var x = targets[i].x;
            var y = targets[i].y;
            if (targets[i].action.was_bumped)
            {
                set_color(1.0, 0.3, 0.1, 1.0);
                draw_circle(x, y, 0.5);
            }
            else if (targets[i].action.was_top_touched)
            {
                set_color(1.0, 1.0, 1.0, 1.0);
                draw_circle(x, y, 0.5);
            }
        }

    }

    function Random_0_64()
    {
        var random_numbers = [
            0x39,  0x14,  0x28,  0x3c,  0x3c,  0x27,  0x01,  0x25,
            0x0,  0x3,  0x17,  0x07,  0x2e,  0x37,  0x3a,  0x0c,
            0x1b,  0x08,  0x1a,  0x0c,  0x26,  0x11,  0x00,  0x1b,
            0x0a,  0x0b,  0x3a,  0x3c,  0x30,  0x19,  0x16,  0x18,
            0x0e,  0x26,  0x14,  0x10,  0x12,  0x37,  0x3d,  0x38,
            0x3e,  0x0,  0x08,  0x39,  0x29,  0x3,  0x0,  0x10,
            0x09,  0x26,  0x40,  0x0e,  0x01,  0x04,  0x19,  0x14,
            0x1b,  0x11,  0x14,  0x1d,  0x29,  0x40,  0x06,  0x39,
            0x37,  0x34,  0x1b,  0x26,  0x18,  0x33,  0x35,  0x3a,
            0x36,  0x2e,  0x31,  0x40,  0x3,  0x28,  0x37,  0x07,
            0x34,  0x24,  0x31,  0x30,  0x1c,  0x39,  0x2c,  0x23,
            0x1c,  0x28,  0x22,  0x34,  0x22,  0x3c,  0x09,  0x34,
            0x10,  0x26,  0x10,  0x29,  0x29,  0x00,  0x2d,  0x26,
            0x40,  0x22,  0x40,  0x3c,  0x2c,  0x21,  0x40,  0x15,
            0x09,  0x0d,  0x17,  0x32,  0x34,  0x1a,  0x07,  0x32,
            0x01,  0x2b,  0x30,  0x0,  0x18,  0x12,  0x10,  0x33,
            0x18,  0x05,  0x31,  0x34,  0x1c,  0x2e,  0x09,  0x35,
            0x00,  0x10,  0x10,  0x11,  0x0b,  0x10,  0x15,  0x03,
            0x09,  0x1b,  0x09,  0x0c,  0x2a,  0x16,  0x2b,  0x35,
            0x1,  0x23,  0x15,  0x19,  0x05,  0x10,  0x0a,  0x28,
            0x33,  0x1,  0x27,  0x19,  0x35,  0x3,  0x12,  0x0a,
            0x3b,  0x23,  0x06,  0x1c,  0x19,  0x19,  0x05,  0x05,
            0x1e,  0x02,  0x2c,  0x24,  0x12,  0x1b,  0x2c,  0x1c,
            0x2c,  0x3e,  0x37,  0x20,  0x00,  0x10,  0x02,  0x17,
            0x32,  0x0,  0x21,  0x38,  0x12,  0x38,  0x2c,  0x36,
            0x2c,  0x0c,  0x14,  0x2a,  0x19,  0x0b,  0x3e,  0x1b,
            0x16,  0x37,  0x15,  0x0e,  0x29,  0x24,  0x0a,  0x2a,
            0x37,  0x07,  0x0a,  0x0a,  0x3d,  0x40,  0x31,  0x0,
            0x01,  0x3b,  0x12,  0x28,  0x01,  0x25,  0x1e,  0x2d,
            0x38,  0x38,  0x3a,  0x3b,  0x13,  0x2d,  0x07,  0x07,
            0x14,  0x28,  0x33,  0x3d,  0x24,  0x3c,  0x31,  0x31,
            0x05,  0x30,  0x40,  0x07,  0x14,  0x12,  0x30,  0x12,
            0x1c,  0x03,  0x05,  0x02,  0x0a,  0x15,  0x3b,  0x38,
            0x40,  0x2b,  0x0d,  0x3b,  0x1d,  0x00,  0x08,  0x0b,
            0x1c,  0x12,  0x24,  0x04,  0x04,  0x30,  0x14,  0x3d,
            0x20,  0x15,  0x3,  0x20,  0x0e,  0x15,  0x1c,  0x14,
            0x28,  0x14,  0x20,  0x0c,  0x17,  0x3b,  0x16,  0x32,
            0x14,  0x3d,  0x06,  0x22,  0x36,  0x3c,  0x11,  0x14,
            0x0,  0x29,  0x14,  0x09,  0x3a,  0x0a,  0x18,  0x1b,
            0x24,  0x30,  0x3,  0x2e,  0x07,  0x1a,  0x16,  0x25,
            0x0b,  0x0c,  0x12,  0x10,  0x06,  0x0c,  0x16,  0x29,
            0x27,  0x0b,  0x24,  0x03,  0x18,  0x33,  0x2,  0x03,
            0x06,  0x32,  0x19,  0x32,  0x3c,  0x0e,  0x0,  0x0c,
            0x15,  0x2a,  0x17,  0x1a,  0x07,  0x01,  0x15,  0x1c,
            0x38,  0x3a,  0x37,  0x09,  0x24,  0x24,  0x00,  0x1b,
            0x32,  0x26,  0x14,  0x2e,  0x22,  0x35,  0x38,  0x1d,
            0x07,  0x1b,  0x02,  0x00,  0x32,  0x0,  0x33,  0x38,
            0x36,  0x12,  0x26,  0x17,  0x0c,  0x14,  0x38,  0x3a,
            0x29,  0x03,  0x12,  0x1b,  0x24,  0x1a,  0x3c,  0x3c,
            0x10,  0x3e,  0x13,  0x3,  0x0d,  0x0b,  0x13,  0x0a,
            0x40,  0x0a,  0x2d,  0x20,  0x18,  0x28,  0x05,  0x3a,
            0x31,  0x3b,  0x40,  0x31,  0x02,  0x0a,  0x15,  0x39,
            0x29,  0x08,  0x2e,  0x1a,  0x04,  0x05,  0x12,  0x3c,
            0x1e,  0x2,  0x2,  0x26,  0x11,  0x2e,  0x31,  0x1b,
            0x1c,  0x0a,  0x37,  0x00,  0x40,  0x05,  0x21,  0x2c,
            0x11,  0x26,  0x1c,  0x07,  0x3e,  0x3,  0x06,  0x2,
            0x36,  0x2d,  0x3d,  0x1c,  0x14,  0x36,  0x39,  0x21,
            0x3b,  0x2e,  0x20,  0x1b,  0x36,  0x2d,  0x0d,  0x1,
            0x00,  0x32,  0x15,  0x16,  0x18,  0x21,  0x0,  0x34,
            0x3e,  0x0,  0x2c,  0x02,  0x0e,  0x01,  0x1e,  0x23,
            0x31,  0x09,  0x33,  0x28,  0x1,  0x07,  0x0d,  0x3a,
            0x38,  0x14,  0x2e,  0x2e,  0x12,  0x16,  0x03,  0x38,
            0x3d,  0x39,  0x37,  0x0,  0x15,  0x1e,  0x01,  0x00,
            0x01,  0x08,  0x17,  0x09,  0x3a,  0x05,  0x25,  0x36
        ];

        result = random_numbers[random_index];
        random_index = (random_index + 1) % (random_numbers.length);
        return result;
    }

    function ObstacleWaitStart(event, internal, action)
    {
        action.red_led = 1;
    }

    function ObstacleRunStart(event, internal, action)
    {
        action.red_led = 1;
        action.green_led = 0;
        action.left_wheel = Robot_Speed - 9 * Millimeters / Seconds;
        action.right_wheel = Robot_Speed + 9 * Millimeters / Seconds;
    }

    function ObstacleCollisionStart(event, internal, action)
    {
        action.left_wheel = 0.0;
        action.right_wheel = 0.0;
        action.red_led = 1;
        action.green_led = 1;
    }

    function TargetWaitStart(event, internal, action)
    {
        action.green_led = 1;
    }

    function TargetRunStart(event, internal, action)
    {
        action.left_wheel = Robot_Speed;
        action.right_wheel = Robot_Speed;
        action.green_led = 1;
        action.red_led = 0;
    }

    function TrajectoryNoiseStart(event, internal, action)
    {
        offset = Random_0_64() - Random_Max / 2;
        offset_mps = offset * Millimeters / Seconds;
        action.left_wheel = Robot_Speed - offset_mps;
        action.right_wheel = Robot_Speed + offset_mps;
        action.red_led = 1;
        internal.begin_noise = event.elapsed_sim_time;
    }

    function ReverseStart(event, internal, action)
    {
        action.left_wheel = -Robot_Speed / 2.0;
        action.right_wheel = Robot_Speed / 2.0;
        action.red_led = 1;
        internal.begin_reverse = event.elapsed_sim_time;
    }

    function TargetCollisionStart(event, internal, action)
    {
        action.left_wheel = 0.0;
        action.right_wheel = 0.0;
    }

    function TopTouchStart(event, internal, action)
    {
        action.left_wheel = -Robot_Speed / 2.0;
        action.right_wheel = Robot_Speed / 2.0;
        action.red_led = 1;
        internal.begin_top_touch = event.elapsed_sim_time;
    }

    function robot_fsm(state, internal, event, action)
    {
        action.was_bumped = 0;
        action.was_top_touched = 0;
        if (!internal.initialized)
        {
            internal.begin_noise = event.elapsed_sim_time;
            internal.begin_reverse = event.elapsed_sim_time;
            internal.begin_top_touch = event.elapsed_sim_time;
            internal.last_noise = event.elapsed_sim_time;
            internal.last_reverse = event.elapsed_sim_time;

            internal.initialized = true;
        }
        switch (state)
        {
            case Robot_Start:
            {
                if (event.target_switch_pin)
                {
                    TargetWaitStart(event, internal, action);
                    return Robot_TargetWait;
                }
                else
                {
                    ObstacleWaitStart(event, internal, action);
                    return Robot_ObstacleWait;
                }
            } break;

            case Robot_ObstacleWait:
            {
                if (event.is_run_sig)
                {
                    ObstacleRunStart(event, internal, action);
                    return Robot_ObstacleRun;
                }
            } break;

            case Robot_ObstacleRun:
            {
                if (event.is_wait_sig)
                {
                    action.left_wheel = 0.0;
                    action.right_wheel = 0.0;
                    ObstacleWaitStart(event, internal, action);
                    return Robot_ObstacleWait;
                }
                else if (event.is_bumper)
                {
                    action.left_wheel = 0.0;
                    action.right_wheel = 0.0;
                    ObstacleCollisionStart(event, internal, action);
                    return Robot_ObstacleCollision;
                }
            } break;

            case Robot_ObstacleCollision:
            {
                if (event.is_wait_sig)
                {
                    ObstacleWaitStart(event, internal, action);
                    return Robot_ObstacleWait;
                }
                else if (!event.is_bumper)
                {
                    ObstacleRunStart(event, internal, action);
                    return Robot_ObstacleRun;
                }
            } break;

            case Robot_TargetWait:
            {
                if (event.is_run_sig)
                {
                    // Reset noise and reverse timers
                    internal.last_noise = event.elapsed_sim_time;
                    internal.last_reverse = event.elapsed_sim_time;
                    TargetRunStart(event, internal, action);
                    return Robot_TargetRun;
                }
            } break;

            case Robot_TargetRun:
            {
                if (event.is_wait_sig)
                {
                    TargetWaitStart(event, internal, action);
                    return Robot_TargetWait;
                }
                else if (event.is_top_touch)
                {
                    TopTouchStart(event, internal, action);
                    return Robot_TopTouch;
                }
                else if (event.elapsed_sim_time - internal.last_reverse > Reverse_Interval)
                {
                    internal.last_reverse = event.elapsed_sim_time;
                    ReverseStart(event, internal, action);
                    return Robot_Reverse;
                }
                else if (event.elapsed_sim_time - internal.last_noise > Noise_Interval)
                {
                    TrajectoryNoiseStart(event, internal, action);
                    return Robot_TrajectoryNoise;
                }
                else if (event.is_bumper)
                {
                    TargetCollisionStart(event, internal, action);
                    return Robot_TargetCollision;
                }
            } break;

            case Robot_TrajectoryNoise:
            {
                if (event.is_wait_sig)
                {
                    TargetWaitStart(event, internal, action);
                    return Robot_TargetWait;
                }
                else if (event.is_top_touch)
                {
                    TopTouchStart(event, internal, action);
                    return Robot_TopTouch;
                }
                else if (event.elapsed_sim_time - internal.begin_noise > Noise_Length)
                {
                    internal.last_noise = event.elapsed_sim_time;
                    TargetRunStart(event, internal, action);
                    return Robot_TargetRun;
                }
                else if (event.is_bumper)
                {
                    TargetCollisionStart(event, internal, action);
                    return Robot_TargetCollision;
                }
            } break;

            case Robot_Reverse:
            {
                if (event.is_wait_sig)
                {
                    TargetWaitStart(event, internal, action);
                    return Robot_TargetWait;
                }
                else if (event.is_top_touch)
                {
                    TopTouchStart(event, internal, action);
                    return Robot_TopTouch;
                }
                else if (event.elapsed_sim_time - internal.begin_reverse > Reverse_Length)
                {
                    TargetRunStart(event, internal, action);
                    return Robot_TargetRun;
                }
            } break;

            case Robot_TargetCollision:
            {
                action.was_bumped = 1;
                ReverseStart(event, internal, action);
                return Robot_Reverse;
            } break;

            case Robot_TopTouch:
            {
                action.was_top_touched = 1;
                if (event.is_wait_sig)
                {
                    TargetWaitStart(event, internal, action);
                    return Robot_TargetWait;
                }
                else if (event.elapsed_sim_time - internal.begin_top_touch > Top_Touch_Time)
                {
                    TargetRunStart(event, internal, action);
                    return Robot_TargetRun;
                }
                else if (event.is_bumper)
                {
                    TargetCollisionStart(event, internal, action);
                    return Robot_TargetCollision;
                }
            } break;
        }

        // Remain in current state
        return state;
    }

}
