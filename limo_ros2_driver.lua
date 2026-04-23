-- limo_ros2_driver.lua
-- Script CoppeliaSim équivalent au plugin diff_drive de Gazebo.
-- Attaché à l'objet base_footprint_visual dans la scène.
--
-- Topics ROS2 :
--   Subscribe : /cmd_vel  (geometry_msgs/msg/Twist)
--   Publish   : /odometry (nav_msgs/msg/Odometry)
--               /tf       (tf2_msgs/msg/TFMessage)

local sim    = require('sim')
local simROS2

-- Paramètres cinématiques LIMO (mode différentiel 4WD)
local WHEEL_RADIUS = 0.045    -- r [m]
local HALF_TRACK   = 0.1725   -- R [m]  demi-voie

-- Chemins exacts des joints tels que vus dans la hiérarchie CoppeliaSim
local PATH_FL = '/base_footprint_visual/front_left_wheel'
local PATH_FR = '/base_footprint_visual/front_right_wheel'
local PATH_RL = '/base_footprint_visual/rear_left_wheel'
local PATH_RR = '/base_footprint_visual/rear_right_wheel'


-- ─────────────────────────────────────────────────────────────────────────────
function sysCall_init()
    -- Vérifier que simROS2 est disponible
    local ok
    ok, simROS2 = pcall(require, 'simROS2')
    if not ok then
        print('[LIMO-ROS2] ERREUR : simROS2 non disponible.')
        print('  → Lancer CoppeliaSim depuis un terminal avec ROS2 sourcé.')
        print('  Détail : ' .. tostring(simROS2))
        return
    end
    print('[LIMO-ROS2] simROS2 chargé.')

    -- Handles des joints
    jFL = sim.getObject(PATH_FL)
    jFR = sim.getObject(PATH_FR)
    jRL = sim.getObject(PATH_RL)
    jRR = sim.getObject(PATH_RR)
    print('[LIMO-ROS2] Joints trouvés.')

    -- Activer les moteurs (contrôle en vitesse)
    for _, j in ipairs({jFL, jFR, jRL, jRR}) do
        sim.setObjectInt32Param(j, sim.jointintparam_motor_enabled, 1)
        sim.setJointTargetVelocity(j, 0)
    end

    -- Subscriber /cmd_vel
    cmdVelSub = simROS2.createSubscription(
        '/cmd_vel',
        'geometry_msgs/msg/Twist',
        'cmdVelCallback'
    )

    -- Publisher odométrie
    odomPub = simROS2.createPublisher('/odometry', 'nav_msgs/msg/Odometry')

    -- État odométrie
    x        = 0.0
    y        = 0.0
    theta    = 0.0
    lastTime = sim.getSimulationTime()

    -- Commande courante
    cmdVlin = 0.0
    cmdVang = 0.0

    print('[LIMO-ROS2] Prêt. /cmd_vel → roues | /odometry publié.')
end


-- ─────────────────────────────────────────────────────────────────────────────
function cmdVelCallback(msg)
    cmdVlin = msg.linear.x
    cmdVang = msg.angular.z
end


-- ─────────────────────────────────────────────────────────────────────────────
function sysCall_actuation()
    if not simROS2 then return end

    -- Cinématique différentielle inverse : Twist → vitesses roues [rad/s]
    local vL = (cmdVlin - cmdVang * HALF_TRACK) / WHEEL_RADIUS
    local vR = (cmdVlin + cmdVang * HALF_TRACK) / WHEEL_RADIUS

    sim.setJointTargetVelocity(jFL, vL)
    sim.setJointTargetVelocity(jRL, vL)
    sim.setJointTargetVelocity(jFR, vR)
    sim.setJointTargetVelocity(jRR, vR)
end


-- ─────────────────────────────────────────────────────────────────────────────
function sysCall_sensing()
    if not simROS2 then return end

    local t  = sim.getSimulationTime()
    local dt = t - lastTime
    lastTime = t
    if dt <= 0 then return end

    -- Vitesses angulaires mesurées (moyenne gauche / droite)
    local wL = (sim.getJointVelocity(jFL) + sim.getJointVelocity(jRL)) / 2.0
    local wR = (sim.getJointVelocity(jFR) + sim.getJointVelocity(jRR)) / 2.0

    -- Cinématique directe : vitesses roues → Twist
    local v = WHEEL_RADIUS * (wR + wL) / 2.0
    local w = WHEEL_RADIUS * (wR - wL) / (2.0 * HALF_TRACK)

    -- Intégration odométrie
    theta = theta + w * dt
    x     = x + v * math.cos(theta) * dt
    y     = y + v * math.sin(theta) * dt

    local q   = eulerToQuat(0, 0, theta)
    local now = simROS2.getTime()

    -- Odometry
    simROS2.publish(odomPub, {
        header         = {stamp = now, frame_id = 'odom'},
        child_frame_id = 'base_footprint',
        pose = {
            pose = {
                position    = {x = x, y = y, z = 0.0},
                orientation = q
            },
            covariance = {0.001,0,0,0,0,0, 0,0.001,0,0,0,0, 0,0,0,0,0,0,
                          0,0,0,0,0,0,     0,0,0,0,0,0,     0,0,0,0,0,0.001}
        },
        twist = {
            twist = {
                linear  = {x = v, y = 0, z = 0},
                angular = {x = 0, y = 0, z = w}
            },
            covariance = {0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,
                          0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0}
        }
    })

end


-- ─────────────────────────────────────────────────────────────────────────────
function eulerToQuat(roll, pitch, yaw)
    local cy = math.cos(yaw   * 0.5)
    local sy = math.sin(yaw   * 0.5)
    local cp = math.cos(pitch * 0.5)
    local sp = math.sin(pitch * 0.5)
    local cr = math.cos(roll  * 0.5)
    local sr = math.sin(roll  * 0.5)
    return {
        x = sr*cp*cy - cr*sp*sy,
        y = cr*sp*cy + sr*cp*sy,
        z = cr*cp*sy - sr*sp*cy,
        w = cr*cp*cy + sr*sp*sy
    }
end


-- ─────────────────────────────────────────────────────────────────────────────
function sysCall_cleanup()
    if not simROS2 then return end
    for _, j in ipairs({jFL, jFR, jRL, jRR}) do
        sim.setJointTargetVelocity(j, 0)
    end
    if cmdVelSub then simROS2.shutdownSubscription(cmdVelSub) end
    if odomPub   then simROS2.shutdownPublisher(odomPub)      end
    print('[LIMO-ROS2] Nœud arrêté.')
end
