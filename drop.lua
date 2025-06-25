SCRIPT_NAME = "Drop System Controls"
SCRIPT_NAME_SHORT = "Drop"
SCRIPT_VERSION = "2.0.0"

REFRESH_FREQUENCY_HZ = 10
REFRESH_PERIOD_MS    = 1000/REFRESH_FREQUENCY_HZ

MISSION_REFRESH_FREQUENCY_HZ = 1
MISSION_REFRESH_LOOP_COUNT = REFRESH_FREQUENCY_HZ/MISSION_REFRESH_FREQUENCY_HZ

MAV_CMD_ID = {WAYPOINT = 16}
MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}
MAV_FRAME = { GLOBAL = 0, GLOBAL_RELATIVE_ALT = 3, GLOBAL_TERRAIN_ALT = 10}
MAV_CMD_INT = { DO_SET_MODE = 176, DO_CHANGE_SPEED = 178, DO_REPOSITION = 192, 
                  GUIDED_CHANGE_SPEED = 43000, GUIDED_CHANGE_ALTITUDE = 43001, GUIDED_CHANGE_HEADING = 43002 }

ALT_FRAME = { ABSOLUTE = 0, ABOVE_HOME = 1, ABOVE_ORIGIN = 2, ABOVE_TERRAIN = 3 }
FLIGHT_MODE = {AUTO=10, RTL=11, LOITER=12, GUIDED=15, QHOVER=18, QLOITER=19, QRTL=21}

INITIAL_PARAM_TABLE_KEY = 68
PARAM_TABLE_PREFIX = 'DROP'

TIME_MS = millis()
LOCATION = ahrs:get_location()

gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME.." "..SCRIPT_VERSION)

local function send_emergency(msg) gcs:send_text(MAV_SEVERITY.EMERGENCY, SCRIPT_NAME_SHORT..': '..msg) end
local function send_error(msg)     gcs:send_text(MAV_SEVERITY.ERROR, SCRIPT_NAME_SHORT..': '..msg)     end
local function send_text(msg)      gcs:send_text(MAV_SEVERITY.INFO, SCRIPT_NAME_SHORT..': '..msg)      end

local function follow_frame_to_mavlink(follow_frame)
    local mavlink_frame = MAV_FRAME.GLOBAL
    if (follow_frame == ALT_FRAME.ABOVE_TERRAIN) then
        mavlink_frame = MAV_FRAME.GLOBAL_TERRAIN_ALT
    elseif (follow_frame == ALT_FRAME.ABOVE_HOME) then
       mavlink_frame = MAV_FRAME.GLOBAL_RELATIVE_ALT
    end
    return mavlink_frame
end

local function extrapolate_location(location1, location2, distance)
    if type(location1) ~= 'userdata' then
        send_error('extrapolate_location argument #1 must be of type \'userdata\'')
        return nil
    end

    if type(location2) ~= 'userdata' then
        send_error('extrapolate_location argument #2 must be of type \'userdata\'')
        return nil
    end

    if type(distance) ~= 'number' then
        send_error('extrapolate_location argument #3 must be of type \'number\'')
        return nil
    end

    local unit_diff_vector = Vector3f()
    unit_diff_vector:x(location2:lat()-location1:lat())
    unit_diff_vector:y(location2:lng()-location1:lng())
    unit_diff_vector:z(0)
    unit_diff_vector:normalize()

    local diff_vector = unit_diff_vector:scale(distance)

    local new_location = location1:copy()
    new_location:offset(diff_vector:x(), diff_vector:y())

    return new_location
end

local function set_vehicle_target_location(location)
    if type(location) ~= 'userdata' then
        send_error('set_vehicle_target_location argument must be of type \'userdata\'. type: '..type(location))
        return false
    end

    if not pcall(function()
        vehicle:set_target_location(location)
    end ) then
        send_error('set_vehicle_target_location could not set vehicle target location')
        return false
    else
        return true
    end
end

local function set_vehicle_target_speed(speed)
    if type(speed) ~= 'number' then
        send_error('set_vehicle_target_speed argument must be of type \'number\'')
        return false
    end

    if not pcall(function()
        param:set_and_save('AIRSPEED_CRUISE', speed)
    end ) then
        send_error('set_vehicle_target_speed could not set vehicle target speed')
        return false
    else
        return true
    end
end

local function add_param_table(param_table_key, param_table_name, param_list)
    if not param_table_key then
        send_error('add_param_table no param table key')
        return false
    end
    
    if param_table_key < 0 or param_table_key > 200 then
        send_error('add_param_table param table key out of range (0-200)')
        return false
    end

    if not param_table_name then
        send_error('add_param_table no param table name')
        return false
    end

    if not param_list or #param_list == 0 then
        send_error('add_param_table no param list')
        return false
    end

    if not pcall( function()
        param:add_table(param_table_key, param_table_name, #param_list)
    end ) then
        send_error('add_param_table could not add param table \''..param_table_name..'\'')
        return false
    end

    for param_index, param_data in ipairs(param_list) do
        local param_name =    param_data[1]
        local param_default = param_data[2]

        if not param_name then
            send_error('add_param_table missing param name')
            return false
        end

        if not param_default then
            param_default = 0
        end

        if not pcall( function()
            param:add_param(param_table_key, param_index, param_name, param_default)
        end ) then
            send_error('add_param_table could not add param \''..param_table_name..param_name..'\'')
            return false
        end
    end

    return true
end

local function add_drop_param_tables()
    DROP_COUNT = 0

    if not INITIAL_PARAM_TABLE_KEY then
        send_error('add_drop_param_tables no initial param table key')
        return false
    end

    if not PARAM_TABLE_PREFIX then
        send_error('add_drop_param_tables no param table prefix')
        return false
    end

    local param_table_key = INITIAL_PARAM_TABLE_KEY
    local param_table_name = PARAM_TABLE_PREFIX..'_'
    -- Specify generic drop params
    local param_list = {
        {'COUNT', 2}, ------------ Specifies the number of drop systems
        {'DURATION', 1.0}, ------- Specifies the length of time that the drop system should be open for
        {'DELAY', 1.0}, ---------- Specifies how long it takes for the payload to fall out of the drop system
        {'MSG_ID', 17490}, ------- Specifies the script message ID that this script should be waiting for
        {'TRGT_MSG_ID', 17491}, -- Specifies the script message ID for a targeted drop
        {'AIRSPEED', 15},  ------- Specifies the target airspeed that the plane should be flying at while dropping
        {'OVERSHOOT', 20}, ------- Specifies the distance between the drop position and the planned waypoint
        {'RT_OF_DESC', 8}  ------- Specifies the rate of descent of the payload
    }
    -- Add a param table for generic drop options
    add_param_table(param_table_key, param_table_name, param_list)

    local drop_count = param:get('DROP_COUNT')

    if not drop_count then
        send_error('add_drop_param_tables no drop count')
        return false
    end

    if drop_count < 0 or drop_count > 8 then
        send_error('add_drop_param_tables drop count out of range (0-8)')
        return false
    end

    for drop_index = 1, drop_count do
        -- Specify unique drop param table properties
        -- Offset the table key by the unique drop's index, giving a unique key to each drop param table
        param_table_key = INITIAL_PARAM_TABLE_KEY + drop_index
        param_table_name = PARAM_TABLE_PREFIX..drop_index..'_'
        -- Specify unique drop params
        param_list = {
            {'RC_CH', 8}, ------------------- Specifies the RC channel that's responsible for initiating the drop sequence
            {'RC_LOW_BND', 800}, ------------ Specifies the the upper bound of the RC input drop sequence trigger range
            {'RC_UPP_BND', 1000}, ----------- Specifies the the lower bound of the RC input drop sequence trigger range
            {'SRV_NUM', 9 + drop_index-1}, -- Specifies the servo number associated with the drop system
            {'SRV_OPEN', 800}, -------------- Specifies the value that should be sent to the servo to open the drop system
            {'SRV_SHUT', 2200}, ------------- Specifies the value that should be sent to the servo to close the drop system
        }
        -- Add a param table for unique drop options
        add_param_table(param_table_key, param_table_name, param_list)
    end

    DROP_COUNT = drop_count
    return true
end

add_drop_param_tables()

if not DROP_COUNT then
    send_error('no drop count')
end

local function check_drop_index(drop_index)
    if type(drop_index) ~= 'number' or drop_index < 1 or drop_index > DROP_COUNT then
        return false
    else
        return true
    end
end

local function get_generic_drop_param(param_name)
    if not PARAM_TABLE_PREFIX then
        send_error('get_generic_drop_param no param table prefix')
        return nil
    end

    if type(param_name) ~= 'string' then
        send_error('get_generic_drop_param argument #1 must be of type \'string\'')
        return nil
    end

    local param_value = param:get(PARAM_TABLE_PREFIX..'_'..param_name)

    if not param_value then
        send_error('get_generic_drop_param generic drop param \''..param_name..'\' does not exist')
    end

    return param_value
end

local function get_unique_drop_param(drop_index, param_name)
    drop_index = math.tointeger(drop_index)
    if not check_drop_index(drop_index) then
        send_error('get_unique_drop_param drop #'..drop_index..' does not exist')
        return nil
    end

    if not PARAM_TABLE_PREFIX then
        send_error('get_unique_drop_param no param table prefix')
        return nil
    end

    if type(param_name) ~= 'string' then
        send_error('get_unique_drop_param argument #2 must be of type \'string\'')
        return nil
    end

    local param_value = param:get(PARAM_TABLE_PREFIX..math.tointeger(drop_index)..'_'..param_name)

    if not param_value then
        send_error('get_unique_drop_param unique drop param \''..param_name..'\' does not exist')
    end

    return param_value
end

local function get_drop_rc(drop_index)
    drop_index = math.tointeger(drop_index)
    if not check_drop_index(drop_index) then
        send_error('get_drop_rc drop #'..drop_index..' does not exist')
        return false
    end

    local channel  =    get_unique_drop_param(drop_index, 'RC_CH')
    local lower_bound = get_unique_drop_param(drop_index, 'RC_LOW_BND')
    local upper_bound = get_unique_drop_param(drop_index, 'RC_UPP_BND')

    local current_pos = rc:get_pwm(channel)

    if not current_pos then
        send_error('get_drop_rc RC channel #'..channel..' does not exist')
        return false
    end

    local in_bounds = (
            current_pos >= lower_bound 
        and current_pos <= upper_bound
    )

    return in_bounds
end

local function set_drop_state(drop_index, drop_state)
    drop_index = math.tointeger(drop_index)
    if not check_drop_index(drop_index) then
        send_error('set_drop_state drop #'..math.tointeger(drop_index)..' does not exist')
        return false
    end

    local servo_num = get_unique_drop_param(drop_index, 'SRV_NUM')

    local servo_pos
    if drop_state then
        servo_pos = get_unique_drop_param(drop_index, 'SRV_OPEN')
    else
        servo_pos = get_unique_drop_param(drop_index, 'SRV_SHUT')
    end

    SRV_Channels:set_output_pwm_chan(servo_num-1, servo_pos)
end

local drop_data = {}

for drop_index = 1, DROP_COUNT do
    set_drop_state(drop_index, false)

    drop_data[drop_index] = {
        prev_drop_rc = get_drop_rc(drop_index),
        drop_time_ms = -1000 * get_generic_drop_param('DURATION')
    }
end

local function initiate_drop_sequence(drop_index, distance)
    drop_index = math.tointeger(drop_index)
    if not check_drop_index(drop_index) then
        send_error('initiate_drop_sequence drop #'..drop_index..' does not exist')
        return false
    end

    drop_data[drop_index].drop_time_ms = TIME_MS
    if type(distance) == 'number' then
        gcs:send_text(MAV_SEVERITY.INFO, 'Dropping payload #'..drop_index..' dist '..math.floor(distance+0.5)..'m')
    else
        gcs:send_text(MAV_SEVERITY.INFO, 'Dropping payload #'..drop_index)
    end
    return true
end

local function update_drop_states()
    local duration = get_generic_drop_param('DURATION')

    for drop_index = 1, DROP_COUNT do
        if TIME_MS <= drop_data[drop_index].drop_time_ms + 1000*duration then
            set_drop_state(drop_index, true)
        else
            set_drop_state(drop_index, false)
        end
    end
end

local function handle_rc()
    for drop_index = 1, DROP_COUNT do
        local drop_rc = get_drop_rc(drop_index)

        if drop_rc and not drop_data[drop_index].prev_drop_rc then
            initiate_drop_sequence(drop_index)
        end

        drop_data[drop_index].prev_drop_rc = drop_rc
    end
end

local drop_target = nil

local function update_adjusted_drop_location()
    if not drop_target then
        send_error('set_adjusted_drop_location no drop target')
        return false
    end

    local wind_velocity = ahrs:wind_estimate()
    local flight_time = drop_target.height / (get_generic_drop_param('RT_OF_DESC') - wind_velocity:z())

    local drift = wind_velocity:scale(flight_time)
    local adjusted_drop_location = drop_target.target_location:copy()
    adjusted_drop_location:offset(-drift:x(), -drift:y())

    drop_target.adjusted_drop_location = adjusted_drop_location

    return true
end

local function set_drop_target(drop_index)
    if mission:get_current_nav_id() ~= MAV_CMD_ID.WAYPOINT then
        send_error('set_drop_target drop target command must be followed by a waypoint command')
        return false
    end

    local mission_index = mission:get_current_nav_index()

    local target_location = vehicle:get_target_location()
    target_location:change_alt_frame(ALT_FRAME.ABOVE_TERRAIN)

    drop_target = {
        drop_index = drop_index,

        mission_index = mission_index, 
        original_mission_item = mission:get_item(mission_index),
        original_airspeed = param:get('AIRSPEED_CRUISE'),

        target_location = target_location,
        height = target_location:alt() * 0.01
    }

    update_adjusted_drop_location()

    return true
end

local function clear_drop_target()
    if not drop_target then
        send_error('clear_drop_target no drop target')
        return false
    end

    local drop_location = drop_target.adjusted_drop_location

    local waypoint_location = extrapolate_location(
        drop_location, 
        LOCATION, 
        -get_generic_drop_param('OVERSHOOT') - param:get('WP_RADIUS')
    )

    local new_mission_item = mavlink_mission_item_int_t() 
    new_mission_item:frame(    drop_target.original_mission_item:frame()    )
    new_mission_item:command(  drop_target.original_mission_item:command()  )
    new_mission_item:seq(      drop_target.original_mission_item:seq()      )
    new_mission_item:z(        drop_target.original_mission_item:z()        )

    new_mission_item:x(waypoint_location:lat())
    new_mission_item:y(waypoint_location:lng())

    mission:set_item(drop_target.mission_index, new_mission_item)

    if vehicle:get_mode() ~= FLIGHT_MODE.AUTO then
        vehicle:set_mode(FLIGHT_MODE.AUTO)
    end

    mission:set_item(drop_target.mission_index, drop_target.original_mission_item)

    drop_target = nil

    return true
end

local mission_refresh_counter = MISSION_REFRESH_LOOP_COUNT

local function handle_mission()
    local message = {}  
    message.time_ms, message.id, message.p1, message.p2, message.p3 = mission_receive()

    if message.id then
    if message.id == get_generic_drop_param('MSG_ID') then
        initiate_drop_sequence(message.p1)

    elseif message.id == get_generic_drop_param('TRGT_MSG_ID') then
        set_drop_target(message.p1)
        if vehicle:get_mode() ~= FLIGHT_MODE.GUIDED then
            vehicle:set_mode(FLIGHT_MODE.GUIDED)
        end
        set_vehicle_target_speed(get_generic_drop_param('AIRSPEED'))
    end
    end

    if drop_target then
        mission_refresh_counter = mission_refresh_counter + 1

        if vehicle:get_mode() ~= FLIGHT_MODE.GUIDED then
            mission_refresh_counter = MISSION_REFRESH_LOOP_COUNT
            set_vehicle_target_speed(drop_target.original_airspeed)
            clear_drop_target()
            return true
        end

        if mission_refresh_counter >= MISSION_REFRESH_LOOP_COUNT then
            mission_refresh_counter = 0
            update_adjusted_drop_location()

            local vehicle_target_location = extrapolate_location(
                drop_target.adjusted_drop_location, 
                LOCATION, 
                -get_generic_drop_param('OVERSHOOT') - 2*param:get('WP_LOITER_RAD')
            )

            set_vehicle_target_location(vehicle_target_location)
        end

        local ground_velocity = ahrs:groundspeed_vector()

        local vehicle_target_location = vehicle:get_target_location()

        local unit_target_diff_vector = LOCATION:get_distance_NE(vehicle_target_location)
        unit_target_diff_vector:normalize()

        local drop_diff_vector = LOCATION:get_distance_NE(drop_target.adjusted_drop_location)

        local total_drop_distance = LOCATION:get_distance(drop_target.adjusted_drop_location)
        local normal_drop_distance = drop_diff_vector:x()*unit_target_diff_vector:x()
                                   + drop_diff_vector:y()*unit_target_diff_vector:y()

        if normal_drop_distance <= get_generic_drop_param('DELAY')*ground_velocity:length() then
            mission_refresh_counter = MISSION_REFRESH_LOOP_COUNT
            initiate_drop_sequence(drop_target.drop_index, total_drop_distance)
            set_vehicle_target_speed(drop_target.original_airspeed)
            clear_drop_target()
        end

        return true
    end
end

local function update()
    TIME_MS = millis()
    LOCATION = ahrs:get_location()

    handle_rc()
    handle_mission()

    update_drop_states()

    return update, REFRESH_PERIOD_MS
end

local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
       send_emergency('Error: '..err)
       return protected_wrapper, 1000
    end
    return protected_wrapper, REFRESH_PERIOD_MS
end

return protected_wrapper, REFRESH_PERIOD_MS
