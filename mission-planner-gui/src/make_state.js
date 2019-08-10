import RosLib from 'roslib';
import { GoalStatus } from './goal_status';

function make_gps_coordinate_state(ros, params) {
    let [lat, lon] = params.split(' ');
    lat = parseFloat(lat);
    lon = parseFloat(lon);
    return new NavigateToGpsState({
        ros, lat, lon
    });
}

function quaternion_from_yaw(yaw) {
    const s = Math.sin(yaw * 0.5);
    return new RosLib.Quaternion({x: 0.0, y: 0.0, z: s, w: Math.cos(yaw * 0.5)});
}

function make_relative_coord_with_angle_state(ros, params, description) {
    let position = new RosLib.Vector3();
    let orientation = new RosLib.Quaternion({x: 0, y: 0, z: 0, w: 1.0});
    let [x, y, yaw] = params.split(' ');
    x = parseFloat(x);
    y = parseFloat(y);
    yaw = parseFloat(yaw);
    position.x = x;
    position.y = y;
    orientation = quaternion_from_yaw(yaw);
    const pose = new RosLib.Pose({
        position,
        orientation,
    });
    const goalMessage = {
        target_pose: {
            header: {
                frame_id: 'base_link',
            },
            pose: pose,
        },
    };
    if (description === undefined) {
        description = `Move to relative coordinate (${x}, ${y}) with yaw angle (${yaw} = ${Math.round(yaw*180.0/Math.PI)}°)`;
    }

    return new ActionState({
        ros: ros,
        actionServerName: 'move_base',
        actionName: 'move_base_msgs/MoveBaseAction',
        actionResultMessageType: 'move_base_msgs/MoveBaseActionResult',
        goalMessage: goalMessage,
        description: description,
    });
}

function make_relative_coord_state(ros, params) {
    const [x, y] = params.split(' ');
    return make_relative_coord_with_angle_state(ros, `${x} ${y} 0`);
}

function make_move_forward_state(ros, params) {
    const x = parseFloat(params);
    return make_relative_coord_with_angle_state(ros, `${x} 0 0`, `Move forward ${x} meters`);
}

function make_turn_left_state(ros, params) {
    const angle = parseFloat(params);
    return make_relative_coord_with_angle_state(ros, `0 0 ${angle}`, `Turn left ${angle} radians = ${Math.round(angle*180.0/Math.PI)}°`);
}

function make_turn_right_state(ros, params) {
    const angle = parseFloat(params);
    return make_relative_coord_with_angle_state(ros, `0 0 ${-angle}`, `Turn right ${angle} radians = ${Math.round(angle*180.0/Math.PI)}°`);
}

function make_manual_control_state(ros, params) {
    return new TakeManualControlState();
}

export function make_state(ros, action_class, params) {
    const function_map = {
        'MoveToRelativeCoordWithAngle': make_relative_coord_with_angle_state,
        'MoveToRelativeCoord': make_relative_coord_state,
        'MoveForward': make_move_forward_state,
        'TurnLeft': make_turn_left_state,
        'TurnRight': make_turn_right_state,
        'MoveToGpsCoord': make_gps_coordinate_state,
        'TakeManualControl': make_manual_control_state,
    }
    if (action_class in function_map) {
        return function_map[action_class](ros, params);
    } else {
        console.warn(`Unrecognized action class "${action_class}". Making a dummy state instead.`);
        return new DummyState(action_class, params);
    }
}

class AbstractState {
    constructor() {
        if (new.target === AbstractState) {
            throw new TypeError('Please do not construct AbstractState directly.');
        }
    }
    setNextStateCallback(callback) {
        throw new TypeError('nextStateCallback() was not defined and so was called for AbstractState.');
    }
    enter() {
        throw new TypeError('enter() was not defined and so was called for AbstractState.');
    }
    cancel() {
        throw new TypeError('cancel() was not defined and so was called for AbstractState.');
    }
    getDescription() {
        throw new TypeError('getDescription() was not defined and so was called for AbstractState.');
    }
}

class DummyState extends AbstractState {
    constructor(action_class, params) {
        super();
        this.nextStateCallback = null;
        this.timeout = null;
        this.description = `${action_class} ${params}`;
    }

    setNextStateCallback(callback) {
        this.nextStateCallback = callback;
    }

    enter() {
        this.timeout = window.setTimeout(() => this.nextStateCallback(GoalStatus.Succeeded), 1000);
    }

    cancel() {
        window.clearTimeout(this.timeout);
        window.setTimeout(() => this.nextStateCallback(GoalStatus.Preempted) , 0);
    }

    getDescription() {
        return `Dummy state: ${this.description}`;
    }
}


class ActionState extends AbstractState {
    constructor({ros, actionServerName, actionName, actionResultMessageType, goalMessage, description}) {
        super();
        this.ros = ros;
        this.actionServerName = actionServerName;
        this.actionName = actionName;
        this.actionResultMessageType = actionResultMessageType;
        this.goalMessage = goalMessage;
        this.description = description;

        this.actionClient = new RosLib.ActionClient({
            ros: this.ros,
            serverName: this.actionServerName,
            actionName: this.actionName,
        });
        this.resultListener = new RosLib.Topic({
            ros: this.ros,
            name: `${this.actionServerName}/result`,
            messageType: this.actionResultMessageType,
        });
        this.nextStateCallback = null;
        this.goal = null;
    }

    setNextStateCallback(callback) {
        this.nextStateCallback = callback;
        this.resultListener.subscribe(message => {
            if (this.goal === null) {
                return;
            }
            if (message.status.goal_id.id !== this.goal.goalID) {
                return;
            }
            this.nextStateCallback(message.status.status);
            this.goal = null;
        });
    }

    enter() {
        console.log(`Entering state (${this.getDescription()})`);
        this.goal = new RosLib.Goal({
            actionClient: this.actionClient,
            goalMessage: this.goalMessage,
        });
        this.goal.on('timeout', (event) => {
            this.nextStateCallback(GoalStatus.Aborted);
        });
        this.goal.send();
    }

    cancel() {
        console.log(`Cancelling state (${this.getDescription()})`);
        this.actionClient.cancel();
    }

    getDescription() {
        return this.description;
    }
}

class TakeManualControlState extends AbstractState {
    setNextStateCallback(callback) {
        this.nextStateCallback = callback;
    }

    enter() {
        this.nextStateCallback(GoalStatus.Aborted);
    }

    cancel() {
        this.nextStateCallback(GoalStatus.Preempted);
    }

    getDescription() {
        return 'Take manual control';
    }
}

class NavigateToGpsState extends AbstractState {
    constructor({ros, lat, lon}) {
        super();
        this.ros = ros;
        this.lat = lat;
        this.lon = lon;
        
        this.actionClient = new RosLib.ActionClient({
            ros: this.ros,
            serverName: 'move_base',
            actionName: 'move_base_msgs/MoveBaseAction',
        });
        this.resultListener = new RosLib.Topic({
            ros: this.ros,
            name: 'move_base/result',
            messageType: 'move_base_msgs/MoveBaseActionResult',
        });
        this.nextStateCallback = null;
        this.goal = null;

        this.gpsToFrameClient = new RosLib.Service({
            ros: ros,
            name: 'gps_to_frame',
            serviceType: 'spear_rover/GpsToFrame',
        });
    }

    setNextStateCallback(callback) {
        this.nextStateCallback = callback;
        this.resultListener.subscribe(message => {
            if (this.goal === null) {
                return;
            }
            if (message.status.goal_id.id !== this.goal.goalID) {
                return;
            }
            this.nextStateCallback(message.status.status);
            this.goal = null;
        });
    }

    enter() {
        console.log(`Entering state (${this.getDescription()})`);

        const target_frame = 'map';
        console.log(`Translating (${this.lat}, ${this.lon}) from gps to ${target_frame} coords...`);
        const request = new RosLib.ServiceRequest({
            frame: {data: target_frame},
            gps_coord: {x: this.lat, y: this.lon, z: 0},
        });
        this.gpsToFrameClient.callService(request, result => {
            console.log(`Finished: (${result.coord.x}, ${result.coord.y})`)
            let position = new RosLib.Vector3();
            const orientation = new RosLib.Quaternion({x: 0, y: 0, z: 0, w: 1.0});
            position.x = result.coord.x;
            position.y = result.coord.y;

            this.goal = new RosLib.Goal({
                actionClient: this.actionClient,
                goalMessage: {
                    target_pose: {
                        header: {frame_id: target_frame},
                        pose: new RosLib.Pose({position, orientation}),
                    },
                },
            });
            this.goal.on('timeout', (event) => {
                this.nextStateCallback(GoalStatus.Aborted);
            });
            this.goal.send();
        });
    }

    cancel() {
        console.log(`Cancelling state (${this.getDescription()})`);
        this.actionClient.cancel();
    }

    getDescription() {
        return `Move to gps coordinate (${this.lat}, ${this.lon})`;
    }
}