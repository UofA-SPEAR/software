import RosLib from 'roslib';
import { GoalStatus } from './goal_status';

function make_relative_coord_state(ros, params) {
    let position = new RosLib.Vector3();
    const orientation = new RosLib.Quaternion({x: 0, y: 0, z: 0, w: 1.0});
    const [x, y] = params.split(' ');
    position.x = parseFloat(x);
    position.y = parseFloat(y);
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
    const description = `Move to relative coordinate (${x}, ${y})`;

    return new State({
        ros: ros,
        actionServerName: 'move_base',
        actionName: 'move_base_msgs/MoveBaseAction',
        actionResultMessageType: 'move_base_msgs/MoveBaseActionResult',
        goalMessage: goalMessage,
        description: description,
    });
}

export function make_state(ros, action_class, params) {
    const function_map = {
        'MoveToRelativeCoord': make_relative_coord_state,
    }
    return function_map[action_class](ros, params);
}


class State {
    constructor({ros, actionServerName, actionName, actionResultMessageType, goalMessage, description}) {
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