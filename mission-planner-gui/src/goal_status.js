export const GoalStatus = {
    Pending: 0,
    Active: 1,
    Preempted: 2,
    Succeeded: 3,
    Aborted: 4,
    Rejected: 5,
    Preempting: 6,
    Recalling: 7,
    Recalled: 8,
    Lost: 9,
};

export function getGoalStatusName(goalStatus) {
    for (const name in GoalStatus) {
        if (GoalStatus[name] === goalStatus) {
            return name;
        }
    }
    return null;
}