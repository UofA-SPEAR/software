import {makeState} from './make_state';
import {GoalStatus, getGoalStatusName} from './goal_status';

export class DisplayState {
  constructor(id, text) {
    this.id = id;
    this.text = text;
  }
}

export class StateMachine {
  constructor(ros, refresh) {
    this.ros = ros;
    this.refresh = refresh;
    this.currentState = null;
    this.states = [];
    console.log('INITIALIZING STATE MACHINE');
    this.goToManualModeState();
  }

  loadFromFile(file) {
    if (!file) {
      return;
    }
    const reader = new FileReader();
    reader.onload = (event) => {
      const contents = event.target.result;
      const lines = contents.split(/\r?\n/g);
      const states = [];
      for (const line of lines) {
        const [actionClass, ...params] = line.split(' ');
        if (actionClass) {
          states.push(makeState(this.ros, actionClass, params.join(' ')));
        }
      }
      for (const state of states) {
        state.setNextStateCallback((status) => this.nextState(status));
      }
      this.states = states;
      this.refresh();
    };
    reader.readAsText(file);
  }

  nextState(status) {
    let willGoToManualState = false;
    if (status !== GoalStatus.Succeeded) {
      const fn = (status === GoalStatus.Preempted || status === GoalStatus.Aborted) ? console.log : console.error;
      fn(`Action failed with status ${status}: ${getGoalStatusName(status)}.`);
      willGoToManualState = true;
    } else {
      console.log('Action succeeded.');
      if (this.currentState === (this.states.length - 1)) {
        willGoToManualState = true;
      }
    }

    if (willGoToManualState) {
      this.goToManualModeState();
    } else {
      this.currentState += 1;
      const nextState = this.states[this.currentState];
      nextState.enter();
    }
    this.refresh();
  }

  abort() {
    if (this.inManualModeState()) {
      return;
    }
    this.states[this.currentState].cancel();
  }

  jumpToState(index) {
    if (!this.inManualModeState()) {
      throw Error('Attempting to jump to state while not in manual state!');
    }
    this.currentState = index;
    const nextState = this.states[this.currentState];
    console.log(`Jumping to state (${nextState.getDescription()})`);
    nextState.enter();
    this.refresh();
  }


  inManualModeState() {
    return this.currentState === -1;
  }

  goToManualModeState() {
    console.log('Entering manual mode.');
    this.currentState = -1;
  }

  getStatesToDisplay() {
    return this.states.map((state, index) => {
      return new DisplayState(index, state.getDescription());
    });
  }

  currentlyExecuting(id) {
    return this.currentState === id;
  }
}
