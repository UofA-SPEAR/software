import React, {Component} from 'react';
import ReactDOM from 'react-dom';
import ROSLib from 'roslib';
import {StateMachine} from './state_machine';
import './index.css';

class Action extends Component {
    render() {
        return (
            <div className='action'>
                <span className='text'>{this.props.text}</span><br></br>
                <span className='id'>ID: {this.props.id}</span>
            </div>
        );
    }
}

class ExecutingDisplayAction extends Component {
    render() {
        const className = this.props.currentlyExecuting ? 'action_executing' : 'action_not_executing';
        return (
            <div className={className}>
                {this.props.action}
            </div>
        )
    }
}

class SelectableAction extends Component {
    onClick() {
        this.props.handleClick(this.props.action.props.id);
    }
    render() {
        return (
            <div className='selectable_action' onClick={() => this.onClick()}>
                {this.props.action}
            </div>
        );
    }
}

class App extends Component {
    constructor(props) {
        super(props);
        const ros = new ROSLib.Ros({url: 'ws://localhost:9090'});
        const stateMachine = new StateMachine(ros, () => {this.forceUpdate()});
        this.state = {
            ros: ros,
            stateMachine: stateMachine,
        }
    }

    render() {
        let actions;
        let buttons = [];

        const sm = this.state.stateMachine;

        if (sm.inManualModeState()) {
            actions = sm.getStatesToDisplay().map(display_state => {
                const actionComponent = <Action text={display_state.text} id={display_state.id} />;
                return (
                    <li key={display_state.id}>
                        <SelectableAction action={actionComponent} handleClick={() => sm.jumpToState(display_state.id)} />
                    </li>
                )
            })
        } else {
            actions = sm.getStatesToDisplay().map(display_state => {
                const currentlyExecuting = sm.currentlyExecuting(display_state.id);
                const actionComponent = <Action text={display_state.text} id={display_state.id} />;
                return (
                    <li key={display_state.id}>
                        <ExecutingDisplayAction action={actionComponent} currentlyExecuting={currentlyExecuting} />
                    </li>
                );
            });
        }
        
        if (sm.inManualModeState()) {
            buttons.push(<input type="file" id="file-input" onChange={(event) => sm.loadFromFile(event.target.files[0])} />)
        } else {
            buttons.push(<button onClick={() => sm.abort()}>Abort</button>);
        }

        return (
            <div>
                <ol className='actionlist'>
                    {actions}
                </ol>
                {buttons}
            </div>
        );
    }
}

ReactDOM.render(
    (
        <App />
    ),
    document.getElementById('root')
);