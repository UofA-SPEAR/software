from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Tuple, Union
from uuid import UUID, uuid4

from smach import StateMachine
from smach.container import Container
from smach.user_data import UserData


@dataclass
class ContainerSaveState:
    """
    Checkpoint for a single non-nested container. The checkpoint represents the
    state of the container right before the state labelled `label` is executed.
    `userdata` stores the userdata as passed into execute() (i.e. not the
    parent userdata).
    """
    label: str
    userdata: UserData


@dataclass
class Checkpoint:
    uid: UUID
    timestamp: datetime
    save_states: List[ContainerSaveState]


class ResumableStateMachine(StateMachine):
    """
    A StateMachine variant which can be started from any checkpoint.
    """
    def __init__(self, outcomes: List[str], input_keys: List[str] = [], output_keys: List[str] = []):
        super().__init__(outcomes, input_keys=input_keys, output_keys=output_keys)
        self._resume_save_state = None

    def _request_resume(self, save_states: List[ContainerSaveState]):
        save_state, *remaining_save_states = save_states
        if save_state.label not in self.get_children():
            raise ValueError(f'Cannot resume to state {save_state} as it does not exist in this state machine')
        self._resume_save_state = save_state
        if len(remaining_save_states) > 0:
            child = self.get_children()[save_state.label]
            if not isinstance(child, ResumableStateMachine):
                raise ValueError(f'Cannot resume non-resumable state {save_state.label}')

            child._request_resume(remaining_save_states)

    def execute(self, parent_ud: UserData = UserData()):
        if self._resume_save_state is not None:
            resume_label = self._resume_save_state.label
            resume_ud = self._resume_save_state.userdata
            self._resume_save_state = None
            # Skip directly to the state we want
            old_initial_states = self.get_initial_states()
            self.set_initial_state([resume_label], resume_ud)
            # Prevent values in parent userdata being written into this userdata
            # XXX: This almost certainly won't work if you use remappings.
            input_keys_to_remove = self._input_keys & set(resume_ud.keys())
            self._input_keys -= input_keys_to_remove
            try:
                return super().execute(parent_ud=parent_ud)
            finally:
                self.set_initial_state(old_initial_states)
                self._input_keys |= input_keys_to_remove
        else:
            return super().execute(parent_ud=parent_ud)

    def execute_from(self, checkpoint: Checkpoint, parent_ud: UserData = UserData()) -> Union[str, None]:
        self._request_resume(checkpoint.save_states)
        return self.execute(parent_ud)


class ContainerHistory:
    def __init__(self, container: Container):
        self._current_uds: Dict[Tuple[str, ...], UserData] = {}
        self._current_labels: Tuple[str, ...] = ()
        self._checkpoints: List[Checkpoint] = []
        self._register_callbacks(container, tuple())

    def _register_callbacks(self, container: Container, labels: Tuple[str, ...]):
        container.register_start_cb(self._start_cb, [labels])
        container.register_transition_cb(self._transition_cb, [labels])
        container.register_termination_cb(self._termination_cb, [labels])
        for child_label, child in container.get_children().items():
            if isinstance(child, Container):
                self._register_callbacks(child, (*labels, child_label))

    @staticmethod
    def _clone_ud(ud: UserData):
        new_ud = UserData()
        for key in ud.keys():
            new_ud[key] = deepcopy(ud[key])
        return new_ud

    def _start_cb(self, userdata: UserData, initial_states: List[str], labels: Tuple[str, ...]):
        labels = (*labels, initial_states[0])
        self._current_uds[labels] = self._clone_ud(userdata)
        self._current_labels = labels
        self.save()

    def _transition_cb(self, userdata: UserData, active_states: List[str], labels: Tuple[str, ...]):
        labels = (*labels, active_states[0])
        self._current_uds[labels] = self._clone_ud(userdata)
        self._current_labels = labels
        self.save()

    def _termination_cb(self, userdata: UserData, terminal_states: List[str], outcome: str, labels: Tuple[str, ...]):
        # Nothing to do here, really, since we save checkpoints *before* a state
        # is executed and not after.
        pass

    def save(self):
        save_states = []
        for i in range(1, len(self._current_labels) + 1):
            key = tuple(self._current_labels[:i])
            label = key[-1]
            userdata = self._current_uds[key]
            save_states.append(ContainerSaveState(label, userdata))

        self._checkpoints.append(Checkpoint(
            uid=uuid4(),
            timestamp=datetime.now(),
            save_states=save_states,
        ))

    @property
    def checkpoints(self):
        return self._checkpoints

    @property
    def latest_checkpoint(self):
        return self._checkpoints[-1]
