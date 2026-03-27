package frc.robot.libraries;

import edu.wpi.first.wpilibj.Timer;

public class StateMachine<E extends Enum<E>> {
    protected E currentState;
    protected E desiredState;

    protected E overrideState;

    protected E defaultState;

    protected int pendingPriority = -1;
    protected E pendingState;

    protected final Timer stateTimer = new Timer();

    /**
     * 
     * @param startingState The state to start the statemachine in
     * @param defaultState The state to transition to if no state requests have been made in a tick. If null then it will stay on the last desired state.
     */
    public StateMachine(E startingState, E defaultState) {
        this.currentState = startingState;
        this.desiredState = startingState;

        this.defaultState = defaultState;

        if (this.defaultState != null) {
            this.pendingState = this.defaultState;
        } else {
            this.pendingState = startingState;
        }

        this.stateTimer.restart();
    }

    public E getCurrentState() {
        return currentState;
    }

    /**
     * Requests the state machine transitions to a new desired state
     * 
     * @param newState The new state to transfer to
     * @param priority An int representing the priority of the state change. Priorities should generally follow this format:
     * <ul>
     *      <li>0: Idle state</li>
     *      <li>1-10: Normal commands</li>
     *      <li>11-20: Manual overrides</li>
     *      <li>21-30: Safety overrides</li>
     * </ul>
     */
    public void requestDesiredState(E newState, int priority) {
        if (newState == null) {
            return;
        }

        if (priority >= pendingPriority) {
            this.pendingPriority = priority;
            this.pendingState = newState;
        }
    }

    public E getDesiredState() {
        if (this.overrideState != null) {
            return this.overrideState;
        }

        return this.desiredState;
    }

    public double getStateTimer() {
        return this.stateTimer.get();
    }

    public void restartStateTimer() {
        this.stateTimer.restart();
    }

    public void setOverrideState(E overrideState) {
        this.overrideState = overrideState;
        if (this.overrideState != null) {
            transitionTo(this.overrideState);
        }
        
    }

    public E getOverrideState() {
        return this.overrideState;
    }

    /**
     * Transitions the current state to the desired state
     **/
    public void transitionTo() {
        transitionTo(getDesiredState());
    }

    /**
     * Transitions the current state to the specified state
     * 
     * @param newState The state to transition too
     **/
    public void transitionTo(E newState) {
        if (this.overrideState != newState && this.overrideState != null) {
            return;
        }

        if (newState != currentState && newState != null) {
            this.currentState = newState;
            
            this.stateTimer.restart();
        }
    }

    /**
     * MUST BE CALLED AT THE START OF EVERY PERIODIC LOOP
     **/
    public void updateDesiredState() {
        if (pendingState != null) {
            this.desiredState = this.pendingState;
        }
        this.pendingPriority = -1;
        this.pendingState = this.defaultState;
    }
}
