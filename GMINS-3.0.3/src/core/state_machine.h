#pragma once

#include <stdint.h>

template<typename StateType>
class StateMachine {
public:
    StateMachine(StateType initial_state) : current_state_(initial_state), previous_state_(initial_state) {}
    
    StateType getCurrentState() const { return current_state_; }
    StateType getPreviousState() const { return previous_state_; }
    
    bool transitionTo(StateType new_state) {
        if (canTransition(current_state_, new_state)) {
            previous_state_ = current_state_;
            current_state_ = new_state;
            onStateExit(previous_state_);
            onStateEnter(current_state_);
            return true;
        }
        return false;
    }
    
    bool hasStateChanged() const { return current_state_ != previous_state_; }
    uint32_t getStateTime() const { return millis() - state_enter_time_; }
    
protected:
    virtual bool canTransition(StateType from, StateType to) = 0;
    virtual void onStateEnter(StateType state) {}
    virtual void onStateExit(StateType state) {}
    
private:
    StateType current_state_;
    StateType previous_state_;
    uint32_t state_enter_time_;
};