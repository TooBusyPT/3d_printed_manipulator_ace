#include <Arduino.h>
#include "FiniteStateMachine.h"

void set_fsm_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed tis is reset
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
  }
}

void update_all_fsm_tis(unsigned long cur_time)
{
    for (int i = 0; i < FSM_COUNT; i++) {
        fsm_array[i]->tis = cur_time - fsm_array[i]->tes;
    }
}