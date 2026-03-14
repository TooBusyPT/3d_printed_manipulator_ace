#define FSM_COUNT 5

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  unsigned long tes, tis;
} fsm_t;

extern fsm_t* fsm_array[FSM_COUNT];

void set_fsm_state(fsm_t& fsm, int new_state);
void update_all_fsm_tis(unsigned long cur_time);