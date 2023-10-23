function state_ground_truth = getNextState(map_, state_ground_truth_, control_input_)
  global THETA_VALUES;
  state_ground_truth = state_ground_truth_;

  #sampling setup
  minimum_probability    = rand(1);
  cumulative_probability = 0;

  #obtain all transition probabilities covering motions over the complete map
  transition_probability = transitionModel(map_, state_ground_truth_(1), state_ground_truth_(2), state_ground_truth_(3), control_input_);

  #available motion range
  min_row = state_ground_truth_(1)-1; #MOVE_UP
  max_row = state_ground_truth_(1)+1; #MOVE_DOWN
  min_col = state_ground_truth_(2)-1; #MOVE_LEFT
  max_col = state_ground_truth_(2)+1; #MOVE_RIGHT
  #over for the available motion range check if probability is higher than the extracted sample
  for (i=1:THETA_VALUES)
    for (row = min_row:max_row)
      for (col = min_col:max_col)
        cumulative_probability += transition_probability(row, col,i);
        if(cumulative_probability > minimum_probability)
          #return with new position
          state_ground_truth = [row, col, i];
          return;
        endif
      endfor
    endfor
  endfor
endfunction
