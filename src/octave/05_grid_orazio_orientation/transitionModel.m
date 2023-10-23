function transition_probability_matrix = transitionModel(map_, row_from_, col_from_, theta_from_, control_input_)
  global THETA_VALUES;
  map_rows = rows(map_);
  map_cols = columns(map_);
  transition_probability_matrix = zeros(map_rows, map_cols, THETA_VALUES);

  #against each other cell and itself
	for row_to = 1:map_rows
		for col_to = 1:map_cols

      #available robot controls (corresponding to keyboard key values)
      global MOVE_FORWARD;
      global MOVE_BACKWARD;
      global ROTATE_LEFT;
      global ROTATE_RIGHT;

      #compute resulting position difference
      translation_rows = row_to - row_from_;
      translation_cols = col_to - col_from_;

      #allow only unit motions (1 cell): check if we have a bigger motion
      if(abs(translation_rows) > 1 || abs(translation_cols) > 1)
	      continue;
      endif

      #compute target robot position according to input
      target_row = row_from_;
      target_col = col_from_;
      switch (control_input_)
	      case MOVE_FORWARD
          switch theta_from_
            case 1
              # move right
              target_col++;
            case 2
              # move up
              target_row--;
            case 3
              # move left
              target_col--;
            case 4
              # move down
              target_row++;
            otherwise
              disp("Not a known angle")
          endswitch
        case MOVE_BACKWARD
		      switch theta_from_
            case 1
              # move left
              target_col--;
            case 2
              # move down
              target_row++;
            case 3
              # move right
              target_col++;
            case 4
              # move up
              target_row--;
            otherwise
              disp("Not a known angle")
          endswitch
        case ROTATE_LEFT
          target_row = target_row;
        case ROTATE_RIGHT
          target_row = target_row;
	      otherwise
          disp("ERROR: Command not recognized and normalizer will be inf")
		      return;
      endswitch

	    #check if the desired motion is infeasible
	    invalid_motion = false;
	    if (target_row < 1 || target_row > map_rows || target_col < 1 || target_col > map_cols) #if we're going over the border
		    invalid_motion = true;
	    elseif (map_(target_row, target_col) == 1 || map_(row_to, col_to) == 1) #obstacle in the goal cell
		    invalid_motion = true;
	    endif
	    if (invalid_motion)

	      #if the desired translation is zero
	      if (translation_rows == 0 && translation_cols == 0)
          transition_probability_matrix(row_to, col_to, theta_from_) = 1; #we stay with 100% probability (no motion has full confidence)
		      continue;
	      else
	        continue; #we cannot move
	      endif
	    endif

      #our motion is feasible - compute resulting transition
      switch (control_input_)
        case MOVE_FORWARD
          switch theta_from_
              case 1
                # move right
                if (translation_rows     ==  0 && translation_cols ==  1) 
                  transition_probability_matrix(row_to, col_to, theta_from_) = 1;
                endif
              case 2
                # move up
                if (translation_rows     ==  -1 && translation_cols ==  0) 
                  transition_probability_matrix(row_to, col_to, theta_from_) = 1;
                endif
              case 3
                # move left
                if (translation_rows     ==  0 && translation_cols ==  -1) 
                  transition_probability_matrix(row_to, col_to, theta_from_) = 1;
                endif
              case 4
                # move down
                if (translation_rows     ==  1 && translation_cols ==  0) 
                  transition_probability_matrix(row_to, col_to, theta_from_) = 1;
                endif
              otherwise
                disp("Not a known angle")
          endswitch
        case MOVE_BACKWARD
          switch theta_from_
              case 1
                # move left
                if (translation_rows     ==  0 && translation_cols ==  -1) 
                  transition_probability_matrix(row_to, col_to, theta_from_) = 1;
                endif
              case 2
                # move down
                if (translation_rows     ==  1 && translation_cols ==  0) 
                  transition_probability_matrix(row_to, col_to, theta_from_) = 1;
                endif
              case 3
                # move right
                if (translation_rows     ==  0 && translation_cols ==  1) 
                  transition_probability_matrix(row_to, col_to, theta_from_) = 1;
                endif
              case 4
                # move up
                if (translation_rows     ==  -1 && translation_cols ==  0) 
                  transition_probability_matrix(row_to, col_to, theta_from_) = 1;
                endif
              otherwise
                disp("Not a known angle")
          endswitch
        case ROTATE_LEFT
          # theta_to = theta_from_ + pi/2;
          theta_to = mod(theta_from_, THETA_VALUES) + 1;
          if (translation_rows     ==  0 && translation_cols == 0) 
            transition_probability_matrix(row_to, col_to, theta_to) = 1.0;
          endif
        case ROTATE_RIGHT
          theta_to = mod(theta_from_ - 2, THETA_VALUES) + 1;
          if (translation_rows     ==  0 && translation_cols == 0) 
            transition_probability_matrix(row_to, col_to, theta_to) = 1.0;
          endif
      endswitch
    endfor
  endfor
endfunction
