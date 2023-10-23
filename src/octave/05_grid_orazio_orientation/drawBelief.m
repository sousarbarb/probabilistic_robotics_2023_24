function drawBelief(state_belief_, map_)
	global THETA_VALUES
	num_rows = rows(map_);
	num_cols = columns(map_);
	state_belief_2D = zeros(num_rows, num_cols);
	#state_belief_2D
	for (row = 1:num_rows)
		for (col = 1:num_cols)
			for (theta = 1:THETA_VALUES)
				state_belief_2D(row, col) += state_belief_(row, col, theta);
			endfor
		endfor
	endfor
	#invert belief value for plotting (0:black: 100% confidence, 1:white: 0% confidence)
	plotted_state_belief_ = flipud(ones(size(state_belief_2D)) - state_belief_2D);

  #plot a colormap with the respective belief values
	colormap(gray(64));
	hold on;
	image([0.5, columns(map_)-0.5], [0.5, rows(map_)-0.5], plotted_state_belief_*64);
	hold off;
endfunction

