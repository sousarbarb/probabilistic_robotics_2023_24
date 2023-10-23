function observation_probability = observationModel(map_, row_, col_, theta_, observations_)
  observation_probability = 1;
  map_rows = rows(map_);
  map_cols = columns(map_);
  theta_ = 1;
  global THETA_VALUES
  orientation_to_cells_mapping = zeros(2, length(observations_), THETA_VALUES);
  orientation_to_cells_mapping(:,:,1) = [ -1 1 0 0; 
                                          0 0 -1 1];
  orientation_to_cells_mapping(:,:,2) = [ 0 0 1 -1; 
                                          -1 1 0 0];
  orientation_to_cells_mapping(:,:,3) = [ 1 -1 0 0; 
                                          0 0 1 -1];
  orientation_to_cells_mapping(:,:,4) = [ 0 0 -1 1; 
                                          1 -1 0 0];
  #evaluate cell occupancy
  cell_bumper_west_occupied    = 0;
  cell_bumper_est_occupied     = 0;
  cell_bumper_south_occupied   = 0;
  cell_bumper_north_occupied   = 0;
  cell_ = [row_; col_];
  cell_bumper_west = cell_ + orientation_to_cells_mapping(:,1,theta_);
  if (cell_bumper_west(1) >= 1) && (cell_bumper_west(1) <= map_rows) && (cell_bumper_west(2) >= 1) && (cell_bumper_west(2) <= map_cols)
    cell_bumper_west_occupied = map_(cell_bumper_west(1), cell_bumper_west(2));
  endif
  cell_bumper_est = cell_ + orientation_to_cells_mapping(:,2,theta_);
  if (cell_bumper_est(1) >= 1) && (cell_bumper_est(1) <= map_rows) && (cell_bumper_est(2) >= 1) && (cell_bumper_est(2) <= map_cols)
    cell_bumper_est_occupied = map_(cell_bumper_est(1), cell_bumper_est(2));
  endif
  cell_bumper_south = cell_ + orientation_to_cells_mapping(:,3,theta_);
  if (cell_bumper_south(1) >= 1) && (cell_bumper_south(1) <= map_rows) && (cell_bumper_south(2) >= 1) && (cell_bumper_south(2) <= map_cols)
    cell_bumper_south_occupied = map_(cell_bumper_south(1), cell_bumper_south(2));
  endif
  cell_bumper_north = cell_ + orientation_to_cells_mapping(:,3,theta_);
  if (cell_bumper_north(1) >= 1) && (cell_bumper_north(1) <= map_rows) && (cell_bumper_north(2) >= 1) && (cell_bumper_north(2) <= map_cols)
    cell_bumper_north_occupied = map_(cell_bumper_north(1), cell_bumper_north(2));
  endif

  #update probability depending on observations
  if (cell_bumper_west_occupied == observations_(1))
	  observation_probability *= .8;
  else
    observation_probability *= .2;
  endif
  if (cell_bumper_est_occupied == observations_(2))
	  observation_probability *= .8;
  else
    observation_probability *= .2;
  endif
  if (cell_bumper_south_occupied == observations_(3))
	  observation_probability *= .8;
  else
    observation_probability *= .2;
  endif
  if (cell_bumper_north_occupied == observations_(4))
	  observation_probability *= .8;
  else
    observation_probability *= .2;
  endif
endfunction
