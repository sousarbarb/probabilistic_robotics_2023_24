# Prob-Rob-2020-21_Discrete-Filters-Exercise

_(2023/09/29)_

- [Slides](/doc/lectures/prob-rob-2023-24_05_discrete-filters-exercise.pdf) (2023/2024)
- [YouTube Video 2020/21](https://youtu.be/-mdFA70BxaA)
- [YouTube Video 2021/22](https://youtu.be/nuKDks6mFj0)

## Implementing a Bayes Filter

1. Describe our system as if it was noise free, a deterministic system!
   - If your model does not work with synthetic data with perfect data without
     noise, something is wrong with your model...
2. Then, increase the complexity of your model.

- Trasition model is a matrix over the possible next states the robot may be
  depending on the control input!

**Note:** the transition model is increasing the uncertainty over your current
state. Why?
- Marginalisation over the previous state
- Each marginalisation summs the columns of the joint distribution probability
  and so uniforming the probability in each transition

- **Initial State Belief**: uniform distribution over all map cells (in the case
  of the example shown in the class)
  - probability of each cell will be $1/{\# cells}$

## Usage

1. Open a terminal
2. Execute the following commands:
   ```sh
   cd <repo directory>/
   cd src/04_grid_orazio/
   octave-cli grid_orazio.m maps/map.txt
   ```
