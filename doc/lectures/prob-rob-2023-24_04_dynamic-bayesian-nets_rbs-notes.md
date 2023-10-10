# Prob-Rob-2020-21_Model-Dynamic-Bayesian-Networks

_(2023/09/26)_

- [Slides](/doc/lectures/prob-rob-2023-24_04_dynamic-bayesian-nets.pdf) (2023/2024)
- [YouTube Video - Part 1](https://youtu.be/gcpfsMkv6zw?t=2182)
- [YouTube Video - Part 2](https://www.youtube.com/watch?v=SMmDmVMtGxE)

## Probabilistic Dynamic Systems

### Dynamic System Deterministic View

- $x_t=f\left(x_{t-1},u_{t-1}\right)$
- $z_t=h\left(x_t\right)$
- $f\left(x_{t-1},u_{t-1}\right)$: transition function
- $h\left(x_t\right)$: observation function
- $x_{t-1}$: previous state
- $x_{t}$: current state
- $u_{t-1}$: previous control / action
- $z_t$: current observation
- $\Delta t$: delay

If we have the state, we can predict the observation.

## Dynamic System Probabilistic View

- $x_t\sim p\left(x_t|x_{t-1},u_{t-1}\right)$ (transition model, instead of a
  function)
- $z_t\sim p\left(z_t|x_t\right)$ (observation model)

where $x_{t-1}$, $x_t$ and $z_t$ become random variables

## Dynamic Bayesian Networks (DBN)

### Evolution of a Dynamic System

1. Starting from a known initial state distribution $p\left(x_0\right)$
2. A control $u_0$ becomes available
3. The transition model $p\left(x_t|x_{t-1},u_{t-1}\right)$ correlates the
   current state $x_1$ with the previous control $u_0$ and the previous state
   $x_0$
4. The observation model $p\left(z_t|x_t\right)$ correlates the observation
   $z_1$ and the current state $x_1$
```
(u_0)
  |
  --------.
          v
(x_0)-->(x_1)--> ....
          |
          v
        (z_1)
```

The previous directed graphical model leads to a recurrent structure that
depends on the time $t$:

- time slices
- graphical representations of stochastic dynamic processes
- repetitive structure over time!

### States in a DBN

- Domain of the states $x_t$, contros $u_t$ amd obseravtions $z_t$ not
  restricted to be boolean or discrete:
  - Robot localisation with a laser range finder
    - States $x_t\in SE(2)$, isometries on a plane
    - Observations $z_t\in \R^{\#beams}$, laser range measurements
    - Controls $u_t\in \R^2$, translational and rotational speed
  - Hidden Markov Models (HMMs): discrete time discrete state systems
    - States $x_t\in \left[X_1,\dots,X_{N_x}\right]$, finite states
    - Observations $z_t \in \left[Z_1,\dots,Z_{N_z}\right]$, finite observations
    - Controls $u_t \in \left[U_1,\dots,U_{N_u}\right]$, finite controls
- Inference in DBN requires to design a data structure that can represent a
  distribution over states

## Inference on DBN

- In a dynamic systems, "usually" we knwo:
  - observations $z_{1:T}$ made by the system, because we _measure_ them
  - controls $u_{0:T-1}$, because we _issue_ them
- Typical inferences in a DBN:
  - Filtering: know the distribution of the current state knowing all everything
    else (compute the distribution of the most recent state)
    - $p\left(x_T | u_{0:T-1},z_{1:T}\right)$
  - Smoothing: estimate a location in the past
    - $p\left(x_t | u_{0:T-1},z_{1:T}\right),0 \lt t \lt T$
    - "generally", provides a better estimate than filtering, but depends on
      what happens / the system evolves after instant $t$ (typically!, because
      we have more information)
  - Max a Posteriori: given all the knowledge, interested in knowing the most
    likely trajectory over the state
    - $\argmax_{x_{0:T}} p\left(x_{0:T} | u_{0:T-1},z_{1:T}\right)$
- Using traditional tools for Bayes Netoworks not a good idea:
  - too many variables (potentially infinite) render the solution intractable
  - domains not necessarily discrete
- But we can exploit the recurrent structure to design procedures to take
  advantage of it

### Belief

- Algorithms for performing inference on DBN keep track of the estimate of a
  distribution of states
- This distribution should be stored in an appropriate data structure!
  - depends on knowledge of characteristics of the distribution
    (e.g. Gaussian) + domain state variables (continuous vs discrete)
- When writing $b\left(x_t\right)$, we mean our current belief of
  $p\left(x_t|\dots\right)$
- Algorithms performing inference on DBN work by updating a belief!
- Discrete state $x\in\{X_{1:n}\}$ >>> array $x$ of float values
  - each cell $x\left[i\right]=p\left(x=X_i\right)$ contains the probability of
    that state
- Continuous state + distributed according to Gaussian >>> belief = mean +
  covariance matrix
- Continuous state + unknown distribution >>> approximate representation (e.g.,
  weighted samples of state values)

## Recursive Bayes Equation

### Filtering

- **Query:** $p\left(x_T | u_{0:T-1},z_{1:T}\right)$
  - determine the state distribution at time $T$ knowing all the discrete
    controls and all discrete observations from previously
- **Know:**
  - Observations $z_{1:T}$
  - Controls $u_{0:T-1}$
  - $p\left(x_t|x_{t-1},u_{t-1}\right)$: transition model, equivalent to the
    deterministic transition fucntion $f$ that, given a previous state $x_{t-1}$
    and control $u_{t-1}$ tells us how likely it is to lan in state $x_t$
    - distribution of the possible state at time $t$ controlled by two knobs
      $u_{t-1}$ and $x_{t-1}$
  - $p\left(z_t|x_t\right)$: obserbation model, equivalent to the deterministic
    observation function $h$ that, given the current state $x_t$, tells us how
    likely it is to observe $z_t$
  - belief $b\left(x_{t_1}\right)$ about the previous state
    (equivalente to $p\left(x_{t_1}|u_{0:t-2},z_{1:t-1}\right)$)

Filtering Inference:

1. $p\left(x_T | u_{0:T-1},z_{1:T}\right)=$
   $p\left(x_t | z_t, u_{0:t-1},z_{1:t-1}\right)$ _(splitting $z_t$ from the_
   _equation)_
   - recall the conditional Bayes rule $p\left(A|B,C\right)=$
     $\frac{p\left(B|A,C\right)p\left(A|C\right)}{p\left(B|C\right)}$
     - explanation of Bayes rule:
       - $p\left(A|B,C\right)=$
         $\frac{p\left(A,B,C\right)}{p\left(B,C\right)}=$
         $\frac{p\left(B|A,C\right)p\left(A|C\right)p\left(C\right)}{p\left(B|C\right)p\left(C\right)}=$
         $\frac{p\left(B|A,C\right)p\left(A|C\right)}{p\left(B|C\right)}$
     - $\frac{p\left(z_t|x_t,u_{0:t-1},z_{1:t-1}\right)p\left(x_t|u_{0,t-1},z_{1:t-1}\right)}{p\left(z_t|u_{0:t-1},z_{1:t-1}\right)}$
       - $A=x_t$
       - $B=z_t$
       - $C=u_{0:t-1},z_{1:t-1}$
2. let the denominator $\eta_t=1/p\left(z_t|u_{0:t-1},z_{1:t-1}\right)$
   - the denominator does not depend on the state $x$, i.e., the state does not
     influence this factor
   - to the extent of our computation, $\eta_t$ is just a normalising constant!
   - (we will come back to this denominator!)
3. filtering equation becomes
   $\eta_t \cdot p\left(z_t|x_t,u_{0:t-1},z_{1:t-1}\right)p\left(x_t|u_{0,t-1},z_{1:t-1}\right)$
   - recalling the directed graphical model of our dynamic temporal system
   - if we know $x_t$, we do not need to know $u_{0:t-1},z_{1:t-1}$ to predict
     $z_t$, since state $x_t$ encodes all the knowledge about the past
     _**(Markov assumption!)**_
   - $p\left(z_t|x_t,u_{0:t-1},z_{1:t-1}\right)=p\left(z_t|x_t\right)$
     _(is our observation model)_
4. thus, our current equation is $p\left(x_t | u_{0:t-1},z_{1:t}\right)=$
   $\eta_t \cdot p\left(z_t|x_t\right)p\left(x_t|u_{0,t-1},z_{1:t-1}\right)$
   - but second part of equation obscure...
5. however, if we would know the probability distribution of the previous state,
   we can predict the distribution of $x_t$
   - $p\left(x_t|u_{0,t-1},z_{1:t-1}\right)=p\left(x_t|x_{t-1},u_{t-1}\right)$
   - however, we do not have $x_{t-1}$!!!
6. by combining maginalisation
   ( $p\left(A|C\right)=\sum_B p\left(A,B|C\right)$ )
   and the chain rule
   ( $p\left(A,B|C\right)=p\left(A|B,C\right)p\left(B|C\right)$ ), we can obtain
   the following:
   - $p\left(A|C\right)=\sum_B p\left(A|B,C\right)p\left(B|C\right)$
7. going back to $p\left(x_t|u_{0,t-1},z_{1:t-1}\right)$
   - assuming:
     - $A=x_t$
     - $B=x_{t-1}$
     - $C=u_{0,t-1},z_{1:t-1}$
   - $p\left(x_t|u_{0,t-1},z_{1:t-1}\right) =$
     $\sum_{x_{t-1}} p\left(x_t|x_{t-1},u_{0,t-1},z_{1:t-1}\right) p\left(x_{t-1}|u_{0,t-1},z_{1:t-1}\right) =$
     $\sum_{x_{t-1}} p\left(x_t|x_{t-1},u_{t-1}\right) p\left(x_{t-1}|u_{0,t-1},z_{1:t-1}\right)$
     - note that $p\left(x_t|x_{t-1},u_{t-1}\right)$ is our known transition
       model
     - as for $p\left(x_{t-1}|u_{0,t-1},z_{1:t-1}\right)$, it is the probability
       distribution of the previous state given whatever I know up to the
       previous instante **including** the previous control
     - but this last control $u_{t-1}$ has no influence on $x_{t-1}$, if we do
       not know $x_t$
8. final equation figurint out that hte recursive filtering equation is the
   following one:
   - $p\left(x_t | u_{0:t-1},z_{1:t}\right)=$
     $\eta_t \cdot p\left(z_t|x_t\right) \cdot \sum_{x_{t-1}} p\left(x_t|x_{t-1},u_{t-1}\right) p\left(x_{t-1}|u_{0,t-2},z_{1:t-1}\right)$
   - $b\left(x_t\right)=p\left(x_t | u_{0:t-1},z_{1:t}\right)$
   - $b\left(x_{t-1}\right)=p\left(x_{t-1}|u_{0,t-2},z_{1:t-1}\right)$
   - **Final equation:** $b\left(x_t\right)=\eta_t \cdot p\left(z_t|x_t\right) \cdot \sum_{x_{t-1}} p\left(x_t|x_{t-1},u_{t-1}\right) b\left(x_{t-1}\right)$
     - tells us how to update a current belief once new observations / controls
       become available

Note that the normalizer $\eta_t$ just a constant ensuring that the belief
$b\left(x_t\right)$ is still a probability distribution:

$\eta_t=1/p\left(z_t|u_{0:t-1},z_{1:t-1}\right)=$
$1/\sum_{x_t} p\left(z_t|x_t\right)\sum_{x_{t-1}}p\left(x_t|x_{t-1},u_{t-1}\right) b\left(x_{t-1}\right)$

#### Alternative Formulation

- Relies in a different view from the classical formulation
  ```
  u_{t-1} -----.
               v
  x_{t-1} --> x_t
  ```
  - from the previous Bayesian network, we know $u_{t-1}$ and $x_{t-1}$
  - basically, we want to shift all the previous knowledge to $x_t$
  - marginalising ~ colapsing the variables space
  - conditioning ~ slicing
- **Predict:** incorporate in the last belief $b_{t-1|t-1}$ the most recent
  control $u_{t-1}$
  - control is known >>> can work with a "2D" distribution selected according to
    the current $u_{t-1}$
    - x axis: $x_t$
    - y axis: $y_t$
    - 3rd dimension represents $p\left(x_t|x_{t-1},u_{t-1}\right)$
  - Ingredients:
    - transition model $p\left(x_t|x_{t-1},u_{t-1}\right)$
    - Prior belief: $p\left(x_{t-1}|t-1\right)$
    - Control $u_{t-1}$
  - multiply the table by the prior ~ marginalisation
  - all done was doing a chain rule that will return a joint distribution, then
    marginalisation
  - from the transition model and last state, compute the following joint
    distribution through chain rule:
    - $p\left(x_t,x_{t-1}|t-1\right)=p\left(x_t|x_{t-1}, u_{t-1}\right)p\left(x_{t-1}|t-1\right)=$
      $p\left(x_t|x_{t-1}, u_{t-1}\right)b_{t-1|t-1}$
  - from the joint, remove $x_{t-1}$ through marginalisation:
    - $b_{t|t-1}=p\left(x_{t}|t-1\right)=$
      $\sum_{x_{t-1}} p\left(x_t,x_{t-1}\right|t-1)$
  - programmatically for disccrete case:
    ```cpp
    BeliefType b_pred = BeliefType::Zero;
    for (x_i : X)
      for (x_j : X)
        b_pred += b[x_i] * transitionModel(x_j,x_i,u);
    ```
- **Update:** incorporate in the predicted belieb $b_{t|t-1}$ the new
  measurement $z_t$
  - Ingredients:
    - predicted belief $p\left(x_t|t-1\right)=b_{t|t-1}$
    - observation model $p\left(z_t|x_t\right)$
    - known measurement $z_t$
  - from the predicted belief $b_{t|t-1}$, compute the joint distribution that
    predicts the observation:
    - joint over state and measurement (chain rule):
      - $p\left(x_t,z_t\right)=p\left(z_t|x_t\right)p\left(x_t|t-1\right)=$
        $p\left(z_t|x_t\right)b_{t|t-1}$
    - condition on the actual measurement:
      - $b_{t|t}=p\left(x_t|t\right)=\frac{p\left(x_t,z_t|t\right)}{p\left(z_t|t\right)}$
  - programmatically for discrete case:
    ```cpp
    float normalizer = 0;

    for (x_i : X)
      b[x_i] = b_pred[x_i] * observationModel(z,x_i);
      normalizer += b[x_i];

    b *= 1./normalizer;   // divide all elements in the array by the normalizer
    ```
    - do not know if it was intentional, but funny the way to normalize:
      1. only performs 1 division (computation of the constat 1/normalizer)
      2. then, multiply each element by the constant computed previously
         - note that division is more computational costly than multiplication!!
