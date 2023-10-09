# Prob-Rob-2020-21_Intro-Probabilities (RBS Notes)

_(2023/09/25)_

- [Slides](/doc/lectures/prob-rob-2023-24_02_intro-probabilities.pdf) (2023/2024)
- [YouTube Video - Part 1](https://www.youtube.com/watch?v=mS5631VNOcg) (2020/2021)
- [YouTube Video - Part 2](https://www.youtube.com/watch?v=Tx4OYSnaaiQ) (2020/2021)

## Scenario

- Orazio (mobile platform) located in a building
- There is an access point in the building
- What is the WiFi signal strength sensed by Orazio at different locations
  (let's suppose that robot has 4 bar indicator, where 4 bars means maximum
  strength, 1 means weak signal and 0 no signal at all)?
  1. put Orazio in a location + start recording samples of measured signal
     strengths
     - will give statistic on signal strength at a location
  2. define set of intervals on the strengths
  3. to each interval assign a number
     - $P_{l_j}\left(S=s_i\right)=\frac{1}{N} N_i$
     - $l_j$ different locations in the building
     - $s_i,\text{ where }i=1,\dots,4$ is the signal strength intervals
       considered
     - $N_i$ is the number of times the signal strength $s_i$ was observed with
       the robot at the location $l_j$ in the building
     - $N$ is the total number of experiments at the location $l_j$
       - this number should be equal to each location?
       - or should the number be greater that e.g. 20 / 100 samples, the
         previous question becomes irrelevant?... probably this one!
  4. repeat process for each location

### Events and Probability

- **Event:** the signal strength $S$ falls in the interval $s_i _(example)_
  - characterise a subset of the possible outcome
  - event is either true or false (occurred or not!)
- Event $S_i \sim S=s_i$ to compact the notation
- $P_{l_j}\left(S_i\right)$ is the probability of the event of observing a
  signal strength, given a location $l_j$
- a probability is a function going from each measurable subset of the event 
  space $\Omega$ to the interval $\left[0,1\right]$

### Axioms

- **Important:** to be a probability it does not need to fit in a certain shape
  of probability functions! Only has to obey certain axioms!

1. $P(E) \geq 0$
   - an event cannot have a negative probability
   - in the worst case, the event has a 0 probability of occurring
2. $P(\bigcup_i E_i) = \sum_i P\left(E_i\right)$
   - disjoint events are events that are mutually exclusive (if one event
     occurs, the other one cannot occur)
   - an example of disjoint events is measuring the signal stregnth $s_1$ versus
     measuring signal strength $s_2$ (two different possible outcomes!)
   - probability of the union of a set of disjoint events is the sum of
     probabilities of the events
3. $P(\Omega)=1$
   - probability of the outcome set is 1!

### Continuous Domains

- in a continuous domain, the probability of the outcome assuming exactly a
  specific value $x is, in general, 0
  - in the case of signal strength, the respective variable is a continuous one
  - the event of signal strength being exactly equal to a certain value is
    almost 0, given that the probability of strength being exactly that value
    would be almost null...
- so, how do we define an event in continuous domain?
- we may define the event $P\left(X\lt x\right)$ that is true when outcome of
  the event is lower than a certain value
  - $=$ cumulative density function
  - cumulative density iis monotonically increasing
- e.g., the probability that an element falls in an interval
  $\left[x_1,x_2\right]$ is $P\left(x_2\right)-P\left(x_1\right)$

### Probability Density

- is nothing more than the derivative!
- $p\left(x\right)=\frac{\partial P\left(x\right)}{\partial x}$
- to compute the probability for a generic event $E \subseteq \Omega $ is by
  integration
  - $P\left(E\right)=\int_{x\in E} p\left(x\right) dx$
  - summing up all infinitesimal disjoint events that cover the subset $E$ of
    our possible outcomes

## Back to Scenario (Orazio)

- similar experiment and acquire statistics on how often the robot visits each
  location
- when acquiring stats about location, ignoring signal strength
- the result is the statistics $P\left(L_j\right)$

### Conditional Probability

- during experiment on signal strength, independent statistics at each location!
- so, we have a separate distribution over signal strength, given the location
  - $P_{l_j}\left(S=s_i\right) := P\left(S_i|L_j\right)$
    - distribution over the signal strengths **GIVEN** the knowledge of the
      location, and serves us to pick up the correct instance of an experiment
  - $L_j$ denotes the part of the event that controls the shape of the
    probability distribution
  - this definition remains a probability distribution over the signal strength,
    not on the robot location

### Chain Rule

- How likely is it that orazio sits at location $L_2$ and from there it measures
  the signal strength $S_3$?
  - $P\left(S_3,L_2\right)=?$
- a **certainty!** is that it is less likely of occurring the event
  $\{S_3,L_2\}$ that being at location $L_2$ and measuring **ANY** signal
  strength:
  - $P\left(S_3,L_2\right)\leq P\left(L_2\right)$
- it is as much likely of occurring the joiont event $\{S_3,L_2\}$ as being in
  $L_2$ **AND**, from $L_2$, measuring a signal strength $S_3$:
  - $P\left(S_3,L_2\right)=P\left(S_3|L_2\right)\cdot P\left(L_2\right)$
  - to speed up a little, think about how the probability of joint independent
    events is the product of the probability of occurring each independent event
  - the experiments of measuring the signal strength given a location,
    $S_i|L_j$, was independent from the experiment of acquiring statistics over
    the robot location in the building!
  - $L_2$ acts as the conditioner

### Joint Distributions

- If we consider that being at a location and measuring a signal strength is a
  particular event, these events are disjoints of all other events of being at
  other locations and measuring other signal strengths
- $P\left(S_3,L_2\right)$ is joint distribution, whose domain (possible
  outcomes) is all possible pairs of signal strengths and locations!
- we can formulate a grid for joint distribution!
  - columns correspond to locations + rows to signal strengths
  - limits the search space for when we want to know the probability of
    measuring a certain signal strength and being at a certain location
  - can respond any specific question in the joit distribution domain
- joint distribution can be computed using the chain rule!!!
  - multiply each element of the conditional $\{S_i|L_j\}$ by the probability of
    the conditioner $L_j$

### Marginals

- What is teh probabily of sensing a very weak signal strength $s_1$?
  - $P\left(S_1\right)=?$
  - $=$ probability measuring $s_1$ from any location and corresponds to a row
    in the joint table
  - $P\left(S_1\right)=\sum_j P\left(S_1,L_j\right)$
    - joint events $\lt S_i,L_j\gt$ are disjoint!
    - usage of the second axiom
    - $P\left(S_1\right)=P(\bigcup_j \lt S_1,L_j\gt) = \sum_j P\left(S_1,L_j\right)$
- Basically, we collapsed one variable in space > deleting a variable by summing
  up all possible events that include that variable in the part.... (?) 
- Collapsing the table by deleting the variable that corresponds to the marginal
  $=$ **marginalisation**
  - **Definition:** Marginalisation is a method that requires summing over the
    possible values of one variable to determine the marginal contribution of
    another
    _(source: https://towardsdatascience.com/probability-concepts-explained-marginalisation-2296846344fc)_
- **Marginalisation** is the process of suppressing a variable from a table
  - instead of having a 2D table of $\lt S_i,L_j\gt$ to $S_i$
  - e.g., do not care about the other dimension of the domain
  - compute the marginal of one variable by the joint distribution of 2
    variables

### Independence

- in reality not all phenomena are correlated:
  $P\left(A|B\right)=P\left(A\right)$
  - whatever the value of B, value of A remains
  - an example in Orazio world is an independent phenomenon might be the floor
    at which the elevator is located and signal strength (physically, the
    position of the elevator does not affect the signal strength)
- chain rule applies also to independent variables:
  - $P\left(A,B\right)=P\left(A|B\right)P\left(B\right)=P\left(A\right)P\left(B\right)$
  - joint distribution of independent variables is the product of their
    distributions
- **independence test**
  1. compute the marginal over the first variable
  2. compute the marginal over the second variable
  3. compute the joint distribution from the marginals as if the variables were
     independent
     - if the resulting table is the same as the original one, the variables are
       independent
     - otherwise, they are not!

### Localizing Orazio

- given the joi t table over locations and strengths
- we measure a signal strength $s_2$
- so, we want to figure the location of the robot Orazio, given the measured
  strength $s_2$:
  - $P\left(L|S_2\right)$
  - given that we do measure $s_2$, all other parts / signals are not relevant
    in this case!
  - however, the row of the table $s_2$ is not a valid distribution (it does not
    sum up to one, sums up to the marginal of $S_2$!)
  - even so, the row comprises all the knowledge we can consider given the
    evidence!
    - the more likely to measure the signal strength $s_2$ the more likely the
      robot is there
  - dividing each element of the row by the (constant) value of
    $P\left(S_2\right)$ renders the row a **valid distribution** over the
    possible locations
    - $P\left(L_j|S_2\right)=\frac{P\left(S_2,L_j\right)}{S_2}$

## Functions

- $X$ is a random variable distributed according to $P\left(X\right)$
- Let $y=f\left(x\right)$ be a generic function defined on possible values of
  $X$
- $Y$ is also a random variable whose distribution is the following one:
  - $P\left(Y\right)=\sum_{X_i=f^{-1}\left(Y\right)} P\left(X_i\right)$

## Conditional Independence

- Now suppose that we have 2 access points at different locations, that they
  transmit their signal at different frequencies and do not interfere with each
  other
- So, can compute statistics for the strength of first and second signals
- Let's assume that only element influencing signal strength is the location
  (the signals do not interfere)
  - $P\left(S^A|L\right)$
  - $P\left(S^B|L\right)$
- What about $P\left(S^A,S^B\right)$?
  - Are the two variables independent?
  - If I know $S^A$ can I tell anything about $S^B$?
  - we know that strength depends on location > location + strength are
    correlated
  - if strength of 1st signal influences the location, and 2nd signal is
    influenced by the location >>> signals are correlated through the location
    - so, they are not independent!
    - however, if the signal strength $S^A$ influences the location of where the
      robot could be, we can use the location information to get information on
      the signal strength $S^B$
  - **Note:** note that in the previous formula we do not have the location!
- So, if we know the robot location...
  - $P\left(S^A,S^B|L\right)=P\left(S^A|L\right)P\left(S^B|L\right)$
  - the two signals are independent if I know the location!
  - exploiting the structure of the problem (physical properties, state
    relations, etc.), we can figure out if two variables are correlated or not!


## Important Notes

- Marginalisation $\sim$ colapsing the variables from the joint probability
- Conditioning $\sim$ slicing the joint distribution and selecting all the parts
  of the joint probability that are consistent from the evidence that I get
- Chain rule $\sim$ probability that something happens is equal that one of the
  things happend multiplied by the probability of a second thing happeng given
  the first one

## Probability Identities

- All content discussed in this class was not assuming any particular shape on
  probability distribution! Even regardless the domain shape etc...
- A probability equation obtained:
  - by applying the axioms and
  - exploiting conditional independence from domain
  - ... hold regardless the shape of the densities!!!
- **Consequence:** you can add the same conditioning term to **all** terms of
  the equation and it still holds
  - $P\left(A,B\right)=P\left(A|B\right)P\left(B\right)$ $\rightarrow$
    $P\left(A,B|C\right)=P\left(A|B,C\right)P\left(B|C\right)$
  - $P\left(A|B\right)=\frac{P\left(A,B\right)}{B}$ $\rightarrow$
    $P\left(A|B,C\right)=\frac{P\left(A,B|C\right)}{P\left(B|C\right)}$
  - $P\left(A\right)=\sum_i P\left(A,B_i\right)$ $\rightarrow$
    $P\left(A|C\right)=\sum_i P\left(A,B_i|C\right)$
