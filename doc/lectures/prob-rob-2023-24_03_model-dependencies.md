# Prob-Rob-2020-21_Model-Dependencies-with-Bayesian-Networks

_(2023/09/26)_

- [Slides](/doc/lectures/prob-rob-2023-24_03_model-dependencies.pdf) (2023/2024)
- [YouTube](https://www.youtube.com/watch?v=gcpfsMkv6zw)

## Considerations about dimensions

### Orazio

- Scene = 2 access points (AP) + orazio moving + 1 elevator
- Quantities:
  - signal strengths from 2 APs
  - floor of the elevator
  - location of Orazio

### Dimensionality

- Having:
  - 4 possible levels of strength per AP
  - 2 APs
  - 6 floors
  - 100 locations
- How many possible disjoint outcomes of events do I have?
  - $4\cdot 4\cdot 6\cdot 100 = 9600$ (number of possible configurations)
- What if we gift Orazio a monochromatic camera with resolution "poorVGA"
  (10 x 10 ~ camera of optical mouses), and 16 levels of gray?
  - Number of disjoint events becomes
    $4\cdot 4\cdot 6\cdot 100\cdot \left(10\cdot 10\right)^16 = 9600\cdot 10^{32}$
  - Enough to challenge some good computer...

## Role of Conditional Independence

- To perform a query of the type $P\left(S^A|{S_2}^B\right)$, where ${S_2}^B$ is
  the probability of the second AP having signal strength of $S_2$ how do we do
  it?
  1. Start from the joint probability distribution
     $P\left(S^A,{S_2}^B, I^{1,1}, \dots, I^{10,10},L,E\right)$
  2. Eliminate all variables that are non relevant through marginalisation (lot
     of sums and collaping the joint distribution "table" / space)
  3. Use the _Bayes' Rule_ to get the answer when we have a two dimensional
     space (${S^A|{S_2}^B}$)
     - to obtain the one dimensional answer, I need to compute the two
       dimensional table that contains both $S^A$ and $S^B$
     - then, we can do contitional probability
- But if we know that signal strength is only affected by the location, we can
  ignore a lot of variables and speed up the processing time
- So,
  - signal strength of an access point depends only on the location
  - pixels in the image depend only on the location
    - if nothing but Orazio moves (static world)
    - if the light conditions do not change
  - elevator lives in his own world (as long as Orazio does not take it)

## Modeling Phenomena: Bayesian Networks

- **Bayesian Network**
  - probabilistic graphical model that represents a set of variables and their
    conditional dependencies via a directed acyclic graph (DAG)
    - **DAG:** directed graph with no directed cycles (no loops!, cannot go
      back)
  - each node represents a variable of the inference process
  - one arrow / edge exists if the source influences the destination (cause VS
    effect)
  - no loops it is a directed acyclic graph
    ```
     L                                        E
     |                                    elevator
     |------------------------------.
     |   |         |       |        |
     v   v         v       v        v
    S^a S^b     I^{1,1} I^{1,2} I^{10,10}
     signal              pixel
    strengths         intensities
    ```
      - all we need to know to render a picture is the location of the robot
        (assuming static world, similar light conditions etc.)
      - similar to signal strength
- For storing the conditional probabilities...
  - 100 possible robot locations
  - 4*100 for signal strength A
  - 4*100 for signal strength B
  - 16*100 for each pixel, as they are independent **GIVEN** the lcoation
  - 6 numbebrs for the elevator
  - So...
    - $100+2\cdot 4 \cdot 100 + 100\cdot 16 \cdot \left(10\cdot 10\right)+6=$
      $160906$
    - much more less variables for joint distribution
    - and captures the same information as before!

### Using the Order

- By storing the conditional probability need much less numbers
- Can recover the joint distribution and perform inference on that using the
  chain rule
- Bayesian Networks are a formalism to highlight independence between variables,
  without loss of information
- Each node:
  - represents a variable $x$
  - and stores a conditional probability! (the node stores the probability, not
    the edge!)

## Inference on Bayesian Networks

### Example

- We want $P\left(S^A|I_7^{1,1}\right)=?$
  1. compute the joint probability
     $P\left(S^A,S^B,I_7^{1,1}, \dots,I^{10,10},L,E\right) =$
     $P\left(S^A|S^B,I_7^{1,1}, \dots,I^{10,10},L,E\right)P\left(S^B|I_7^{1,1}, \dots,I^{10,10},L,E\right)\left(I_7^{1,1}|\dots,I^{10,10},L,E\right)\cdots P\left(L|E\right)P\left(E\right)=$
     $P\left(S^A|L\right)P\left(S^B|L\right)\left(I_7^{1,1}|L\right)\cdots P\left(L\right)P\left(E\right)$
     - chain rule
     - then, through conditional probability, only location L matters for $S^A$,
       etc., also as presented in the Bayesian network presented earlier
  2. marginalise out the variables we do not need:
     - $P\left(S^A,I_7^{1,1}\right)=\sum_{S^B} \sum_{I^{1,2}} \cdots \sum_{I^{10,10}}\sum_{L}\sum_E P\left(S^A,S^B,I_7^{1,1}, \dots,I^{10,10},L,E\right)$
     - sum over all vallues of variables to suppress
     - collapse the joint distribution dimension space to only 2 dimensions
  3. use conditioning to get the answer
     - $P\left(S^A|I_7^{1,1}\right)=\frac{P\left(S^A,I_7^{1,1}\right)}{\sum_{S^A} P\left(S^A,I_7^{1,1}\right)}$

### Considerations

- Would we get the same answer if the elevator was not in the domain? **YES**
- Knowing the intensity of another pixel would provide us with more information?
  **YES**
  - probability of measuring the intensity of two pixels is lower or equal to
    measuring the intensity in a single pixel...
  - would add more information about the place where the robot may be
- Does the same hold if we know the location? **NO** (not clear if right)
  - the location is the cause
  - but depends I think if the locations is the current one or from the previous
    time instant or something like that...

### Local Semantics

- Each node is conditionally independent from its non-descendants given its
  parents

### Global Semantics

- Each node is independent from the rest, given
  - its parents
  - its children
  - the parents of its children

### Using Semantics (in the case of $P\left(S^A|I_7^{1,1}\right)=?$)

- Elevator is disconnected thus independent
- If we would know the location, the task would be easy
- The only consequence of the location that we can observe is $I^{1,1}$
- So, we can compute $P\left(L|I_7^{1,1}\right)$ ignoring all the rest
  - Indeed, all other variables would be redundant considering the Bayes network
  1. chain rule on $P\left(I_7^{1,1}|L\right)$ and $P\left(L\right)$ to get
     $P\left(L,I\right)$
  2. conditioning on $P\left(L,I\right)$ to get $P\left(L|I\right)$
- Finally, determine signal strength from improved location estimate
  $L|I_7^{1,1}$
  - using the chain rule on $P\left(S^A|L\right)$ and $P\left(L|I\right)$

# References

- https://www.cs.ubc.ca/~murphyk/Bayes/bnintro.html (tutorial on Bayes nets and
  graphical networks)
