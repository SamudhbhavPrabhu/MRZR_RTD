# MRZR_RTD

Reachability-based Trajectory Design for autonomous navigation of MRZR in an unstructured environment (off-road terrain in UE4 simulator).

This repository is based on the master repository: https://github.com/skvaskov/RTD , where the RTD has been applied on segway and rover agents.

Please refer the paper, ["Bridging the Gap Between Safety and Real-Time Performance in Receding-Horizon Trajectory Design for Mobile Robots"](https://arxiv.org/abs/1809.06746), which introduces **Reachability-based Trajectory Design**, or RTD.

Check out the [tutorial](https://github.com/skousik/RTD_tutorial) to get a gentler introduction to the method. 

# Navigation Arena setup in UE4 for real-time testing

Arena1:

<img src="https://github.com/SamudhbhavPrabhu/MRZR_RTD/blob/main/RTD/figures/Arena1.png" width="240" height="200">

Arena2:

<img src="https://github.com/SamudhbhavPrabhu/MRZR_RTD/blob/main/RTD/figures/Arena2.PNG" width="240" height="200">

#### Getting Started

All the code in this repository runs in MATLAB (R2018a or newer).

This repository contains almost everything you need to start playing with RTD. The other things you will need are:

1. To run the simulations and most of the code, you'll need a copy of our [simulator](https://github.com/skousik/simulator) repository on your MATLAB path.

2. To run the reachable set computations, you'll need [MOSEK](https://www.mosek.com/) (free for academic use).


#### Citing this Work

Please cite the [paper](https://arxiv.org/abs/1809.06746). Also, check out other papers by the RTD authors: [RTD on a car](https://arxiv.org/abs/1902.01786), [RTD on a drone](https://arxiv.org/abs/1904.05728), and [RTD on an arm](https://arxiv.org/abs/2002.01591).

#### RTD Authors

Sean Vaskov, Shreyas Kousik, Hannah Larson, and Ram Vasudevan

