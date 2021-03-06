# Optimal Drone Recharging Scheduling for Wireless Sensor Power Transfer and Data Collection
This project aims at solving Optimization Problem in scheduling recharing tasks for Power Delivery Vehicles (PDVs) under the Wireless Rechargeable Sensor Network. 

<img src='Figure/gif_demo.gif' width="700" height="600">

If you are interested, please refer to this paper for more information:
```
@ARTICLE{9402261,
  author={Qian, Qiuchen and Pandiyan, Akshayaa Y. S. and Boyle, David E.},
  journal={IEEE Access}, 
  title={Optimal Recharge Scheduler for Drone-to-Sensor Wireless Power Transfer}, 
  year={2021},
  volume={9},
  number={},
  pages={59301-59312},
  doi={10.1109/ACCESS.2021.3073076}}
```

## Brief Index
- Source code can be found in [Code directory](Code).
- Example input and output files are stored in [input](input) and [output](output).
- Documentation of C++ code can be found in [Documentation directory](Code/Documentation).
- Experimental results achieved by author team can be found in [Result directory](Result).
- Visualization of complete flight paths can be found in [VisualResult directory](Result/VisualResult).
- Final report can be found in [FinalReport directory](FinalReport).
- DEMO figures used for readme files can be found in [Figure directory](Figure).

## Prerequisites
This project adopted:
- [MSVS 2019](https://visualstudio.microsoft.com/zh-hans/vs/) & ISO C++17 language standard
- [Jupyter Notebook](https://jupyter.org/) & Python3 
*WARNING!! Lower language standard may lead to build errors. Please check your compiler and its version!* 

## Quick Start with console user interface
1. Create empty folder structure: (Note that the left integer `|0---` means the depth level of the directory or file)
```
	|0---- project/
	|1-------- input/
	|2------------ initial_guess/
	|1-------- output/
	|2------------ sub_path/
	|1-------- sys/
```
2. Open MSVS -> `Create a new project` -> `Empty Project` -> select `Location` at a created directory `project/`
3. Place the input file (.csv) in `input/` and [source code](Code/ensemble_system) in `sys/`
4. Open `Solution Explorer` in MSVS, `Add` -> `Existing Item` all files (.h) to `Header Files` 
5. `Add` -> `Existing Item` all files (.cpp) to `Source Files`
6. In `Solution Explorer`, open `Properties` of `sys`
7. In `General`, set `C++ Language Standard` to `ISO C++17 Standard (std:c++17)`
8. In `C/C++`, set `SDL checks` to `No (/sdl-)`
9. In `main.cpp`, uncomment user interface usage code in the main function
10. `Bulid` this project
11. Run the executable file (.exe)


## Normal Procedure
### Set up Folder Structure
To build the project successfully, the following folder structure are **recommended**. Otherwise, users are required to change default file directory in the codes. The structure includes **all files may be required**. In pratice, some files may not be needed! The directory structure to store code and I/O data should be:

```
	|0--- project/
	|1------- input/						# Place input files
	|2----------- input<case_id>.csv				# Raw sensor info
	|2----------- ...						# Other cases
	|2----------- initial_guess/					# Place initial sub-path solutions
	|3--------------- case<case_id>/				# Each case should have a separte directory 
	|4------------------- pop<population_id>_pdv<pdv_id>.txt	
	|4------------------- ...
	|3--------------- ...
	|2----------- naive/
	|3--------------- case<case_id>/				# Each case should have a separte directory  
	|4------------------- sub_path<pdv_id>.csv
	|4------------------- ...
	|3--------------- ...	
	|1------- output/						# Place output files
	|2----------- final_info.csv								
	|2----------- final_output.csv
	|2----------- sub_path/						# Calculated sub-paths solutions
	|3--------------- ga_path<pdv_id>.csv						
	|3--------------- ...
	|1------- naive_sol/						# Place naive solution codes
	|2----------- sim/						# Place system codes
	|3--------------- point.h							
	|3--------------- ...
	|2----------- output/						# Place generated output
	|3--------------- sum.csv
	|1------- alg/							# Place optimization algorithm codes
	|2----------- 1_data_generation.ipynb
	|2----------- genetic.h
	|2----------- ...
```
Note that:
- Name with `/` is a directory name (i.e. `output/`).
- Name with `...` means there are some similar directories or files.
- [1_data_generation.ipynb](Code/1_data_generation.ipynb) and [3_result_visualization.ipynb](Code/3_result_visualization.ipynb) should be placed under the same directory with algorithm header and source files. 
- Modifications of dierctory name or location are **NOT** recommanded. Please check algorithm I/O functions and I/O parameters in two `.ipynb` files if modifications are needed!
- Because there are 3 algorithms, the output file names are different (i.e. `ga_path0.csv`, `bh_path0.csv`, etc.).     

### Integrated system with user interface
All algorithms and system model are integrated within one project. Users can use that system with following steps:
1. Use MSVS to build and run [source code](Code/ensemble_system).
2. Enter commands in the console terminal (following given instructions).


## Run Simulation Separately
### A NAIVE SOLUTION (OPTIONAL): Scikit-learn K-Means Clustering Method + Shortest Next Job (SJN) as Baseline
The [data generation file](Code/1_data_generation.ipynb) also supports some typical clustering algorithms (i.e. K-Means, Spectral and Agglomerative Clustering) with scikit-learn library. The sub path file (i.e. [sub_path0.csv](Code/input/sub_path0.csv)) will be generated at `input/` directory for further usage. Then code in [pdv simulation](Code/pdv_simulation) is designed for recharging simulation of this situation.

*Quick Start for Clustering + SJN*: 
1. Implement *1. Input data generation* (**INCLUDING OPTIONAL STEP**).

    The data format of output sub path file: 
    ```
        x_pos           y_pos
        -672.08	    -193.15
        -371.66	    76
        -980.82	    -661
    ```
    *!!! NOTICE !!! The output file will be placed in `input/` directory*

2. Create a new project with MSVS and add all needed header and source files in [pdv_simulation](Code/pdv_simulation).
3. Open `C/C++`, set `SDL checks` to `No (/sdl-)`.
4. Match the marco `NODE_NUM` and `PDV_NUM` in [main.cpp of pdv_simulation](Code/pdv_simulation/main.cpp) with the number of sensor nodes.
5. Set wanted minimum charge number for single PDV.
6. Build and run the code.
7. Find recharging summary in `output/` directory.

***Clustering Results Comparison DEMO*** <br/>
<img src='Figure/comparison of kmeans and spectral.png' width="950" height="500">

### A NOVEL SOLUTION: Generate Solutions with Optimization Algorithms
Users can solve the drone recharging scheduling problem with provided optimization algorithms as well (Genetic, Black Hole, Simulated Annealing Algorithm). But users are strongly recommended to clear all input and output files (but remain folder structure). Please note that there are three main parts of the project. The instructions of executing them can be found as below (*please check [tips for users](Code/README.md) in advance as well*):

#### Step 1. Input data generation
Python is used to generate CSV file of input data. In [data generation file](Code/1_data_generation.ipynb), users can change the variable `sensor_node_num` to automatically generate specific number of sensor nodes. There are many other hyper-paramters like `p_v_max` (set maximum voltage level), `scale_x_max` (set the maximum value of x axis), etc. Plese see comments in [data generation file](Code/1_data_generation.ipynb) for more details!

**OR** directly modify [inputs.csv](Code/input/inputs.csv) with below format:

```
    x_pos	y_pos	p_flag	   volts	   weights
    4851	16711	  0	    2.010605073	     9
    26894	110695	  1	    3.002501137	     3
    117004	50480	  0	    1.966313626	     8
```

*!!! NOTICE !!! The system allow users to manually input information of sensors but the format should be same with above `.csv` file*

#### Step 2. Calculate the optimal solution with GA, BH or SA with SJN strategy
Before executing optimization algorithm codes, users can tune hyper-parameters according to specific requirements, which are defined at the top of [genetic.cpp](Code/iterated_GA/genetic.cpp), [blackhole.cpp](Code/iterated_BH/blackhole.cpp) and [annealing.cpp](Code/iterated_SA/annealing.cpp). For example, in `genetic.cpp`, changing `cr` to 60 can reduce the possibility of crossover. Or changing `init_temp` to 1.3e3 can increase iterations. Please check related comments or documentations for more details! 

*Quick Start for Optimization Algorithms*: (Here take GA as an example)
1. Empty all input and output files but remain folder structure.
2. Implement *1. Input data generation* (**EXCLUDE OPTIONAL STEP**).
3. Create a new project with MSVS and add all needed header and source files in [iterated_GA](Code/iterated_GA).
4. Open `Configuration Properties`, set `C++ Language Standard` to `ISO C++17 Standard (std:c++17)`.
5. Open `C/C++`, set `SDL checks` to `No (/sdl-)`.   
6. Set wannted hyper-parameters and change the marco `NODE_NUM` in `main.cpp` to correct number.
7. Build and run the code.
8. Find recharging summary (two CSV files) in `output/` directory. (If there exists output files due to other algorithm execution, former results will be ***covered***.)

***Optimization Algorithm Results Comparison DEMO*** <br/>
<img src='Figure/BH_vs_GA.png' width="850" height="900">

#### Step 3. Result visualization
Visualization can be done using generated sub path files in `output/` directory (i.e. [bh_path0.csv](Code/output/sub_path/bh_path0.csv)). Please check comments in [visualization code](Code/3_result_visualization.ipynb) for more information! Visualized experimental results by author team can be found in [VisualResult directory](Result/VisualResult). Note that for the final step (histogram generation), the code needs a file called [ensemble_output.csv](Code/output/ensemble_output.csv), which is the summary of algorithms under different scenarios.

***Complete Flight Path DEMO*** <br/>
<img src='Figure/complete_flight_path.gif' width="550" height="550">

To better visualize summary of results, histograms can be generated as well in [visualization code](Code/3_result_visualization.ipynb).

***Summary of Total PDV Energy Cost DEMO*** <br/>
<img src='Figure/energy_cost_summary.png' width="550" height="400">

***Summary of Total Recharged Energy DEMO*** <br/>
<img src='Figure/recharge_energy_summary.png' width="550" height="400">

Note that the `Case` (x-axis) here means tested 9 scenarios (different scales of networks). Details are stated in the report.


## Additional Part
### Execution time of algorithms
The execution time of the optimization algorithm is counted through `std::chrono::high_resolution_clock` (unit: ms). The time counter starts after calculation of inital guesses and ends when finding the solution with best fitness metric. The duration time will be recorded to [final_info.csv](Code/output/final_info.csv) as well (`alg_time`).

### Verification of code
This project adopts the [Unit Test tool of Miscrosoft](https://docs.microsoft.com/en-us/visualstudio/test/unit-test-your-code?view=vs-2019) to verify code implementations. We provide [sample code](Code/unit_test/unit_test.cpp) to test algorithm module and system model. Please check [the tutorial from Miscrosoft Doc](https://docs.microsoft.com/en-us/visualstudio/test/getting-started-with-unit-testing?view=vs-2019) to run tests correctly. Notice that:

- Because all included optimization algorithms share similar system model, in this case, author team only tested functionality of Genetic Algorithm and system model as the main example.
- Because random number wil be used in all included optimization algorithms, small errors will be allowed in some particular tests (i.e. error = 5 when involving random number). Normally, if there is no random variable, the allowed tolerance should be 1e-10.

*Quick Start for test code*: (Here take GA sample test as an example)
1. Create a new project with MSVS and add all needed header and source files in [iterated_GA](Code/iterated_GA).
2. Open `Solution Explorer` -> right click the first line `Solution 'xx'(your solution name)` -> `Add` -> `New Project` -> Select `Native Unit Test Project`.
3. Add test code to source files (Note that MSCV may have self-loading `pch.h` and `pch.cpp`, which were not used in the sample test code)
4. Include correct `.h` and `.cpp` at the top of test code
5. In your test project -> right click `References` -> `Add Reference` -> You should see the project (`iterated_GA` in this case) in the list -> select it -> click `OK`.
6. Build all solutions.
7. Run all tests in `Test Explorer`.


## Complementary Experiments
### Test impact of PDV numbers
To explore the impact of required number of PDVs to average throughput, we also did several complementary experiments with increasing number of assigned PDVs. Below 3 figures show the experiment result:

<img src='Figure/throughput-comp.png' width="550" height="1150">

The first figure is execution summary of all cases. For the case with the lowest average throughput (6 & 9), we can observe that assigning 6 PDVs to case 6 is the most efficient. For case 9, it may take 7 or even 8 PDVs to achieve the highest efficiency. Thus, it's necessary to assign appropriate numbers of PDVs to execute the recharging task.

### Test impact of iteration numbers
To explore the impact of iterations, we also tried to reduce the computational cost. Visualized results can be found in [less_iters](Result/less_iters) directory. 

Conclusion: As the number of iterations increases, the optimal degree of the calculated solution tends to be 'stable'. The algorithms don't need a large iteration number to be 'stable'.

### Test impact of inital solution generation methods
Based on [the research of R.-H Cheng et al.](https://www.researchgate.net/publication/330474652_A_Genetic_Approach_to_Solve_the_Emergent_Charging_Scheduling_Problem_Using_Multiple_Charging_Vehicles_for_Wireless_Rechargeable_Sensor_Networks), it was proven that different initialization methods can have different effects on final solutions. Thus, we designed two initialization methods to test:
1. Random initialization: Define a threshold integer `k`, find a random integer `r < k`. Starting from the BS, add the r-th nearest target node into the sub-path of the PDV.
2. Clustering initialization: Apply K-Means clustering first as initial guess solutions.
The result shows solutions initialized by method 2 is *much better* than those initialized by method 1. See below tables for detail comparisons:

<img src='Figure/rand_compare.png'>
<img src='Figure/kmeans_compare.png'>

### Define recharging efficiency of single PDV
single PDV recharging efficiency is defined as *total recharged energy of WSN* divided by *PDV energy cost*. The recharging efficiency of PDV0 and PDV1 of 3 optimization algorithms and K-Means clustering is summarized as below figure:

<img src='Figure/efficiency_compare.jpg'>


## Further Updates
Please note that this is a open-source project with continus updates. Currently, we consider the next step to:
1. Develop an IDE with GUI
2. Convert to 3D space
3. Consider parallel computing of all algorithms


## Other Information
### Code Documentation
Documentation files are generated through [Doxygen](https://www.doxygen.nl/index.html). Through index file [ga_index](Code/Documentation/ga_doc/ga_index.lnk), [bh_index](Code/Documentation/bh_doc/bh_index.lnk) and [sa_index](Code/Documentation/sa_doc/sa_index.lnk), users can check the code structure, function usage, variable definitions, etc. If content of the index file cannot display properly, please check corresponding source file `index.lnk` in `html\` directory.

### Model Selection
Please note that the original system model is based on [the research of A. Y. S. Pandiyan et al](https://www.researchgate.net/publication/341077461_Optimal_Energy_Management_of_Two_Stage_Energy_Distribution_Systems_Using_Clustering_Algorithm). However, this project made several modifications to make the system model more practical. More details can be found in related paper and [introduction of model comparison](Model_Comparison.pdf). **Quick Start with console user interface** section adopts *original* model. Other sections will adopt *new* model. Note that in the paper, the system model adopts the original one. More technical descriptions can be found in the paper.

### Slide Demonstration
Presentation slides can be seen [here](irp_presentation.pptx).

### Contact Us: 
Mr. Qiuchen Qian - qiuchen.qian19@imperial.ac.uk  
Ms. Akshayaa Pandiyan - a.pandiyan@imperial.ac.uk  
Dr. David Boyle - david.boyle@imperial.ac.uk  
