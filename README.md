# Instructions

## Project layout
All of my code is in ```solver/``` to keep things clean. The file ```solver.cpp``` is the main executable.

## Building

```bash
# Dependencies.
apt update
apt install cmake build-essential git

# Should clone and install GoogleTest if not on the system.
mkdir build && cd build
cmake ..
make
```

## Running the solver

The build should place a ```run_solver``` executable in the ```build/solver/``` directory.

Run it with (e.g)
```bash
./build/solver/run_solver problem1/problem.txt problem1/solution.txt
```

## Running tests

I added some tests in ```test.cpp```, mainly for my own debugging. There is an executable called ```run_tests``` in the ```build/solver/``` directory also.

I also added some problems in ```large_problems/``` to test my routing algorithm at a larger scale. Due to the sparsity of these problems (no obstacles) and my design choices, my solver wastes a lot of space for these, but still runs in a reasonable amount of time up to a 10,000 x 10,000 board.
