# Collision-detection-for-a-CPU-FPGA-heterogeneous-System
Implementation of a collision detection kernel in Verilog to be used for collision detection algorithms in a CPU-FPGA heterogeneous system. The design implements the Spheres collision detection algorithm implemented on the ODE (Open Dynamics Engine Platform), we also provide benchmarks for testing and comparing the design and the ODE processing speed.

   The .v files compose the Verilog design for the Sphere Collision detection Kernel.
   The dCollideSpheres.cpp is the application implemented with the AAL Framework libraries to send data to and fetch results from the FPGA.
   This design is made for the AAL framework and should be used with the Intel SPL2 RTL.
   The design also makes use of the FPGA DSP blocks for the floating point operations (with exception of the square root operation) which should be generated with the Altera Quartus 13.1 tool.
   
   The folder AAL app contains the AAL C++ application.
   The folder ODE simulations contains the parametrizable test sets described in a C++ file, these are the spheres diamonds used to fetch timing results for the paper.
   The folder Verilog Design contains the full verilog designfor use with the HARP platform.
