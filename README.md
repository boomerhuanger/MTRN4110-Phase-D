# MTRN4110 Phase D: E-puck goes brrrrrrrrrr 

## Prerequisites (assuming a Windows setup)

1. Webots 2021a and Anaconda 2021.05 are both already installed on your computer. If not, they can be downloaded respectively via the following links: 
https://github.com/cyberbotics/webots/releases 
https://www.anaconda.com/products/individual

1. In addition, the Webots path environment variable should already be setup. If not, this can be done following these steps: [Adding webots to Path environment variable](#Adding-'webots'-to-Path-environment-variable)

2. A conda virtual environment should also be created and installed with the packages in the `programs/requirements.txt`


## Running the Program
1. Open the conda virtual environment `conda activate mtrn4110`.
2. Run the python file `python programs/run_phases.py` with default settings. Use `python programs/run_phases.py -h` to see optional additional arguments.
3. When running the program, a window with the intermediate results will pop-up. The program will pause on each image, and pressing any key will proceed to the next image.
4. Once the first phase has run, you are prompted to press any key to launch Webots and the remaining phases.
5. Once Webots will open, you can choose what mode what to run based on user input. 
6. Closing Webots will close the program, and then running `conda deactivate` in the terminal to close the virtual environment



## Adding 'webots' to Path environment variable
1. Open the file location of Webots by right-clicking it from your start menu and pressing "Open File Location". A window should opens as shown below:
<p align="center">
  <img src="https://user-images.githubusercontent.com/42131486/130306301-08f379ab-6990-4a06-bd60-fc591913e3d9.PNG" />
</p>

2. Right click the 'Webots' icon and press "Open File Location". This will take you to the follow window. 
<p align="center">
  <img src="https://user-images.githubusercontent.com/42131486/130306633-574d8ded-0db1-44c0-902b-07eb91629b29.png" />
</p>

3. From here, right click the top taskbar and select "Copy address as text". 
4. Now, go to "Edit System Environment Variables". This can be done by searching that in your search bar or by navigating to your control panel. You should reach this window: 
<p align="center">
  <img src="https://user-images.githubusercontent.com/42131486/130306848-5fcf68b9-7b93-447b-86d3-d1c603cadd7b.png" />
</p>

5. Press on "Environment Variables" near the bottom. Now, it should display the following:
<p align="center">
  <img src="https://user-images.githubusercontent.com/42131486/130307131-51adc281-1d0a-4140-94e9-3465ee289495.png" />
</p>

6.  Press 'Edit' underneath user variables and then add a new variable, pasting in the address that was copied from before. 


