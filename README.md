# OxfordBrookesRacing-Tools
Oxford Brookes Racing Tools from data viewers to setup comparers.

Current program list:
- OBRLapSim
- Setup Compare
- Setup Creator

Setup directions:
Once downloaded place the files into a safe easy to find location. Then set the main folder to your MATLAB path. Instructions for doing this can be found here: https://www.mathworks.com/help/matlab/matlab_env/add-remove-or-reorder-folders-on-the-search-path.html. After setting the path you can run any of the functions

Under OBRLapSim is Jay's lap simulation code developed in the 2017 to 2018 season, this was modified to change the simulations from scripts to functions. This allows the possibility to run sweep functions. The simulations include:
- Acceleration
- Skidpad
- Cornering Simulation
- J turn simulation
- Open Track
- Closed Track
Under the same folder other functions can be found which are used in the other scripts.

The Setup Creator GUI is an easy way to create the .mat setups used in Jay's lap simulations. These are created from the excel files found in OBRLapSim/Car Data. If you would like to modify a variable in the setup you will need to modify the value in the importable excel file, then to compile the setup to a .mat file you need to run the class setupCreater by typing "CS = setupCreator()" into your console.

Setup Compare holds all of the scripts for the Setup Compare class. This class can be run by typing "SC = setupCompare()" into your MATLAB console, this should open a GUI where you can select setups that you created and compare them. Instructions on how this works can be found in the same folder.