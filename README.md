# Bioreactor
## Understanding the project structure

The main file is called bioreactor.ino. This is the only file that should have the setup and loop functions defined.  
Each subsystem has its own file eg. ph.ino  
In each of these files there are placeholder functions (that you need to implement for data logging!)  
These files also contain a setupSubsystem and loopSubsystem function. Please move the code that you would have had in the setup and loop functions into these functions, which are then called in the main file.