------------------------------------------------------------------------------------------
To compile: make
------------------------------------------------------------------------------------------
To run: First, place yourself in the 'bin' directory.

./uc2urdf -input oldurdf.xml -uc file.uc -output newurdf.xml

oldurdf.xml: The "name" of the original urdf file built from tf data
The program will automatically search for this file in ../usarsim_inf/urdf

file.uc: The name of the uc file used to extract link and joint data
The program will automatically search for this file in ../etc/

newurdf.xml: The "name" of the output urdf file
The program will automatically create the new file in ../etc/
------------------------------------------------------------------------------------------
Documentation: To generate the documentation (html and latex) for the code, 
place yourself in the 'doc' directory and run 'doxygen Doxyfile'.
------------------------------------------------------------------------------------------

